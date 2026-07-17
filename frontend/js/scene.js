// scene.js — Three.js scene manager. Receives decoded scene ops from ws_client
// and maintains Three.js objects keyed by id. Supports ops: add/update/remove
// for kinds: points, box, line, label, mesh.
//
// Design: one THREE.Points object per 'points' id. On 'update' we replace its
// BufferGeometry (cheap enough at voxel-downsampled sizes ~10^5). Colors come
// as a separate attribute.

import * as THREE from '../vendor/three.module.js';
import { OrbitControls } from '../vendor/examples/jsm/controls/OrbitControls.js';

export class SceneManager {
  constructor(container) {
    this.objects = new Map();   // id -> THREE.Object3D
    this._labels = new Map();   // id -> {el} CSS2D-like (simple div overlay)

    // Scene + fog-free deep background.
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x181818);

    // Camera — orthographic-ish perspective, Z up to match ccenter's map frame.
    // NOTE: near must stay reasonably large. A tiny near (e.g. 0.005) crushes
    // depth-buffer precision (near/far ratio ~100k:1) and lets the camera dolly
    // so close that points balloon into full-screen color blocks at certain
    // angles — the "giant color block from some viewpoints" bug.
    this.camera = new THREE.PerspectiveCamera(55, 1, 0.1, 1000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(8, -8, 6);

    // Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);

    // Controls
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0, 0, 0);
    // Keep the camera from dollying so close that points (sizeAttenuation)
    // balloon into full-screen blocks, or so far that depth precision collapses.
    this.controls.minDistance = 0.3;
    this.controls.maxDistance = 200;

    // Ground grid + axes for orientation (matches ccenter's create_grid).
    this._addReferenceGrid();

    // Resize handling
    this.container = container;
    this._onResize = () => this.resize();
    window.addEventListener('resize', this._onResize);
    this.resize();
    this._setupPick();

    // Stats for the status bar
    this.stats = { objects: 0, points: 0, fps: 0 };
    this._frames = 0;
    this._fpsT = performance.now();
  }

  _addReferenceGrid() {
    const grid = new THREE.GridHelper(40, 40, 0x3c3c3c, 0x2a2a2a);
    grid.rotation.x = Math.PI / 2;  // lie flat on XY (Z up)
    grid.material.opacity = 0.5;
    grid.material.transparent = true;
    this.scene.add(grid);
    const axes = new THREE.AxesHelper(1.5);
    this.scene.add(axes);
  }

  resize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  // --- click-to-set-target: raycast to the z=0 ground plane ---
  // Shift+click in the 3D viewport picks a world (x,y) for navigation. The
  // app wires onPick to send a set_target WS message.
  onPick = null;  // callable(worldX, worldY)
  _setupPick() {
    const raycaster = new THREE.Raycaster();
    const ndc = new THREE.Vector2();
    const ground = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0); // z=0 plane
    this.renderer.domElement.addEventListener('click', (e) => {
      if (!e.shiftKey || !this.onPick) return;
      const rect = this.renderer.domElement.getBoundingClientRect();
      ndc.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      ndc.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(ndc, this.camera);
      const hit = new THREE.Vector3();
      if (raycaster.ray.intersectPlane(ground, hit)) {
        this.onPick(hit.x, hit.y);
      }
    });
  }

  // --- main render loop ---
  start() {
    const tick = () => {
      this._raf = requestAnimationFrame(tick);
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      this._frames++;
      const now = performance.now();
      if (now - this._fpsT >= 1000) {
        this.stats.fps = Math.round(this._frames * 1000 / (now - this._fpsT));
        this._frames = 0; this._fpsT = now;
      }
    };
    tick();
  }

  // --- op application ---
  applyOps(ops) {
    for (const op of ops) this._applyOp(op);
    this._recount();
  }

  applyPointsOp(op) { this._applyOp(op); this._recount(); }
  applyMeshOp(op)   { this._applyOp(op); this._recount(); }

  _applyOp(op) {
    if (op.op === 'remove') {
      const o = this.objects.get(op.id);
      if (o) {
        this.scene.remove(o);
        if (o.geometry) o.geometry.dispose();
        if (o.material) o.material.dispose();
        this.objects.delete(op.id);
      }
      return;
    }
    // add or update — update path replaces geometry/material data.
    if (op.kind === 'points') this._setPoints(op);
    else if (op.kind === 'box') this._setBox(op);
    else if (op.kind === 'line') this._setLine(op);
    else if (op.kind === 'mesh') this._setMesh(op);
    else if (op.kind === 'arrow') this._setArrow(op);
  }

  _setPoints(op) {
    let pts = this.objects.get(op.id);
    const positions = op.positions;   // Float32Array (N*3)
    if (!positions || positions.length === 0) {
      if (pts) { this.scene.remove(pts); this.objects.delete(op.id); }
      return;
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    if (op.colors && op.colors.length * 3 >= positions.length) {
      // colors is Float32Array N*3 too; guard length
      geo.setAttribute('color', new THREE.BufferAttribute(op.colors, 3));
    }
    // Auto-fit near/far so large scenes stay visible.
    geo.computeBoundingSphere();
    const mat = new THREE.PointsMaterial({
      size: op.point_size || 0.04,
      vertexColors: !!(op.colors),
      sizeAttenuation: true,
    });
    if (pts) {
      pts.geometry.dispose();
      pts.material.dispose();
      pts.geometry = geo;
      pts.material = mat;
    } else {
      pts = new THREE.Points(geo, mat);
      this.scene.add(pts);
      this.objects.set(op.id, pts);
    }
    // First-time auto-frame: point camera at the data.
    if (!this._framed) this._autoFrame(geo.boundingSphere);
  }

  _setBox(op) {
    let box = this.objects.get(op.id);
    const size = op.size || [1, 1, 1];
    if (!box) {
      const geo = new THREE.BoxGeometry(size[0], size[1], size[2]);
      const mat = new THREE.MeshBasicMaterial({ color: rgb(op.color) });
      box = new THREE.Mesh(geo, mat);
      this.scene.add(box);
      this.objects.set(op.id, box);
    }
    const T = op.pose || identity();
    box.matrix.fromArray(flatten(T));
    box.matrixAutoUpdate = false;
  }

  _setArrow(op) {
    let arrow = this.objects.get(op.id);
    const T = op.pose || identity();
    const px = T[0][3], py = T[1][3], pz = (T[2] || [0,0,0])[3] || 0;
    const dx = T[0][0], dy = T[1][0];
    const color = rgb(op.color || [1, 1, 0]);
    const length = op.length || 0.5;
    if (!arrow) {
      arrow = new THREE.ArrowHelper(
        new THREE.Vector3(dx, dy, 0).normalize(),
        new THREE.Vector3(px, py, pz + 0.3),
        length, color, length * 0.4, length * 0.2);
      this.scene.add(arrow);
      this.objects.set(op.id, arrow);
    } else {
      arrow.position.set(px, py, pz + 0.3);
      arrow.setDirection(new THREE.Vector3(dx, dy, 0).normalize());
      arrow.setColor(color);
    }
  }

  _setLine(op) {
    // Simple line from positions; rebuilt each update (lines are small).
    let line = this.objects.get(op.id);
    const pos = new Float32Array(op.positions.flat ? op.positions.flat() : op.positions);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const mat = new THREE.LineBasicMaterial({ color: rgb(op.color) });
    if (line) { line.geometry.dispose(); line.material.dispose();
                line.geometry = geo; line.material = mat; }
    else { line = new THREE.Line(geo, mat); this.scene.add(line); this.objects.set(op.id, line); }
  }

  _setMesh(op) {
    let mesh = this.objects.get(op.id);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(op.positions, 3));
    geo.setIndex(new THREE.BufferAttribute(op.indices, 1));
    if (op.colors) geo.setAttribute('color', new THREE.BufferAttribute(op.colors, 3));
    geo.computeVertexNormals();
    const mat = new THREE.MeshStandardMaterial({
      vertexColors: !!op.colors, flatShading: true,
      metalness: 0.1, roughness: 0.8,
    });
    if (mesh) { mesh.geometry.dispose(); mesh.material.dispose();
                mesh.geometry = geo; mesh.material = mat; }
    else { mesh = new THREE.Mesh(geo, mat); this.scene.add(mesh); this.objects.set(op.id, mesh); }
    if (!this._framed) { geo.computeBoundingSphere(); this._autoFrame(geo.boundingSphere); }
  }

  _autoFrame(sphere) {
    if (!sphere || !isFinite(sphere.radius) || sphere.radius === 0) return;
    this.controls.target.copy(sphere.center);
    const r = sphere.radius;
    const dist = r / Math.tan((this.camera.fov * Math.PI / 180) / 2) * 1.2;
    const dir = new THREE.Vector3(1, -1, 0.7).normalize();
    this.camera.position.copy(sphere.center.clone().add(dir.multiplyScalar(dist)));
    // Tie near/far to the actual data extent instead of a fixed tiny near and a
    // forced far=500. The previous near=dist/1000 + far=max(500,...) gave a
    // near/far ratio of ~50k:1, which exhausted depth precision and made the
    // cloud render as a solid wall/block at certain viewing angles.
    this.camera.near = Math.max(0.1, (dist - r) * 0.2);
    this.camera.far = Math.max(500, (dist + r) * 6);
    this.camera.updateProjectionMatrix();
    this._framed = true;
  }

  _recount() {
    let pts = 0;
    for (const o of this.objects.values()) {
      if (o.isPoints && o.geometry.attributes.position) {
        pts += o.geometry.attributes.position.count;
      }
    }
    this.stats.objects = this.objects.size;
    this.stats.points = pts;
  }
}

function rgb(arr) {
  if (!arr) return 0xffffff;
  return new THREE.Color(arr[0], arr[1], arr[2]).getHex();
}
function identity() {
  return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];
}
function flatten(m) {
  return [].concat(...m.map(row => Array.isArray(row) ? row : [row]));
}
