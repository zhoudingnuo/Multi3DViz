// plugin_panel.js — plugin catalog + instance management UI.
// Two modes per plugin type (from catalog `multiple` flag):
//   - single-instance (services/tools): a checkbox toggles enable/disable.
//   - multi-instance (sources/displays): a "+ add" button creates instances;
//     each live instance is a row with its own property editor + remove btn.
//
// Talks to the backend via ws_client. All ops key by instance_id:
//   add_instance {name} → creates instance, returns instance_id
//   disable_plugin {instance_id} / set_property {instance_id, key, value}

import { icon } from './icons.js';

export class PluginPanel {
  constructor(rootEl, ws) {
    this.root = rootEl;
    this.ws = ws;
    this.catalog = [];        // [{name, category, multiple, properties, ...}]
    this.instances = [];      // [{name, instance_id, category, properties}]
    this.expanded = new Set(); // instance_ids whose property editor is open
    this._installStates = {}; // {pluginName: {phase, pct, msg}} — in-app dep install progress
    this._render();
  }

  setCatalog(catalog) {
    this.catalog = catalog || [];
    this._render();
  }

  setState(instances) {
    this.instances = instances || [];
    // Auto-expand instances that have no properties shown yet, so the user
    // sees what each instance is configured for (e.g. which robot).
    this.instances.forEach(i => { if (!this.expanded.has(i.instance_id) && i.instance_id.endsWith('#1')) this.expanded.add(i.instance_id); });
    this._render();
  }

  _render() {
    const order = { source: 0, display: 1, tool: 2, service: 3 };
    const cats = { source: 'Sources', display: 'Displays', tool: 'Tools', service: 'Services' };
    const sorted = [...this.catalog].sort((a, b) =>
      (order[a.category] ?? 9) - (order[b.category] ?? 9) || a.name.localeCompare(b.name));

    let html = '';
    let lastCat = null;
    for (const p of sorted) {
      if (p.category !== lastCat) {
        html += `<div class="section-title">${cats[p.category] || p.category}</div>`;
        lastCat = p.category;
      }
      const live = this.instances.filter(i => i.name === p.name);
      if (p.multiple) {
        // Multi-instance: header row with +add button, then one row per instance.
        html += `<div class="plugin-type-row">
          <span class="ptype-name">${esc(p.name)}</span>
          <button class="btn-icon add-inst" data-add="${esc(p.name)}" title="Add instance">${icon('play', 13)}</button>
        </div>`;
        for (const inst of live) {
          html += this._instanceRow(inst, p, true);
        }
      } else {
        // Single-instance: checkbox toggle. If the plugin has missing optional
        // deps (e.g. Semantics needs torch), show an "Install" button.
        const on = live.length > 0;
        const inst = live[0];
        const missing = p.missing_deps || [];
        const installState = this._installStates[p.name];
        let installBtn = '';
        if (missing.length > 0 && !installState) {
          installBtn = `<button class="btn-icon install-dep" data-install="${esc(missing[0])}" data-plugin="${esc(p.name)}" title="Install ${esc(missing.join(', '))}">⚙</button>`;
        } else if (installState) {
          const pct = installState.pct || 0;
          const phase = installState.phase || '';
          if (phase === 'done') {
            installBtn = `<span class="install-done" title="Installed">✓</span>`;
          } else if (phase === 'error') {
            installBtn = `<button class="btn-icon install-dep err" data-install="${esc(missing[0] || 'torch')}" data-plugin="${esc(p.name)}" title="Retry: ${esc(installState.msg || 'failed')}">⚠</button>`;
          } else {
            installBtn = `<span class="install-progress" title="${esc(installState.msg || phase)}">${Math.round(pct)}%</span>`;
          }
        }
        html += `<div class="plugin-row ${on ? 'on' : ''}" data-name="${esc(p.name)}">
            <input type="checkbox" ${on ? 'checked' : ''} data-toggle="${esc(p.name)}" ${missing.length > 0 ? 'disabled title="Install deps first"' : ''}/>
            <span class="name">${esc(p.name)}</span>
            ${installBtn}
            <span class="cat">${esc(p.category)}</span>
          </div>`;
        if (inst) html += this._instanceRow(inst, p, false);
      }
    }
    this.root.innerHTML = html;
    this._wire();
  }

  _instanceRow(inst, schema, removable) {
    const exp = this.expanded.has(inst.instance_id);
    const label = inst.instance_id.includes('#') ? inst.instance_id : inst.name;
    // Show the robot_id prop inline if present (quick visual of which robot).
    const rid = inst.properties && inst.properties.robot_id;
    const ridTag = rid ? `<span class="inst-robot">${esc(rid)}</span>` : '';
    const rmBtn = removable
      ? `<button class="btn-icon danger rm-inst" data-rm="${esc(inst.instance_id)}" title="Remove">${icon('trash', 13)}</button>`
      : '';
    let html = `<div class="instance-row" data-inst="${esc(inst.instance_id)}">
        <span class="inst-dot"></span>
        <span class="inst-label" data-expand="${esc(inst.instance_id)}">${esc(label)}${ridTag}</span>
        ${rmBtn}
      </div>`;
    if (exp) html += this._renderProps(inst, schema);
    return html;
  }

  _renderProps(inst, schema) {
    const vals = inst.properties || {};
    const defaults = {};
    Object.entries(schema.properties || {}).forEach(([k, s]) => { defaults[k] = s.default; });
    const merged = { ...defaults, ...vals };
    let rows = '';
    const groups = {};
    Object.entries(schema.properties || {}).forEach(([k, s]) => {
      const g = s.group || 'Properties';
      (groups[g] = groups[g] || []).push([k, s]);
    });
    for (const [g, items] of Object.entries(groups)) {
      rows += `<div class="prop-group">${esc(g)}</div>`;
      for (const [k, s] of items) {
        rows += `<label>${esc(s.label || k)}${this._input(inst.instance_id, k, s, merged[k])}</label>`;
      }
    }
    return `<div class="props" data-props="${esc(inst.instance_id)}">${rows}</div>`;
  }

  _input(instanceId, key, schema, val) {
    const t = schema.type;
    const d = v => esc(val == null ? '' : v);
    const di = esc(instanceId);
    const dk = esc(key);
    if (t === 'select') {
      const opts = (schema.options || []).map(o =>
        `<option value="${esc(o)}" ${o === val ? 'selected' : ''}>${esc(o)}</option>`).join('');
      return `<select data-instance="${di}" data-prop="${dk}">${opts}</select>`;
    }
    if (t === 'bool') {
      return `<input type="checkbox" data-instance="${di}" data-prop="${dk}" ${val ? 'checked' : ''}/>`;
    }
    if (t === 'float' || t === 'int') {
      const step = schema.step || (t === 'int' ? 1 : 'any');
      return `<input type="range" min="${schema.min ?? 0}" max="${schema.max ?? 1}" step="${step}"
                value="${d(val)}" data-instance="${di}" data-prop="${dk}"/>
              <span class="prop-val">${d(val)}</span>`;
    }
    return `<input type="text" value="${d(val)}" data-instance="${di}" data-prop="${dk}"/>`;
  }

  // --- in-app optional dependency install (torch for Semantics) ---
  setInstallProgress(p) {
    // p = {pkg, phase, pct, msg}. Find which plugin owns this dep.
    for (const item of this.catalog) {
      if ((item.missing_deps || []).includes(p.pkg)) {
        this._installStates[item.name] = { phase: p.phase, pct: p.pct || 0, msg: p.msg || '' };
        this._render();
        return;
      }
    }
    // Dep might already be satisfied after install — still show progress.
    this._installStates['_pending'] = { phase: p.phase, pct: p.pct || 0, msg: p.msg || '' };
    this._render();
  }

  setPluginStatus(s) {
    // s = {plugins: [{name, missing_deps, available}]}. Update catalog's
    // missing_deps so the Install button disappears when deps are met.
    if (!s.plugins) return;
    for (const st of s.plugins) {
      const item = this.catalog.find(c => c.name === st.name);
      if (item) {
        item.missing_deps = st.missing_deps || [];
        if (item.missing_deps.length === 0) delete this._installStates[st.name];
      }
    }
    this._render();
  }

  _wire() {
    // +add instance
    this.root.querySelectorAll('[data-add]').forEach(btn => {
      btn.addEventListener('click', () => {
        this.ws.request({ type: 'add_instance', name: btn.dataset.add });
      });
    });
    // install optional dependency (⚙ button)
    this.root.querySelectorAll('[data-install]').forEach(btn => {
      btn.addEventListener('click', (e) => {
        e.stopPropagation();
        const pkg = btn.dataset.install;
        const plugin = btn.dataset.plugin;
        this._installStates[plugin] = { phase: 'starting', pct: 0, msg: 'Starting...' };
        this._render();
        this.ws.request({ type: 'install_dependency', name: pkg });
      });
    });
    // remove instance
    this.root.querySelectorAll('[data-rm]').forEach(btn => {
      btn.addEventListener('click', () => {
        this.ws.request({ type: 'disable_plugin', instance_id: btn.dataset.rm });
      });
    });
    // single-instance toggle
    this.root.querySelectorAll('[data-toggle]').forEach(cb => {
      cb.addEventListener('click', (e) => {
        e.stopPropagation();
        const name = cb.dataset.toggle;
        if (cb.checked) this.ws.request({ type: 'enable_plugin', name });
        else this.ws.request({ type: 'disable_plugin', name });
      });
    });
    // expand/collapse instance props
    this.root.querySelectorAll('[data-expand]').forEach(el => {
      el.addEventListener('click', () => {
        const iid = el.dataset.expand;
        if (this.expanded.has(iid)) this.expanded.delete(iid);
        else this.expanded.add(iid);
        this._render();
      });
    });
    // property inputs
    this.root.querySelectorAll('[data-prop]').forEach(inp => {
      const iid = inp.dataset.instance;
      const key = inp.dataset.prop;
      const handler = () => {
        let val = inp.value;
        if (inp.type === 'number' || inp.type === 'range') val = parseFloat(val);
        if (inp.type === 'checkbox') val = inp.checked;
        this.ws.send({ type: 'set_property', instance_id: iid, key, value: val });
      };
      inp.addEventListener('change', handler);
      if (inp.type === 'range' || inp.type === 'number') {
        let t;
        inp.addEventListener('input', () => {
          clearTimeout(t); t = setTimeout(handler, 200);
          // live-update the displayed value
          const sib = inp.parentElement.querySelector('.prop-val');
          if (sib) sib.textContent = inp.value;
        });
      }
    });
  }
}

function esc(s) {
  return String(s == null ? '' : s).replace(/[&<>"']/g,
    c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
}
