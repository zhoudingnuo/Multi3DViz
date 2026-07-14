// icons.js — minimal inline SVG icon set (stroke style, currentColor).
// Keeps buttons modern (no emoji/legacy glyphs) and themeable via CSS color.
// Usage: import { icon } from './icons.js'; el.innerHTML = icon('play');

const PATHS = {
  // robot actions
  play:  '<polygon points="6 4 20 12 6 20 6 4" fill="currentColor" stroke="none"/>',
  stop:  '<rect x="6" y="6" width="12" height="12" rx="1.5" fill="currentColor" stroke="none"/>',
  trash: '<path d="M4 7h16M9 7V5a1 1 0 011-1h4a1 1 0 011 1v2M6 7l1 13a1 1 0 001 1h8a1 1 0 001-1l1-13"/>',
  // registration
  refresh: '<path d="M4 12a8 8 0 0114-5l2-2M20 12a8 8 0 01-14 5l-2 2M18 3v4h-4M6 21v-4h4"/>',
  camera: '<path d="M4 8a2 2 0 012-2h2l1.5-2h5L16 6h2a2 2 0 012 2v9a2 2 0 01-2 2H6a2 2 0 01-2-2V8z"/><circle cx="12" cy="12" r="3.5"/>',
  // status
  dot:   '<circle cx="12" cy="12" r="5" fill="currentColor" stroke="none"/>',
  // emergency stop (octagon + exclamation mark)
  estop: '<path d="M8 3h8l5 5v8l-5 5H8l-5-5V8z" fill="currentColor" fill-opacity="0.15" stroke="currentColor" stroke-width="2"/><path d="M12 8v5" stroke="currentColor" stroke-width="2.2"/><circle cx="12" cy="16.5" r="1.1" fill="currentColor" stroke="none"/>',
};

export function icon(name, size = 16) {
  const inner = PATHS[name];
  if (!inner) return '';
  return `<svg class="ico" width="${size}" height="${size}" viewBox="0 0 24 24" fill="none"
            stroke="currentColor" stroke-width="1.8" stroke-linecap="round"
            stroke-linejoin="round" aria-hidden="true">${inner}</svg>`;
}
