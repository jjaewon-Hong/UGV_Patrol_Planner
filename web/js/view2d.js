// 2D map on a canvas. Same look as the OpenCV view of the ROS 2 demo (nodes/visualizer.py):
// elevation as grey shading, obstacles by class, blue path, magenta UGV, cyan base, yellow flags.

export const COLORS = {
  building: [220, 40, 40],
  water: [90, 190, 230],
  tree: [40, 150, 50],
  unknown: [220, 40, 40],   // obstacles from the config with no class
  learned: [255, 150, 30],  // found by the sensor or added by a click
  path: 'rgb(30,120,220)',
  ugv: 'rgb(230,60,230)',
  base: 'rgb(0,200,200)',
  flag: 'rgb(255,210,0)',
  gate: 'rgb(185,185,185)',
  plan: 'rgb(255,140,0)',
};
const CLASS_RGB = [COLORS.unknown, COLORS.building, COLORS.water, COLORS.tree];
const GATE_NAMES = { 'Main gate (300 m N)': '정문 (북쪽 300 m)', 'East gate': '동문' };

/** Yellow -> pink -> purple for the return-energy overlay, t in 0..1 (colours no obstacle class uses). */
function heat(t) {
  const stops = [[252, 232, 100], [236, 112, 120], [110, 30, 160]];
  const x = Math.min(1, Math.max(0, t)) * 2, i = Math.min(1, Math.floor(x)), f = x - i;
  return stops[i].map((v, k) => Math.round(v + (stops[i + 1][k] - v) * f));
}

export class View2D {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.base = document.createElement('canvas');    // one pixel per cell: terrain + obstacles
    this.overlay = document.createElement('canvas'); // one pixel per cell: return energy
    this.map = null;
    this.energyOn = false;
  }

  setMap(map) {
    this.map = map;
    const n = map.n;
    this.cell = Math.max(2, Math.floor(800 / n));
    this.canvas.width = this.canvas.height = n * this.cell;
    this.base.width = this.base.height = this.overlay.width = this.overlay.height = n;
    let lo = Infinity, hi = -Infinity;
    for (const e of map.elevation) { lo = Math.min(lo, e); hi = Math.max(hi, e); }
    this.grey = Uint8Array.from(map.elevation, (e) => Math.round((e - lo) / Math.max(hi - lo, 1e-9) * 180 + 50));
    this.setObstacles([]);
    this.setEnergy(null);
  }

  /** learned: [[r, c], ...] obstacles found or added after the map was loaded. */
  setObstacles(learned) {
    const { n, obstacles, classes } = this.map;
    const ctx = this.base.getContext('2d');
    const img = ctx.createImageData(n, n);
    for (let i = 0; i < n * n; i++) {
      const rgb = obstacles[i] ? CLASS_RGB[classes[i]] ?? COLORS.unknown : [this.grey[i], this.grey[i], this.grey[i]];
      img.data.set([rgb[0], rgb[1], rgb[2], 255], i * 4);
    }
    this.known = new Uint8Array(obstacles);
    for (const [r, c] of learned) {
      this.known[r * n + c] = 1;
      img.data.set([...COLORS.learned, 255], (r * n + c) * 4);
    }
    ctx.putImageData(img, 0, 0);
  }

  /** energy: Float32Array of Wh to the base per cell (-1 unreachable), or null to hide. */
  setEnergy(energy) {
    this.energyOn = !!energy;
    if (!energy) return;
    const n = this.map.n;
    const ctx = this.overlay.getContext('2d');
    const img = ctx.createImageData(n, n);
    let max = 0;
    for (const e of energy) max = Math.max(max, e);
    this.energyMax = max;
    for (let i = 0; i < n * n; i++) {
      if (this.known[i]) continue;                       // obstacles keep their colour
      if (energy[i] < 0) img.data.set([0, 0, 0, 215], i * 4);     // base cannot be reached from here
      else img.data.set([...heat(energy[i] / Math.max(max, 1e-9)), 170], i * 4);
    }
    ctx.putImageData(img, 0, 0);
  }

  /** Cell under a pointer event, or null outside the map. */
  cellAt(ev) {
    const box = this.canvas.getBoundingClientRect();
    const n = this.map.n;
    const c = Math.floor((ev.clientX - box.left) / box.width * n);
    const r = Math.floor((ev.clientY - box.top) / box.height * n);
    return r >= 0 && r < n && c >= 0 && c < n ? [r, c] : null;
  }

  px([r, c]) { return [(c + 0.5) * this.cell, (r + 0.5) * this.cell]; }

  /**
   * f: { state, ugv: [r, c] (may be fractional), trail, hidden, plan: {a, b, path}, hover, sensorRadius }
   */
  draw(f) {
    const { ctx, cell: S } = this;
    const W = this.canvas.width;
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(this.base, 0, 0, W, W);
    if (this.energyOn) ctx.drawImage(this.overlay, 0, 0, W, W);
    ctx.lineJoin = ctx.lineCap = 'round';

    // obstacles the controller has not found yet
    ctx.strokeStyle = 'rgba(255,255,255,0.75)';
    ctx.lineWidth = 1;
    ctx.setLineDash([2, 2]);
    for (const [r, c] of f.hidden ?? []) {
      if (!this.known[r * this.map.n + c]) ctx.strokeRect(c * S + 1.5, r * S + 1.5, S - 3, S - 3);
    }
    ctx.setLineDash([]);

    const line = (cells, style, width, dash = []) => {
      if (cells.length < 2) return;
      ctx.beginPath();
      cells.forEach((cellRC, i) => { const [x, y] = this.px(cellRC); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.strokeStyle = style; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.stroke(); ctx.setLineDash([]);
    };

    line(f.trail ?? [], 'rgba(230,60,230,0.45)', 2);

    // planned path, from where the UGV is now
    const path = f.state?.path ?? [];
    if (path.length > 1) {
      const [pr, pc] = f.state.pos;
      const at = Math.max(0, path.findIndex(([r, c]) => r === pr && c === pc));
      const rest = [f.ugv ?? f.state.pos, ...path.slice(at + 1)];
      line(rest, 'rgba(0,0,0,0.45)', 5);
      line(rest, COLORS.path, 3);
    }

    // path explorer
    if (f.plan?.path) {
      line(f.plan.path, 'rgba(0,0,0,0.5)', 5);
      line(f.plan.path, COLORS.plan, 3, [6, 5]);
    }
    for (const [key, label] of [['a', 'A'], ['b', 'B']]) {
      if (!f.plan?.[key]) continue;
      const [x, y] = this.px(f.plan[key]);
      ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2);
      ctx.fillStyle = COLORS.plan; ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
      this.text(label, x, y + 4.5, '#111', 'center', false);
    }

    for (const lm of this.map.landmarks) this.gate(lm);

    // base
    const [bx, by] = this.px(this.map.base);
    ctx.strokeStyle = 'rgba(0,0,0,0.6)'; ctx.lineWidth = 5; ctx.strokeRect(bx - 8, by - 8, 16, 16);
    ctx.strokeStyle = COLORS.base; ctx.lineWidth = 3; ctx.strokeRect(bx - 8, by - 8, 16, 16);

    // patrol points
    for (const p of f.state?.status.points ?? []) {
      const [x, y] = this.px(p.cell);
      const active = p.pid === f.state.status.target;
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 2.5;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x, y - 21); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(x + 1, y - 21); ctx.lineTo(x + 16, y - 16); ctx.lineTo(x + 1, y - 10); ctx.closePath();
      ctx.fillStyle = COLORS.flag; ctx.fill();
      ctx.strokeStyle = active ? 'rgb(220,40,40)' : '#000'; ctx.lineWidth = active ? 2.5 : 1; ctx.stroke();
      const visits = f.state.visits[p.pid] ?? 0;
      this.text(`${p.pid}${visits ? ` ·${visits}` : ''}`, x + 19, y - 8, COLORS.flag);
    }

    // UGV with its sensor range
    if (f.ugv) {
      const [x, y] = this.px(f.ugv);
      if (f.sensorRadius) {
        ctx.beginPath(); ctx.arc(x, y, f.sensorRadius * S, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(230,60,230,0.10)'; ctx.fill();
        ctx.strokeStyle = 'rgba(255,255,255,0.45)'; ctx.lineWidth = 1; ctx.stroke();
      }
      ctx.beginPath(); ctx.arc(x, y, 7, 0, Math.PI * 2);
      ctx.fillStyle = COLORS.ugv; ctx.fill();
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 2; ctx.stroke();
    }

    if (f.hover) {
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
      ctx.strokeRect(f.hover[1] * S + 0.5, f.hover[0] * S + 0.5, S - 1, S - 1);
    }
  }

  text(label, x, y, color, align = 'left', box = true) {
    const { ctx } = this;
    ctx.font = '600 15px system-ui, "Segoe UI", "Malgun Gothic", sans-serif';
    ctx.textAlign = align;
    if (box) {
      const w = ctx.measureText(label).width;
      const left = align === 'right' ? x - w : x;
      ctx.fillStyle = 'rgba(20,20,20,0.72)';
      ctx.fillRect(left - 3, y - 14, w + 6, 19);
    }
    ctx.fillStyle = color;
    ctx.fillText(label, x, y);
    ctx.textAlign = 'left';
  }

  /** Campus gate as a grey arch, kept on screen, with its name on the side that has room. */
  gate({ name, cell }) {
    const { ctx } = this;
    const W = this.canvas.width;
    let [x, y] = this.px(cell);
    x = Math.min(Math.max(x, 12), W - 12);
    y = Math.min(Math.max(y, 8), W - 16);
    ctx.beginPath();
    for (const [i, [dx, dy]] of [[-9, 12], [-9, -4], [9, -4], [9, 12], [4, 12], [4, 2], [-4, 2], [-4, 12]].entries()) {
      i ? ctx.lineTo(x + dx, y + dy) : ctx.moveTo(x + dx, y + dy);
    }
    ctx.closePath();
    ctx.fillStyle = COLORS.gate; ctx.fill();
    ctx.strokeStyle = 'rgb(40,40,40)'; ctx.lineWidth = 1; ctx.stroke();
    const label = GATE_NAMES[name] ?? name;
    ctx.font = '600 15px system-ui, "Segoe UI", "Malgun Gothic", sans-serif';
    const right = x + 16 + ctx.measureText(label).width < W;
    this.text(label, right ? x + 16 : x - 16, y + 10, '#fff', right ? 'left' : 'right');
  }
}
