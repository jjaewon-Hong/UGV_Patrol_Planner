// 3D terrain view with three.js (stands in for RViz2): height field, obstacle boxes, path, UGV, flags, base.
// View only: drag to rotate, wheel or pinch to zoom. Heights above the lowest cell are multiplied by zScale.
import * as THREE from '../vendor/three.module.min.js';
import { COLORS } from './view2d.js';

// Colours are given as sRGB (the same numbers as the 2D view); three.js works in linear light.
const rgb = (c) => (Array.isArray(c)
  ? new THREE.Color().setRGB(c[0] / 255, c[1] / 255, c[2] / 255, THREE.SRGBColorSpace)
  : new THREE.Color(c));
// box height in cells: unknown, building, water, tree, learned
const HEIGHT = [0.8, 1.2, 0.08, 0.7, 0.9];
const BOX_RGB = [COLORS.unknown, COLORS.building, COLORS.water, COLORS.tree, COLORS.learned];
const MAX_PATH = 2500;

function label(text, color) {
  const canvas = document.createElement('canvas');
  canvas.width = 128; canvas.height = 64;
  const ctx = canvas.getContext('2d');
  ctx.font = '700 40px system-ui, sans-serif';
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.lineWidth = 8; ctx.strokeStyle = 'rgba(0,0,0,0.85)'; ctx.strokeText(text, 64, 32);
  ctx.fillStyle = color; ctx.fillText(text, 64, 32);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }));
  sprite.renderOrder = 10;
  return sprite;
}

export class View3D {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x14171c);
    this.camera = new THREE.PerspectiveCamera(40, 1, 1, 100000);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x50555c, 2.2));
    const sun = new THREE.DirectionalLight(0xffffff, 2.4);
    sun.position.set(-1, 1.6, -0.6);  // from the north-west, like a shaded relief map
    this.scene.add(sun);
    this.world = new THREE.Group();   // everything that depends on the map
    this.scene.add(this.world);
    this.orbit = { az: -0.6, polar: 0.95, dist: 1 };
    this.pointers = new Map();
    this.flagKey = '';
    this.setupOrbit();
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  setupOrbit() {
    const el = this.renderer.domElement;
    const o = this.orbit;
    let pinch = 0;
    el.addEventListener('pointerdown', (ev) => { el.setPointerCapture(ev.pointerId); this.pointers.set(ev.pointerId, [ev.clientX, ev.clientY]); });
    el.addEventListener('pointermove', (ev) => {
      const prev = this.pointers.get(ev.pointerId);
      if (!prev) return;
      this.pointers.set(ev.pointerId, [ev.clientX, ev.clientY]);
      if (this.pointers.size === 1) {
        o.az -= (ev.clientX - prev[0]) * 0.006;
        o.polar = Math.min(1.5, Math.max(0.12, o.polar - (ev.clientY - prev[1]) * 0.005));
      } else if (this.pointers.size === 2) {
        const [a, b] = [...this.pointers.values()];
        const d = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (pinch) o.dist = Math.min(3, Math.max(0.25, o.dist * pinch / d));
        pinch = d;
      }
    });
    const up = (ev) => { this.pointers.delete(ev.pointerId); pinch = 0; };
    el.addEventListener('pointerup', up);
    el.addEventListener('pointercancel', up);
    el.addEventListener('wheel', (ev) => {
      ev.preventDefault();
      o.dist = Math.min(3, Math.max(0.25, o.dist * Math.exp(ev.deltaY * 0.001)));
    }, { passive: false });
  }

  // ---------------------------------------------------------------- map
  setMap(map, learned, zScale) {
    this.map = map;
    this.zScale = zScale;
    this.learned = learned;
    this.z0 = Infinity;
    for (const e of map.elevation) this.z0 = Math.min(this.z0, e);
    this.size = map.n * map.cell_size_m;
    this.rebuild();
  }

  setObstacles(learned) { this.learned = learned; this.buildObstacles(); }
  setZScale(z) { this.zScale = z; this.rebuild(); }

  /** Scene position of a cell (may be fractional): x east, y up, z south; the map is centred on the origin. */
  pos([r, c], lift = 0) {
    const { n, cell_size_m: cs, elevation } = this.map;
    const ri = Math.min(n - 1, Math.max(0, Math.round(r))), ci = Math.min(n - 1, Math.max(0, Math.round(c)));
    return new THREE.Vector3((c + 0.5) * cs - this.size / 2, (elevation[ri * n + ci] - this.z0) * this.zScale + lift,
      (r + 0.5) * cs - this.size / 2);
  }

  clear(group) {
    for (const child of [...group.children]) {
      group.remove(child);
      child.traverse?.((o) => { o.geometry?.dispose(); o.material?.map?.dispose(); o.material?.dispose(); });
    }
  }

  rebuild() {
    const { n, cell_size_m: cs, elevation } = this.map;
    this.clear(this.world);
    this.flagKey = '';

    // terrain: one vertex per cell centre, grey by elevation like the 2D view
    let hi = -Infinity;
    for (const e of elevation) hi = Math.max(hi, e);
    const positions = new Float32Array(n * n * 3), colors = new Float32Array(n * n * 3);
    for (let r = 0; r < n; r++) {
      for (let c = 0; c < n; c++) {
        const i = r * n + c, p = this.pos([r, c]);
        positions.set([p.x, p.y, p.z], i * 3);
        const g = (elevation[i] - this.z0) / Math.max(hi - this.z0, 1e-9) * 180 + 50;
        colors.set(rgb([g, g, g]).toArray(), i * 3);
      }
    }
    const index = [];
    for (let r = 0; r + 1 < n; r++) {
      for (let c = 0; c + 1 < n; c++) {
        const a = r * n + c, b = a + 1, d = a + n, e = d + 1;
        index.push(a, d, b, b, d, e);
      }
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.setIndex(index);
    geo.computeVertexNormals();
    this.world.add(new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ vertexColors: true })));

    this.obstacleGroup = new THREE.Group();
    this.world.add(this.obstacleGroup);
    this.buildObstacles();

    // path: one small tile per cell
    this.pathMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(cs * 0.8, cs * 0.25, cs * 0.8),
      new THREE.MeshBasicMaterial({ color: rgb(COLORS.path) }), MAX_PATH);
    this.pathMesh.count = 0;
    this.pathMesh.frustumCulled = false;
    this.planMesh = new THREE.InstancedMesh(new THREE.BoxGeometry(cs * 0.6, cs * 0.25, cs * 0.6),
      new THREE.MeshBasicMaterial({ color: rgb(COLORS.plan) }), MAX_PATH);
    this.planMesh.count = 0;
    this.planMesh.frustumCulled = false;
    this.world.add(this.pathMesh, this.planMesh);

    // UGV: a ball with a thin beam so it can be found from far away
    this.ugv = new THREE.Group();
    this.ugv.add(new THREE.Mesh(new THREE.SphereGeometry(cs * 0.75, 20, 14), new THREE.MeshLambertMaterial({ color: rgb(COLORS.ugv) })));
    const beam = new THREE.Mesh(new THREE.CylinderGeometry(cs * 0.08, cs * 0.08, cs * 6, 6),
      new THREE.MeshBasicMaterial({ color: rgb(COLORS.ugv), transparent: true, opacity: 0.55 }));
    beam.position.y = cs * 3;
    this.ugv.add(beam);
    this.world.add(this.ugv);

    // base: cyan frame
    const base = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(cs * 1.8, cs * 1.2, cs * 1.8)),
      new THREE.LineBasicMaterial({ color: rgb(COLORS.base) }));
    base.position.copy(this.pos(this.map.base, cs * 0.6));
    this.world.add(base);

    // gates: two pillars and a beam
    for (const lm of this.map.landmarks) {
      const gate = new THREE.Group();
      const mat = new THREE.MeshLambertMaterial({ color: rgb(COLORS.gate) });
      for (const x of [-0.9, 0.9]) {
        const pillar = new THREE.Mesh(new THREE.BoxGeometry(cs * 0.4, cs * 2, cs * 0.4), mat);
        pillar.position.set(x * cs, cs, 0);
        gate.add(pillar);
      }
      const top = new THREE.Mesh(new THREE.BoxGeometry(cs * 2.2, cs * 0.4, cs * 0.4), mat);
      top.position.y = cs * 2.2;
      gate.add(top);
      gate.position.copy(this.pos(lm.cell));
      this.world.add(gate);
    }

    this.flags = new THREE.Group();
    this.world.add(this.flags);
    this.target = new THREE.Vector3(0, (hi - this.z0) * this.zScale * 0.3, 0);
  }

  buildObstacles() {
    if (!this.obstacleGroup) return;
    this.clear(this.obstacleGroup);
    const { n, cell_size_m: cs, obstacles, classes } = this.map;
    const cells = [];
    for (let i = 0; i < n * n; i++) if (obstacles[i]) cells.push([Math.floor(i / n), i % n, classes[i] ?? 0]);
    for (const [r, c] of this.learned) if (!obstacles[r * n + c]) cells.push([r, c, 4]);
    const mesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshLambertMaterial(), Math.max(1, cells.length));
    mesh.count = cells.length;
    const m = new THREE.Matrix4(), q = new THREE.Quaternion();
    cells.forEach(([r, c, cls], i) => {
      const h = HEIGHT[cls] * cs;
      m.compose(this.pos([r, c], h / 2), q, new THREE.Vector3(cs * 0.96, h, cs * 0.96));
      mesh.setMatrixAt(i, m);
      mesh.setColorAt(i, rgb(BOX_RGB[cls]));
    });
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    mesh.frustumCulled = false;
    this.obstacleGroup.add(mesh);
  }

  setTiles(mesh, cells, lift) {
    const m = new THREE.Matrix4();
    const count = Math.min(cells.length, MAX_PATH);
    for (let i = 0; i < count; i++) {
      const p = this.pos(cells[i], lift);
      m.makeTranslation(p.x, p.y, p.z);
      mesh.setMatrixAt(i, m);
    }
    mesh.count = count;
    mesh.instanceMatrix.needsUpdate = true;
  }

  buildFlags(state) {
    this.clear(this.flags);
    const cs = this.map.cell_size_m;
    for (const p of state.status.points) {
      const active = p.pid === state.status.target;
      const flag = new THREE.Group();
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(cs * 0.1, cs * 0.1, cs * 4, 6), new THREE.MeshBasicMaterial({ color: 0xffffff }));
      pole.position.y = cs * 2;
      const shape = new THREE.BufferGeometry();
      shape.setAttribute('position', new THREE.BufferAttribute(new Float32Array([0, cs * 4, 0, 0, cs * 2.6, 0, cs * 2.2, cs * 3.3, 0]), 3));
      const pennant = new THREE.Mesh(shape, new THREE.MeshBasicMaterial({ color: active ? 0xff5a36 : rgb(COLORS.flag), side: THREE.DoubleSide }));
      const visits = state.visits[p.pid] ?? 0;
      const text = label(`${p.pid}${visits ? ` ·${visits}` : ''}`, COLORS.flag);
      text.scale.set(cs * 7, cs * 3.5, 1);
      text.position.y = cs * 6;
      flag.add(pole, pennant, text);
      flag.position.copy(this.pos(p.cell));
      this.flags.add(flag);
    }
  }

  // ---------------------------------------------------------------- frame
  draw(f) {
    if (!this.map || !f.state) return;
    const cs = this.map.cell_size_m;
    const s = f.state;

    const key = JSON.stringify([s.status.points, s.status.target, s.visits]);
    if (key !== this.flagKey) { this.flagKey = key; this.buildFlags(s); }

    const at = Math.max(0, s.path.findIndex(([r, c]) => r === s.pos[0] && c === s.pos[1]));
    this.setTiles(this.pathMesh, s.path.length > 1 ? s.path.slice(at) : [], cs * 0.2);
    this.setTiles(this.planMesh, f.plan?.path ?? [], cs * 0.35);

    // height between two cells is blended so the ball does not jump
    const [r, c] = f.ugv;
    const a = this.pos([Math.floor(r), Math.floor(c)]), b = this.pos([Math.ceil(r), Math.ceil(c)]);
    const t = (r - Math.floor(r)) + (c - Math.floor(c));
    this.ugv.position.set((c + 0.5) * cs - this.size / 2, a.y + (b.y - a.y) * t + cs * 0.75, (r + 0.5) * cs - this.size / 2);

    const o = this.orbit, d = this.size * 1.45 * o.dist;
    this.camera.position.set(
      this.target.x + d * Math.sin(o.polar) * Math.sin(o.az),
      this.target.y + d * Math.cos(o.polar),
      this.target.z + d * Math.sin(o.polar) * Math.cos(o.az));
    this.camera.lookAt(this.target);
    this.renderer.render(this.scene, this.camera);
  }
}
