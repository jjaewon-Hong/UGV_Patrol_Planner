// 3D terrain view with three.js, drawn like the RViz2 view of the ROS 2 run: one box per cell on a grey
// background with a ground grid, obstacle cubes on top, path line, UGV, patrol points, base.
// View only: drag to rotate, wheel or pinch to zoom. Heights above the lowest cell are multiplied by zScale.
import * as THREE from '../vendor/three.module.min.js';
import { COLORS } from './view2d.js';

// Colours are given as sRGB; three.js works in linear light.
const rgb = (c) => (Array.isArray(c)
  ? new THREE.Color().setRGB(c[0] / 255, c[1] / 255, c[2] / 255, THREE.SRGBColorSpace)
  : new THREE.Color(c));
const BACKGROUND = [48, 48, 48];          // RViz default background
const GRID = [130, 130, 134];
const GROUND = [52, 108];                 // terrain grey: lowest cell .. highest cell
// obstacle cubes: unknown, building, water, tree, learned (muted, not the bright 2D map colours)
const BOX_RGB = [[150, 150, 150], [196, 34, 34], [74, 158, 204], [34, 132, 48], [226, 136, 30]];
const HEIGHT = [0.8, 1.0, 0.25, 0.8, 0.9];  // in cells
const MAX_PATH = 2500;

/** Text that always faces the camera. Its world height is set by the caller with fit(). */
function label(text, color = '#f2f2f2') {
  const font = '600 34px system-ui, "Segoe UI", sans-serif';
  const canvas = document.createElement('canvas');
  let ctx = canvas.getContext('2d');
  ctx.font = font;
  canvas.width = Math.ceil(ctx.measureText(text).width) + 20;
  canvas.height = 48;
  ctx = canvas.getContext('2d');
  ctx.font = font;
  ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.lineWidth = 5; ctx.strokeStyle = 'rgba(20,20,20,0.9)'; ctx.strokeText(text, canvas.width / 2, 25);
  ctx.fillStyle = color; ctx.fillText(text, canvas.width / 2, 25);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(canvas), depthTest: false }));
  sprite.renderOrder = 10;
  sprite.fit = (height) => { sprite.scale.set(height * canvas.width / canvas.height, height, 1); return sprite; };
  return sprite;
}

export class View3D {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = rgb(BACKGROUND);
    this.camera = new THREE.PerspectiveCamera(40, 1, 1, 100000);
    // flat, even light: tops lit, sides a step darker, so every cell reads as a box
    this.scene.add(new THREE.AmbientLight(0xffffff, 1.5));
    const sun = new THREE.DirectionalLight(0xffffff, 1.9);
    sun.position.set(-0.5, 1.6, 0.9);
    this.scene.add(sun);
    this.world = new THREE.Group();   // everything that depends on the map
    this.scene.add(this.world);
    this.orbit = { az: -0.35, polar: 0.9, dist: 1 };
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

  /** Scene position of a cell's top (cell may be fractional): x east, y up, z south; the map is centred on the origin. */
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
    const floor = -cs;  // bottom of every terrain column, and the level of the grid

    // ground grid (100 m squares for a 10 m map cell), wider than the map
    const grid = new THREE.GridHelper(this.size * 2, 2 * n / 10, rgb(GRID), rgb(GRID));
    grid.position.y = floor;
    this.world.add(grid);

    // terrain: one column per cell with a small gap, grey by elevation
    let hi = -Infinity;
    for (const e of elevation) hi = Math.max(hi, e);
    const ground = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshLambertMaterial(), n * n);
    const m = new THREE.Matrix4(), q = new THREE.Quaternion();
    for (let r = 0; r < n; r++) {
      for (let c = 0; c < n; c++) {
        const i = r * n + c, top = this.pos([r, c]);
        const h = top.y - floor;
        m.compose(new THREE.Vector3(top.x, floor + h / 2, top.z), q, new THREE.Vector3(cs * 0.92, h, cs * 0.92));
        ground.setMatrixAt(i, m);
        const g = GROUND[0] + (elevation[i] - this.z0) / Math.max(hi - this.z0, 1e-9) * (GROUND[1] - GROUND[0]);
        ground.setColorAt(i, rgb([g, g, g]));
      }
    }
    ground.frustumCulled = false;
    this.world.add(ground);

    this.obstacleGroup = new THREE.Group();
    this.world.add(this.obstacleGroup);
    this.buildObstacles();

    // path: one bar per step between two cells, so it reads as a line that follows the ground
    const bar = () => {
      const mesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), new THREE.MeshBasicMaterial(), MAX_PATH);
      mesh.count = 0;
      mesh.frustumCulled = false;
      return mesh;
    };
    this.pathMesh = bar();
    this.pathMesh.material.color = rgb([40, 120, 235]);
    this.planMesh = bar();
    this.planMesh.material.color = rgb(COLORS.plan);
    this.world.add(this.pathMesh, this.planMesh);

    // UGV: magenta ball
    this.ugv = new THREE.Mesh(new THREE.SphereGeometry(cs * 0.8, 24, 16), new THREE.MeshLambertMaterial({ color: rgb([225, 50, 225]) }));
    this.world.add(this.ugv);

    // base: cyan frame
    const base = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(cs * 1.8, cs * 1.2, cs * 1.8)),
      new THREE.LineBasicMaterial({ color: rgb(COLORS.base) }));
    base.position.copy(this.pos(this.map.base, cs * 0.6));
    this.world.add(base);

    // gates: two pillars and a beam, with the name above
    for (const lm of this.map.landmarks) {
      const gate = new THREE.Group();
      const mat = new THREE.MeshLambertMaterial({ color: rgb([200, 200, 200]) });
      for (const x of [-0.9, 0.9]) {
        const pillar = new THREE.Mesh(new THREE.BoxGeometry(cs * 0.4, cs * 2, cs * 0.4), mat);
        pillar.position.set(x * cs, cs, 0);
        gate.add(pillar);
      }
      const top = new THREE.Mesh(new THREE.BoxGeometry(cs * 2.2, cs * 0.4, cs * 0.4), mat);
      top.position.y = cs * 2.2;
      gate.add(top);
      const name = label(lm.name.replace(/\s*\(.*\)/, '').replace(' ', '-')).fit(cs * 2.4);
      name.position.y = cs * 4.4;
      gate.add(name);
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
      m.compose(this.pos([r, c], h / 2), q, new THREE.Vector3(cs * 0.92, h, cs * 0.92));
      mesh.setMatrixAt(i, m);
      mesh.setColorAt(i, rgb(BOX_RGB[cls]));
    });
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
    mesh.frustumCulled = false;
    this.obstacleGroup.add(mesh);
  }

  /** Draw cells as a line of bars, each from one cell centre to the next. */
  setLine(mesh, cells, lift, width) {
    const m = new THREE.Matrix4(), q = new THREE.Quaternion(), z = new THREE.Vector3(0, 0, 1);
    const count = Math.max(0, Math.min(cells.length - 1, MAX_PATH));
    for (let i = 0; i < count; i++) {
      const a = this.pos(cells[i], lift), b = this.pos(cells[i + 1], lift);
      const d = b.clone().sub(a), len = d.length();
      q.setFromUnitVectors(z, d.normalize());
      m.compose(a.add(b).multiplyScalar(0.5), q, new THREE.Vector3(width, width, len + width));
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
      const pole = new THREE.Mesh(new THREE.CylinderGeometry(cs * 0.08, cs * 0.08, cs * 2.4, 6), new THREE.MeshBasicMaterial({ color: 0xd8d8d8 }));
      pole.position.y = cs * 1.2;
      const ball = new THREE.Mesh(new THREE.SphereGeometry(cs * 0.7, 20, 14),
        new THREE.MeshLambertMaterial({ color: rgb(active ? [255, 140, 20] : [240, 200, 20]) }));
      ball.position.y = cs * 2.6;
      const visits = state.visits[p.pid] ?? 0;
      const text = label(visits ? `${p.pid} (${visits})` : p.pid).fit(cs * 2.4);
      text.position.y = cs * 5;
      flag.add(pole, ball, text);
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
    this.setLine(this.pathMesh, s.path.length > 1 ? s.path.slice(at) : [], cs * 0.3, cs * 0.3);
    this.setLine(this.planMesh, f.plan?.path ?? [], cs * 0.45, cs * 0.3);

    // height between two cells is blended so the ball does not jump
    const [r, c] = f.ugv;
    const a = this.pos([Math.floor(r), Math.floor(c)]), b = this.pos([Math.ceil(r), Math.ceil(c)]);
    const t = (r - Math.floor(r)) + (c - Math.floor(c));
    this.ugv.position.set((c + 0.5) * cs - this.size / 2, a.y + (b.y - a.y) * t + cs * 0.8, (r + 0.5) * cs - this.size / 2);

    const o = this.orbit, d = this.size * 1.6 * o.dist;
    this.camera.position.set(
      this.target.x + d * Math.sin(o.polar) * Math.sin(o.az),
      this.target.y + d * Math.cos(o.polar),
      this.target.z + d * Math.sin(o.polar) * Math.cos(o.az));
    this.camera.lookAt(this.target);
    this.renderer.render(this.scene, this.camera);
  }
}
