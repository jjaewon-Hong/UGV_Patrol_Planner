// Loads the WebAssembly module (cpp/web/web_api.cpp) and wraps its C functions.
// No drawing here, so the same file runs in the browser and in Node (web/test/node_test.mjs).
import createUgvModule from '../ugv_web.js';

export const TICK_S = 0.2; // simulated seconds per tick (5 Hz drive rate, as in the ROS 2 demo)

/** base64 text -> typed array (Float64Array for elevation, Uint8Array for grids). */
export function decodeB64(text, Type) {
  const bin = atob(text);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return Type === Uint8Array ? bytes : new Type(bytes.buffer);
}

/** Map JSON (tools/export_web.py) with its arrays decoded. */
export function prepareMap(json) {
  return {
    ...json,
    elevation: decodeB64(json.elevation_b64, Float64Array),
    obstacles: decodeB64(json.obstacles_b64, Uint8Array),
    classes: decodeB64(json.classes_b64, Uint8Array),
  };
}

export async function loadUgv() {
  const M = await createUgvModule();
  const fn = (name, ret, args) => M.cwrap(name, ret, args);
  const api = {
    create: fn('ugv_create', 'number', ['number', 'number', 'number', 'number']),
    lastError: fn('ugv_last_error', 'string', []),
    setHidden: fn('ugv_set_hidden', null, ['number', 'number']),
    command: fn('ugv_command', null, ['string']),
    reportObstacle: fn('ugv_report_obstacle', null, ['number', 'number']),
    setBattery: fn('ugv_set_battery', null, ['number']),
    tick: fn('ugv_tick', 'number', ['number']),
    stateJson: fn('ugv_state_json', 'string', []),
    planJson: fn('ugv_plan_json', 'string', ['number', 'number', 'number', 'number']),
    returnEnergy: fn('ugv_return_energy', 'number', []),
    traversable: fn('ugv_traversable', 'number', ['number', 'number', 'number', 'number']),
  };

  /**
   * Build the world for a prepared map. The module holds one world; creating a new one replaces it.
   * opts: { maxSlopeDeg, batteryWh, extraObstacles: [[r, c], ...] } override the map's own values.
   */
  function createWorld(map, opts = {}) {
    const n = map.n, count = n * n;
    const obstacles = new Uint8Array(map.obstacles);
    for (const [r, c] of opts.extraObstacles ?? []) obstacles[r * n + c] = 1;
    const cfg = [
      map.cell_size_m, opts.maxSlopeDeg ?? map.max_slope_deg, map.slope_weight,
      map.wh_per_m, map.wh_per_climb_m, map.return_margin,
      map.capacity_wh, opts.batteryWh ?? map.initial_battery_wh,
      map.sensor_radius, map.base[0], map.base[1], map.start[0], map.start[1], TICK_S,
    ];
    const pElev = M._malloc(count * 8), pObs = M._malloc(count), pCfg = M._malloc(cfg.length * 8);
    try {
      M.HEAPF64.set(map.elevation, pElev / 8);
      M.HEAPU8.set(obstacles, pObs);
      M.HEAPF64.set(cfg, pCfg / 8);
      if (api.create(n, pElev, pObs, pCfg) !== 0) throw new Error(api.lastError());
    } finally {
      M._free(pElev); M._free(pObs); M._free(pCfg);
    }
    if (map.hidden.length) {
      const p = M._malloc(map.hidden.length * 8);
      M.HEAP32.set(map.hidden.flat(), p / 4);
      api.setHidden(p, map.hidden.length);
      M._free(p);
    }
    return {
      n,
      command: (text) => api.command(text),
      reportObstacle: (r, c) => api.reportObstacle(r, c),
      setBattery: (wh) => api.setBattery(wh),
      /** Advance `count` ticks; returns how many moved the UGV. */
      tick: (ticks = 1) => api.tick(ticks),
      /** Position, battery, mission status, path, counters; alarms and new obstacles since the last call. */
      state: () => JSON.parse(api.stateJson()),
      /** A* between two cells on the current map: { found, path, cost, cells, climb_m, max_slope, plan_ms }. */
      plan: (from, to) => JSON.parse(api.planJson(from[0], from[1], to[0], to[1])),
      /** Wh needed to get back to the base from every cell (-1 = unreachable). */
      returnEnergy: () => {
        const p = api.returnEnergy() / 4;
        return M.HEAPF32.slice(p, p + count);
      },
      traversable: (a, b) => api.traversable(a[0], a[1], b[0], b[1]) === 1,
    };
  }

  return { createWorld };
}
