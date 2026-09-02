// PathAndreas live viewer.
//
// Talks to the running service over its WebSocket protocol - the same JSON the
// Pawn side sends - so what you see drawn is what a consumer would receive, not
// a re-implementation of the routing. There is no build step and no server-side
// component: open this file from disk, point it at the service, click two spots.
//
// Coordinates are GTA world units throughout: x east, y north, z up. The canvas
// flips y so north is up on screen.

'use strict';

const MAP_MIN = -3000, MAP_MAX = 3000;

const el = (id) => document.getElementById(id);
const canvas = el('map');
const ctx = canvas.getContext('2d');

// --- view transform -------------------------------------------------------
// scale = screen pixels per world unit. Start showing the whole state.
const view = { x: 0, y: 0, scale: 0.1 };

function toScreen(wx, wy) {
  return [(wx - view.x) * view.scale + canvas.width / 2,
          canvas.height / 2 - (wy - view.y) * view.scale];
}
function toWorld(sx, sy) {
  return [(sx - canvas.width / 2) / view.scale + view.x,
          view.y - (sy - canvas.height / 2) / view.scale];
}

function resize() {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(r.width * dpr);
  canvas.height = Math.round(r.height * dpr);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  draw();
}
window.addEventListener('resize', resize);

// --- service connection ---------------------------------------------------
let ws = null, nextId = 1;
const pending = new Map();

function log(msg) {
  const d = el('log');
  d.textContent = msg + '\n' + d.textContent;
  if (d.textContent.length > 4000) d.textContent = d.textContent.slice(0, 4000);
}

function connect() {
  if (ws) { try { ws.close(); } catch (e) {} ws = null; }
  const url = el('url').value.trim();
  let sock;
  try { sock = new WebSocket(url); } catch (e) { log('bad url: ' + e.message); return; }
  ws = sock;
  setStatus('connecting…', false);
  sock.onopen = () => {
    setStatus('connected', true);
    send('status', {}).then((s) => {
      log('navmesh=' + s.navmesh + ' roads=' + s.roads +
          (s.road_nodes ? ' (' + s.road_nodes + ' nodes)' : '') +
          ' ped=' + s.ped_roads + (s.ped_nodes ? ' (' + s.ped_nodes + ')' : ''));
      refreshNodes();
    }).catch((e) => log('status: ' + e));
  };
  sock.onclose = () => { setStatus('disconnected', false); ws = null; };
  sock.onerror = () => log('socket error - is the service running with --bind on this host?');
  sock.onmessage = (ev) => {
    let m;
    try { m = JSON.parse(ev.data); } catch (e) { return; }
    const p = pending.get(m.id);
    if (!p) return;
    pending.delete(m.id);
    if (m.type === 'error') p.reject(m.error); else p.resolve(m);
  };
}

function setStatus(text, ok) {
  const s = el('status');
  s.textContent = text;
  s.className = ok ? 'on' : 'off';
}

// Every request carries an id; the service echoes it, which is how replies are
// matched without assuming they come back in order.
function send(type, body) {
  return new Promise((resolve, reject) => {
    if (!ws || ws.readyState !== 1) { reject('not connected'); return; }
    const id = 'v' + (nextId++);
    pending.set(id, { resolve, reject });
    ws.send(JSON.stringify(Object.assign({ type, id }, body)));
    setTimeout(() => {
      if (pending.has(id)) { pending.delete(id); reject('timeout'); }
    }, 120000);
  });
}

// --- node layers ----------------------------------------------------------
// The node graphs are the map: 30k road nodes are the road network and 37k ped
// nodes the sidewalks, so no game image is needed as a backdrop. Fetched per
// viewport because sending all of them on every pan would be absurd.
const layers = { vehicle: null, ped: null };
let nodeReqSeq = 0;

async function refreshNodes() {
  if (!ws || ws.readyState !== 1) return;
  const [x0, y1] = toWorld(0, 0);
  const [x1, y0] = toWorld(canvas.width, canvas.height);
  const pad = (x1 - x0) * 0.1;
  const seq = ++nodeReqSeq;
  const want = [];
  if (el('showveh').checked) want.push('vehicle');
  if (el('showped').checked) want.push('ped');
  for (const g of ['vehicle', 'ped']) if (!want.includes(g)) layers[g] = null;
  for (const g of want) {
    try {
      const r = await send('nodes_in_rect', {
        graph: g,
        min: [x0 - pad, y0 - pad, 0],
        max: [x1 + pad, y1 + pad, 0],
        limit: 6000,
      });
      if (seq !== nodeReqSeq) return;   // a newer view superseded this fetch
      layers[g] = r;
      if (r.truncated) log(g + ' nodes truncated - zoom in for the full graph');
    } catch (e) { layers[g] = null; }
  }
  draw();
}

// --- route state ----------------------------------------------------------
let from = null, to = null, route = null, lastMs = 0;

const CSSVAR = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

async function runQuery() {
  if (!from || !to) { log('set FROM and TO first'); return; }
  const type = el('qtype').value;
  const body = { from, to };
  // offroad_cost and the corridor mask only mean anything to the walking
  // queries; sending them elsewhere would just be noise.
  if (type === 'find_hybrid_path' || type === 'find_path') {
    body.offroad_cost = parseFloat(el('cost').value);
    body.include_corridor = true;
  }
  const t0 = performance.now();
  try {
    route = await send(type, body);
    lastMs = performance.now() - t0;
    renderStats();
    draw();
  } catch (e) {
    route = null;
    el('stats').innerHTML = '<div class="empty">' + e + '</div>';
    draw();
  }
}

function fmt(n, d) { return (Math.round(n * Math.pow(10, d || 0)) / Math.pow(10, d || 0)).toString(); }

function pathLength(wps) {
  let l = 0;
  for (let i = 1; i < wps.length; i++) {
    const dx = wps[i][0] - wps[i - 1][0], dy = wps[i][1] - wps[i - 1][1];
    l += Math.sqrt(dx * dx + dy * dy);
  }
  return l;
}

function renderStats() {
  const r = route;
  if (!r) return;
  const wps = r.waypoints || [];
  const rows = [];
  const add = (k, v, cls) => rows.push('<tr><td>' + k + '</td><td' +
      (cls ? ' class="' + cls + '"' : '') + '>' + v + '</td></tr>');
  add('success', r.success ? 'yes' : 'no', r.success ? 'good' : 'bad');
  if (r.graph) add('graph', r.graph);
  add('waypoints', wps.length);
  add('length', fmt(pathLength(wps), 1) + 'u');
  add('query', fmt(lastMs, 1) + ' ms');
  if (typeof r.sidewalk_ratio === 'number' && r.sidewalk_ratio >= 0) {
    const pct = Math.round(r.sidewalk_ratio * 100);
    add('on corridor', pct + '%', pct >= 80 ? 'good' : (pct >= 50 ? 'warn' : 'bad'));
  }
  if (typeof r.straight_segments === 'number') {
    add('unverified hops', r.straight_segments,
        r.straight_segments === 0 ? 'good' : 'warn');
    add('longest unverified', fmt(r.longest_unconfirmed || 0, 0) + 'u');
  }
  if (r.climb_at) add('climbs', r.climb_at.length, r.climb_at.length ? 'warn' : '');
  if (r.partial) add('partial', 'yes', 'bad');
  if (r.reached_goal === false) {
    add('reached goal', 'NO', 'bad');
    add('gap to goal', fmt(r.goal_gap[0], 1) + 'u across, ' +
        fmt(r.goal_gap[1], 1) + 'u up', 'bad');
  }
  // Vehicle answers carry their own extras.
  if (r.offroad_start) {
    add('offroad start', fmt(r.offroad_start.distance, 0) + 'u ' +
        (r.offroad_start.drivable ? 'ok' : r.offroad_start.reason),
        r.offroad_start.drivable ? '' : 'warn');
    add('offroad goal', fmt(r.offroad_goal.distance, 0) + 'u ' +
        (r.offroad_goal.drivable ? 'ok' : r.offroad_goal.reason),
        r.offroad_goal.drivable ? '' : 'warn');
  }
  el('stats').innerHTML = '<table>' + rows.join('') + '</table>';
}

// --- drawing --------------------------------------------------------------
function draw() {
  ctx.fillStyle = CSSVAR('--bg');
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  if (layers.vehicle) drawLayer(layers.vehicle, CSSVAR('--vnode'), 1.0);
  if (layers.ped) drawLayer(layers.ped, CSSVAR('--pnode'), 0.8);
  if (route && route.waypoints) drawRoute(route);
  drawEndpoint(from, '#7cf', 'FROM');
  drawEndpoint(to, '#fc7', 'TO');
}

function drawGrid() {
  const step = 500;
  ctx.strokeStyle = '#1b2029';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let v = MAP_MIN; v <= MAP_MAX; v += step) {
    const a = toScreen(v, MAP_MIN), b = toScreen(v, MAP_MAX);
    ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]);
    const c = toScreen(MAP_MIN, v), d = toScreen(MAP_MAX, v);
    ctx.moveTo(c[0], c[1]); ctx.lineTo(d[0], d[1]);
  }
  ctx.stroke();
}

function drawLayer(layer, colour, width) {
  const n = layer.nodes;
  ctx.strokeStyle = colour;
  ctx.lineWidth = width;
  ctx.beginPath();
  for (const e of layer.edges) {
    const a = n[e[0]], b = n[e[1]];
    const p = toScreen(a[1], a[2]), q = toScreen(b[1], b[2]);
    ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]);
  }
  ctx.stroke();
  // Dots only once zoomed in far enough that they are not just noise.
  if (view.scale > 0.5) {
    ctx.fillStyle = colour;
    for (const nd of n) {
      const p = toScreen(nd[1], nd[2]);
      ctx.fillRect(p[0] - 1.5, p[1] - 1.5, 3, 3);
    }
  }
}

// The route is drawn segment by segment rather than as one polyline, because
// each segment says something different: whether it is on the pedestrian
// corridor, whether the navmesh confirmed it, whether it is a climb. Colouring
// per segment is the whole point of the viewer - it shows WHY a route looks the
// way it does, not just where it goes.
function drawRoute(r) {
  const w = r.waypoints;
  if (!w || w.length < 2) return;
  const mask = (el('showcorr').checked && r.corridor_mask) || null;
  const climbs = new Set(r.climb_at || []);
  const onCol = CSSVAR('--on'), offCol = CSSVAR('--off'), unvCol = CSSVAR('--unv');

  // Unverified hops are not indexed in the reply - only counted - so they are
  // inferred from length: a straight jump much longer than the mesh's own
  // waypoint spacing is one. Approximate on purpose; the count in the panel is
  // the authoritative figure.
  const longHop = Math.max(30, (r.longest_unconfirmed || 0) * 0.95);
  const anyUnverified = (r.straight_segments || 0) > 0;

  ctx.lineWidth = 2.5;
  ctx.lineCap = 'round';
  for (let i = 1; i < w.length; i++) {
    const a = toScreen(w[i - 1][0], w[i - 1][1]);
    const b = toScreen(w[i][0], w[i][1]);
    const dx = w[i][0] - w[i - 1][0], dy = w[i][1] - w[i - 1][1];
    const seg = Math.sqrt(dx * dx + dy * dy);
    let colour = onCol, dashed = false;
    if (anyUnverified && seg >= longHop) { colour = unvCol; dashed = true; }
    else if (mask) colour = (mask[i - 1] === '1') ? onCol : offCol;
    else colour = onCol;
    ctx.strokeStyle = colour;
    ctx.setLineDash(dashed ? [6, 5] : []);
    ctx.beginPath();
    ctx.moveTo(a[0], a[1]);
    ctx.lineTo(b[0], b[1]);
    ctx.stroke();
  }
  ctx.setLineDash([]);

  // Climbs: where move_along_surface cannot go and the consumer must step
  // directly. Worth seeing on the map, since they explain vertical jumps.
  ctx.fillStyle = CSSVAR('--climb');
  for (const i of climbs) {
    if (i >= w.length) continue;
    const p = toScreen(w[i][0], w[i][1]);
    ctx.beginPath();
    ctx.arc(p[0], p[1], 4, 0, Math.PI * 2);
    ctx.fill();
  }

  // Where a walk stops short of a goal on another level, mark the stop.
  if (r.reached_goal === false && w.length) {
    const p = toScreen(w[w.length - 1][0], w[w.length - 1][1]);
    ctx.strokeStyle = CSSVAR('--unv');
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(p[0] - 6, p[1] - 6); ctx.lineTo(p[0] + 6, p[1] + 6);
    ctx.moveTo(p[0] + 6, p[1] - 6); ctx.lineTo(p[0] - 6, p[1] + 6);
    ctx.stroke();
  }
}

function drawEndpoint(pt, colour, label) {
  if (!pt) return;
  const p = toScreen(pt[0], pt[1]);
  ctx.strokeStyle = colour;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(p[0], p[1], 6, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(p[0], p[1] - 10); ctx.lineTo(p[0], p[1] + 10);
  ctx.moveTo(p[0] - 10, p[1]); ctx.lineTo(p[0] + 10, p[1]);
  ctx.stroke();
  ctx.fillStyle = colour;
  ctx.font = '11px ui-monospace, monospace';
  ctx.fillText(label, p[0] + 9, p[1] - 8);
}

// --- interaction ----------------------------------------------------------
let dragging = false, dragged = false, lastX = 0, lastY = 0;
let nodeTimer = 0;

function scheduleNodes() {
  clearTimeout(nodeTimer);
  nodeTimer = setTimeout(refreshNodes, 250);   // debounce: pan/zoom fire fast
}

canvas.addEventListener('mousedown', (e) => {
  dragging = true; dragged = false;
  lastX = e.offsetX; lastY = e.offsetY;
});
canvas.addEventListener('mousemove', (e) => {
  const dpr = window.devicePixelRatio || 1;
  const [wx, wy] = toWorld(e.offsetX * dpr, e.offsetY * dpr);
  el('coords').textContent = fmt(wx, 1) + ', ' + fmt(wy, 1);
  if (!dragging) return;
  const dx = e.offsetX - lastX, dy = e.offsetY - lastY;
  if (Math.abs(dx) + Math.abs(dy) > 2) dragged = true;
  view.x -= dx * dpr / view.scale;
  view.y += dy * dpr / view.scale;
  lastX = e.offsetX; lastY = e.offsetY;
  draw();
  scheduleNodes();
});
window.addEventListener('mouseup', () => { dragging = false; });

// A click that moved the view was a pan, not a pick.
canvas.addEventListener('click', (e) => {
  if (dragged) return;
  const dpr = window.devicePixelRatio || 1;
  setPoint('from', toWorld(e.offsetX * dpr, e.offsetY * dpr));
});
canvas.addEventListener('contextmenu', (e) => {
  e.preventDefault();
  if (dragged) return;
  const dpr = window.devicePixelRatio || 1;
  setPoint('to', toWorld(e.offsetX * dpr, e.offsetY * dpr));
});

// A clicked point has no height, and every query wants one. Ask the service
// for the ground under it rather than guessing - that is what find_ground_z is
// for, and it makes the clicked coordinate usable as a real world position.
async function setPoint(which, xy) {
  let z = 0;
  try {
    const g = await send('find_ground_z', { x: xy[0], y: xy[1] });
    if (g.found) z = g.z;
  } catch (e) { /* keep 0: still a usable guess for a flat area */ }
  const pt = [Math.round(xy[0] * 100) / 100, Math.round(xy[1] * 100) / 100,
              Math.round(z * 100) / 100];
  if (which === 'from') { from = pt; el('fromtxt').textContent = pt.join(', '); }
  else { to = pt; el('totxt').textContent = pt.join(', '); }
  draw();
  if (from && to) runQuery();
}

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const dpr = window.devicePixelRatio || 1;
  const before = toWorld(e.offsetX * dpr, e.offsetY * dpr);
  const k = Math.exp(-e.deltaY * 0.0015);
  view.scale = Math.min(8, Math.max(0.04, view.scale * k));
  const after = toWorld(e.offsetX * dpr, e.offsetY * dpr);
  // Keep the world point under the cursor fixed while zooming.
  view.x += before[0] - after[0];
  view.y += before[1] - after[1];
  draw();
  scheduleNodes();
}, { passive: false });

el('connect').addEventListener('click', connect);
el('run').addEventListener('click', runQuery);
el('clear').addEventListener('click', () => {
  from = to = route = null;
  el('fromtxt').textContent = el('totxt').textContent = '--';
  el('stats').innerHTML = '<div class="empty">no route yet</div>';
  draw();
});
el('swap').addEventListener('click', () => {
  const t = from; from = to; to = t;
  el('fromtxt').textContent = from ? from.join(', ') : '--';
  el('totxt').textContent = to ? to.join(', ') : '--';
  if (from && to) runQuery(); else draw();
});
el('cost').addEventListener('input', () => {
  el('costval').textContent = parseFloat(el('cost').value).toFixed(1);
});
el('cost').addEventListener('change', () => { if (from && to) runQuery(); });
el('qtype').addEventListener('change', () => { if (from && to) runQuery(); });
for (const id of ['showveh', 'showped']) el(id).addEventListener('change', refreshNodes);
el('showcorr').addEventListener('change', draw);

resize();
connect();
