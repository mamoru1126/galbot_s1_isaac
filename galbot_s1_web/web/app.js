// Galbot S1 web console. Talks to the commander through rosbridge:
//   /galbot/command (std_msgs/String JSON)  ->  commander
//   /galbot/status  (std_msgs/String JSON)  <-  commander (5 Hz)
//   /cmd_vel        (geometry_msgs/Twist)   ->  base controller
//   /galbot/web/<camera>/compressed (sensor_msgs/CompressedImage) <- camera relay
'use strict';

const $ = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

const cfg = window.GALBOT_CONFIG || {};
const params = new URLSearchParams(location.search);
const host = location.hostname || 'localhost';
const wsUrl = params.get('ws') || `ws://${host}:${cfg.rosbridgePort || 9090}`;

const state = {
  status: null,
  selectedObject: null,
  demoArm: 'auto',
  arm: 'right',
  jogStep: 0.02,
  camera: 'overview',
  dragging: new Set(),
  scene: null,
};

// ------------------------------------------------------------------ rosbridge
const connText = { connecting: '接続中…', connected: '接続済み', disconnected: '切断されました', unreachable: 'rosbridge に接続できません' };
const ros = new RosBridge(wsUrl, (s) => {
  $('#conn').dataset.state = s;
  $('#connText').textContent = connText[s] || s;
});

function send(cmd) {
  ros.publish('/galbot/command', 'std_msgs/String', { data: JSON.stringify(cmd) });
}

ros.subscribe('/galbot/status', 'std_msgs/String', (msg) => {
  try { state.status = JSON.parse(msg.data); } catch (e) { return; }
  onStatus();
}, { queue_length: 1 });

// ------------------------------------------------------------------ tabs / segmented
$$('#tabs button').forEach((b) => b.addEventListener('click', () => {
  $$('#tabs button').forEach((x) => x.classList.toggle('on', x === b));
  $$('.tab').forEach((t) => t.classList.toggle('on', t.id === 'tab-' + b.dataset.tab));
}));

function segment(root, attr, onPick) {
  $$(`${root} button`).forEach((b) => b.addEventListener('click', () => {
    $$(`${root} button`).forEach((x) => x.classList.toggle('on', x === b));
    onPick(b.dataset[attr]);
  }));
}
segment('#demoArm', 'arm', (v) => { state.demoArm = v; });
segment('#armSel', 'arm', (v) => { state.arm = v; buildArmSliders(); renderHolding(); });
segment('#jogStep', 'step', (v) => { state.jogStep = parseFloat(v); });
segment('#camSel', 'cam', (v) => setCamera(v));

$('#estop').addEventListener('click', () => { send({ type: 'stop' }); stopDrive(true); });

// ------------------------------------------------------------------ demo tab
$$('#tab-demo [data-task]').forEach((b) => b.addEventListener('click', () => {
  const task = b.dataset.task;
  if (task === 'reset_scene') { send({ type: 'reset_scene' }); return; }
  const cmd = { type: 'task', name: task };
  if (['demo', 'pick', 'approach'].includes(task)) {
    if (!state.selectedObject) { flash('物体を選んでください'); return; }
    cmd.object = state.selectedObject;
    cmd.arm = state.demoArm;
  }
  if (task === 'place') { cmd.target = b.dataset.target; cmd.arm = state.demoArm; }
  send(cmd);
}));

function flash(text) {
  const el = $('#taskLine');
  el.textContent = text;
  el.classList.add('failed');
  setTimeout(() => el.classList.remove('failed'), 1500);
}

function buildChips(scene) {
  const box = $('#objChips');
  box.innerHTML = '';
  scene.objects.forEach((o) => {
    const c = document.createElement('button');
    c.className = 'chip';
    c.dataset.name = o.name;
    const [r, g, b] = o.color.map((v) => Math.round(v * 255));
    c.innerHTML = `<span class="sw" style="background:rgb(${r},${g},${b})"></span>${o.label}`;
    c.addEventListener('click', () => selectObject(o.name));
    box.appendChild(c);
  });
  if (!state.selectedObject && scene.objects.length) selectObject(scene.objects[0].name);
}

function selectObject(name) {
  state.selectedObject = name;
  $$('#objChips .chip').forEach((c) => c.classList.toggle('on', c.dataset.name === name));
  drawMap();
}

// ------------------------------------------------------------------ driving
const drive = { joy: [0, 0], rot: 0, keys: new Set(), timer: null, lastZero: true };
const joy = $('#joy');
const knob = $('#knob');

function joyFromEvent(ev) {
  const r = joy.getBoundingClientRect();
  let x = (ev.clientX - r.left) / r.width * 2 - 1;
  let y = (ev.clientY - r.top) / r.height * 2 - 1;
  const n = Math.hypot(x, y);
  if (n > 1) { x /= n; y /= n; }
  drive.joy = [x, y];
  knob.style.transform = `translate(${x * 100}%, ${y * 100}%)`;
}
joy.addEventListener('pointerdown', (ev) => {
  joy.setPointerCapture(ev.pointerId);
  joy.classList.add('active');
  joyFromEvent(ev);
  startDrive();
});
joy.addEventListener('pointermove', (ev) => { if (joy.classList.contains('active')) joyFromEvent(ev); });
const joyEnd = () => {
  joy.classList.remove('active');
  drive.joy = [0, 0];
  knob.style.transform = 'translate(0, 0)';
};
joy.addEventListener('pointerup', joyEnd);
joy.addEventListener('pointercancel', joyEnd);

function holdButton(el, onDown, onUp) {
  el.addEventListener('pointerdown', (ev) => { el.setPointerCapture(ev.pointerId); el.classList.add('active'); onDown(); });
  const up = () => { if (el.classList.contains('active')) { el.classList.remove('active'); onUp(); } };
  el.addEventListener('pointerup', up);
  el.addEventListener('pointercancel', up);
  el.addEventListener('contextmenu', (e) => e.preventDefault());
}
holdButton($('#rotL'), () => { drive.rot = 1; startDrive(); }, () => { drive.rot = 0; });
holdButton($('#rotR'), () => { drive.rot = -1; startDrive(); }, () => { drive.rot = 0; });

const keyMap = { w: 1, s: 1, a: 1, d: 1, q: 1, e: 1 };
window.addEventListener('keydown', (ev) => {
  if (ev.target.tagName === 'INPUT') return;
  const k = ev.key.toLowerCase();
  if (keyMap[k]) { drive.keys.add(k); startDrive(); }
  if (ev.key === ' ') { send({ type: 'stop' }); stopDrive(true); }
});
window.addEventListener('keyup', (ev) => drive.keys.delete(ev.key.toLowerCase()));
window.addEventListener('blur', () => { drive.keys.clear(); joyEnd(); drive.rot = 0; });

$('#speed').addEventListener('input', (ev) => { $('#speedVal').textContent = `${parseFloat(ev.target.value).toFixed(2)} m/s`; });

function driveCommand() {
  const vmax = parseFloat($('#speed').value);
  let vx = -drive.joy[1];
  let vy = -drive.joy[0];
  let wz = drive.rot;
  if (drive.keys.has('w')) vx += 1;
  if (drive.keys.has('s')) vx -= 1;
  if (drive.keys.has('a')) vy += 1;
  if (drive.keys.has('d')) vy -= 1;
  if (drive.keys.has('q')) wz += 1;
  if (drive.keys.has('e')) wz -= 1;
  const n = Math.hypot(vx, vy);
  if (n > 1) { vx /= n; vy /= n; }
  return [vx * vmax, vy * vmax, Math.max(-1, Math.min(1, wz)) * Math.min(0.8, vmax * 2.0)];
}

function startDrive() {
  if (state.status && state.status.task && state.status.task.state === 'running') {
    send({ type: 'stop' }); // manual driving overrides a running task
  }
  if (drive.timer) return;
  drive.timer = setInterval(() => {
    const [vx, vy, wz] = driveCommand();
    const zero = Math.abs(vx) + Math.abs(vy) + Math.abs(wz) < 1e-3;
    $('#driveReadout').textContent = `vx ${vx.toFixed(2)} / vy ${vy.toFixed(2)} / ω ${wz.toFixed(2)}`;
    publishTwist(vx, vy, wz);
    if (zero) stopDrive(false);
  }, 100);
}

function stopDrive(sendZero) {
  if (drive.timer) { clearInterval(drive.timer); drive.timer = null; }
  if (sendZero) publishTwist(0, 0, 0);
}

function publishTwist(vx, vy, wz) {
  ros.publish('/cmd_vel', 'geometry_msgs/Twist', {
    linear: { x: vx, y: vy, z: 0 }, angular: { x: 0, y: 0, z: wz },
  });
}

// ------------------------------------------------------------------ arm tab
$('#gripOpen').addEventListener('click', () => send({ type: 'gripper', arm: state.arm, action: 'open' }));
$('#gripClose').addEventListener('click', () => send({ type: 'gripper', arm: state.arm, action: 'close' }));

$$('.jog [data-jog]').forEach((b) => {
  let timer = null;
  const fire = () => {
    const [x, y, z, yaw] = b.dataset.jog.split(',').map(Number);
    const s = state.jogStep;
    send({ type: 'jog', arm: state.arm, dx: x * s, dy: y * s, dz: z * s, dyaw: yaw * s * 4 });
  };
  holdButton(b, () => { fire(); timer = setInterval(fire, 280); }, () => { clearInterval(timer); timer = null; });
});

$$('[data-pose]').forEach((b) => b.addEventListener('click', () => send({ type: 'pose', name: b.dataset.pose })));
$$('[data-head]').forEach((b) => b.addEventListener('click', () => {
  const [pan, tilt] = b.dataset.head.split(',').map(Number);
  send({ type: 'head', pan, tilt });
}));

const JOINT_LABELS = {
  torso_lift_joint1: 'リフト [m]', head_joint1: '首 左右', head_joint2: '首 上下',
};

function makeSlider(container, joint, label) {
  const lim = (state.status && state.status.limits && state.status.limits[joint]) || [-3.14, 3.14];
  const wrap = document.createElement('label');
  wrap.className = 'slider';
  wrap.innerHTML = `${label} <span>0.00</span><input type="range" min="${lim[0]}" max="${lim[1]}" step="0.005">`;
  const input = $('input', wrap);
  const out = $('span', wrap);
  input.dataset.joint = joint;
  let last = 0;
  input.addEventListener('pointerdown', () => state.dragging.add(joint));
  const release = () => { state.dragging.delete(joint); };
  input.addEventListener('pointerup', release);
  input.addEventListener('pointercancel', release);
  input.addEventListener('input', () => {
    out.textContent = parseFloat(input.value).toFixed(2);
    const now = performance.now();
    if (now - last > 150) { last = now; send({ type: 'joint', name: joint, position: parseFloat(input.value) }); }
  });
  input.addEventListener('change', () => {
    send({ type: 'joint', name: joint, position: parseFloat(input.value) });
    state.dragging.delete(joint);
  });
  container.appendChild(wrap);
}

function buildArmSliders() {
  const box = $('#armJoints');
  box.innerHTML = '';
  for (let i = 1; i <= 7; i++) makeSlider(box, `${state.arm}_arm_joint${i}`, `J${i}`);
  updateSliders();
}

function buildBodySliders() {
  $('#torsoJoints').innerHTML = '';
  $('#headJoints').innerHTML = '';
  makeSlider($('#torsoJoints'), 'torso_lift_joint1', JOINT_LABELS.torso_lift_joint1);
  makeSlider($('#headJoints'), 'head_joint1', JOINT_LABELS.head_joint1);
  makeSlider($('#headJoints'), 'head_joint2', JOINT_LABELS.head_joint2);
  updateSliders();
}

function updateSliders() {
  const st = state.status;
  if (!st || !st.setpoint) return;
  $$('.sliders input').forEach((input) => {
    const j = input.dataset.joint;
    if (state.dragging.has(j) || !(j in st.setpoint)) return;
    input.value = st.setpoint[j];
    input.previousElementSibling.textContent = Number(st.setpoint[j]).toFixed(2);
  });
}

function renderHolding() {
  const st = state.status;
  const el = $('#holding');
  const held = st && st.holding ? st.holding[state.arm] : null;
  const label = held && state.scene ? (state.scene.objects.find((o) => o.name === held) || {}).label || held : held;
  el.textContent = held ? `${label} を持っています` : '何も持っていません';
  el.classList.toggle('on', !!held);
}

// ------------------------------------------------------------------ camera
let camUnsub = null;
let camTimer = null;
function setCamera(name) {
  state.camera = name;
  if (camUnsub) { camUnsub(); camUnsub = null; }
  const box = $('.cam-box');
  const img = $('#camImg');
  box.classList.remove('live');
  img.removeAttribute('src');
  if (name === 'off') { $('#camMsg').textContent = 'カメラ OFF'; return; }
  $('#camMsg').textContent = '映像待ち… (cameras:=true で起動していますか?)';
  camUnsub = ros.subscribe(`/galbot/web/${name}/compressed`, 'sensor_msgs/CompressedImage', (msg) => {
    img.src = 'data:image/jpeg;base64,' + msg.data;
    box.classList.add('live');
    clearTimeout(camTimer);
    camTimer = setTimeout(() => box.classList.remove('live'), 3000);
  }, { throttle_rate: 80, queue_length: 1 });
}

// ------------------------------------------------------------------ map
const canvas = $('#map');
const ctx = canvas.getContext('2d');
const view = { x0: -1.0, x1: 2.3, y0: -1.25, y1: 1.25 };  // world x (up), y (left)

function mapTransform() {
  const w = canvas.clientWidth;
  const h = Math.round(w * 0.78);
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    canvas.style.height = h + 'px';
  }
  const s = Math.min(w / (view.y1 - view.y0), h / (view.x1 - view.x0));
  const cx = w / 2;
  const cy = h / 2;
  const mx = (view.x0 + view.x1) / 2;
  const my = (view.y0 + view.y1) / 2;
  return {
    w, h, s, dpr,
    toScreen: (x, y) => [cx - (y - my) * s, cy - (x - mx) * s],
    toWorld: (px, py) => [mx + (cy - py) / s, my + (cx - px) / s],
  };
}

function rectWorld(T, x, y, sx, sy, yaw, fill, stroke) {
  ctx.save();
  const [px, py] = T.toScreen(x, y);
  ctx.translate(px, py);
  ctx.rotate(-yaw);
  ctx.fillStyle = fill;
  ctx.fillRect(-sy * T.s / 2, -sx * T.s / 2, sy * T.s, sx * T.s);
  if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 2; ctx.strokeRect(-sy * T.s / 2, -sx * T.s / 2, sy * T.s, sx * T.s); }
  ctx.restore();
}

function drawMap() {
  const T = mapTransform();
  ctx.setTransform(T.dpr, 0, 0, T.dpr, 0, 0);
  ctx.clearRect(0, 0, T.w, T.h);
  // grid
  ctx.strokeStyle = '#1f252d';
  ctx.lineWidth = 1;
  for (let x = Math.ceil(view.x0 * 2) / 2; x <= view.x1; x += 0.5) {
    const [a, b] = T.toScreen(x, view.y0); const [c, d] = T.toScreen(x, view.y1);
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
  }
  for (let y = Math.ceil(view.y0 * 2) / 2; y <= view.y1; y += 0.5) {
    const [a, b] = T.toScreen(view.x0, y); const [c, d] = T.toScreen(view.x1, y);
    ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
  }
  const sc = state.scene;
  const st = state.status;
  if (sc) {
    const [tx, ty] = sc.table.center;
    rectWorld(T, tx, ty, sc.table.size[0], sc.table.size[1], 0, '#5b4632', '#7a5f45');
    if (sc.tray) rectWorld(T, sc.tray.center[0], sc.tray.center[1], sc.tray.size[0], sc.tray.size[1], 0, '#24272e', '#4b515c');
    const [sx, sy] = T.toScreen(sc.robot_spawn[0], sc.robot_spawn[1]);
    ctx.strokeStyle = '#3b4552'; ctx.setLineDash([4, 4]);
    ctx.strokeRect(sx - 0.37 * T.s, sy - 0.37 * T.s, 0.74 * T.s, 0.74 * T.s);
    ctx.setLineDash([]);
  }
  if (st && st.objects && sc) {
    sc.objects.forEach((o) => {
      const p = st.objects[o.name];
      if (!p) return;
      const col = `rgb(${o.color.map((v) => Math.round(v * 255)).join(',')})`;
      const sel = state.selectedObject === o.name;
      const lifted = p.z > sc.table.size[2] + o.size[2] / 2 + 0.03;
      ctx.globalAlpha = lifted ? 0.65 : 1;
      if (o.shape === 'cylinder') {
        const [px, py] = T.toScreen(p.x, p.y);
        ctx.beginPath(); ctx.arc(px, py, Math.max(4, o.size[0] / 2 * T.s), 0, Math.PI * 2);
        ctx.fillStyle = col; ctx.fill();
        if (sel) { ctx.strokeStyle = '#ff7a1a'; ctx.lineWidth = 3; ctx.stroke(); }
      } else {
        rectWorld(T, p.x, p.y, o.size[0], o.size[1], p.yaw, col, sel ? '#ff7a1a' : null);
      }
      ctx.globalAlpha = 1;
      if (sel) {
        const [px, py] = T.toScreen(p.x, p.y);
        ctx.fillStyle = '#ffb27a'; ctx.font = '12px sans-serif'; ctx.textAlign = 'center';
        ctx.fillText(o.label, px, py - 12);
      }
    });
  }
  if (st && st.base) {
    const b = st.base;
    rectWorld(T, b.x, b.y, 0.74, 0.74, b.yaw, 'rgba(200,205,214,.18)', '#c8cdd6');
    const [px, py] = T.toScreen(b.x, b.y);
    const [hx, hy] = T.toScreen(b.x + 0.42 * Math.cos(b.yaw), b.y + 0.42 * Math.sin(b.yaw));
    ctx.strokeStyle = '#ff7a1a'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(hx, hy); ctx.stroke();
    ctx.fillStyle = '#c8cdd6'; ctx.font = '11px sans-serif'; ctx.textAlign = 'center';
    const side = (dy) => T.toScreen(b.x - dy * Math.sin(b.yaw), b.y + dy * Math.cos(b.yaw));
    const [lx, ly] = side(0.25); const [rx, ry] = side(-0.25);
    ctx.fillText('L', lx, ly + 4); ctx.fillText('R', rx, ry + 4);
    $('#poseText').textContent = `x ${b.x.toFixed(2)} / y ${b.y.toFixed(2)} / θ ${(b.yaw * 180 / Math.PI).toFixed(0)}°`;
  }
}

canvas.addEventListener('pointerdown', (ev) => {
  const st = state.status;
  if (!st || !st.objects || !state.scene) return;
  const r = canvas.getBoundingClientRect();
  const T = mapTransform();
  const [wx, wy] = T.toWorld(ev.clientX - r.left, ev.clientY - r.top);
  let best = null;
  for (const [name, p] of Object.entries(st.objects)) {
    const d = Math.hypot(p.x - wx, p.y - wy);
    if (d < 0.09 && (!best || d < best[1])) best = [name, d];
  }
  if (best) selectObject(best[0]);
});
window.addEventListener('resize', drawMap);

// ------------------------------------------------------------------ status
const TASK_NAMES = { demo: '全自動デモ', pick: '掴む', place: '置く', home: '戻る', approach: '移動', reset_scene: 'リセット', startup: '起動' };
function onStatus() {
  const st = state.status;
  if (st.scene && !state.scene) {
    state.scene = st.scene;
    buildChips(st.scene);
    buildArmSliders();
    buildBodySliders();
  }
  const t = st.task || {};
  const badge = $('#taskBadge');
  const tname = TASK_NAMES[(t.name || '').replace(/^_/, '')] || t.name || '';
  badge.dataset.state = st.ready ? (t.state || 'idle') : 'idle';
  if (!st.ready) badge.textContent = 'シミュレータ待ち';
  else if (t.state === 'running') badge.textContent = `${tname}: ${t.step || '実行中'}`;
  else if (t.state === 'failed') badge.textContent = `${tname} 失敗`;
  else if (t.state === 'done') badge.textContent = `${tname} 完了`;
  else if (t.state === 'cancelled') badge.textContent = '停止しました';
  else badge.textContent = '待機中';
  const line = $('#taskLine');
  line.classList.toggle('failed', t.state === 'failed');
  line.textContent = t.state === 'failed' ? `失敗: ${t.error}` : (t.state === 'running' ? `${tname}: ${t.step}` : badge.textContent);
  $$('#objChips .chip').forEach((c) => {
    const held = st.holding && Object.values(st.holding).includes(c.dataset.name);
    c.classList.toggle('held', !!held);
  });
  $('#log').textContent = (st.log || []).join('\n');
  updateSliders();
  renderHolding();
  drawMap();
}

setCamera('overview');
drawMap();
