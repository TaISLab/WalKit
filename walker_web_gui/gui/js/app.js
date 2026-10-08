import { RosLink, StatField } from './ros-link.js';
import { WalkerPlot } from './plot.js';

const STORAGE_KEY = 'walker_gui_ros_url';
const $ = (id) => document.getElementById(id);

function defaultRosUrl() {
  const params = new URLSearchParams(location.search);
  if (params.get('ros')) return params.get('ros');
  const saved = localStorage.getItem(STORAGE_KEY);
  if (saved) return saved;
  const host = location.hostname || '10.42.0.1';
  return `ws://${host}:9090`;
}

const round2 = (v) => Math.round((v + Number.EPSILON) * 100) / 100;

// ---------------------------------------------------------------------
// Connection status UI
// ---------------------------------------------------------------------
const rosLink = new RosLink(defaultRosUrl());
$('connUrlInput').value = rosLink.url;

const STATE_LABEL = {
  idle: 'Desconectado',
  connecting: 'Conectando…',
  connected: 'Conectado',
  reconnecting: 'Reconectando…',
  error: 'Error de conexión',
};

function renderConnState() {
  const dot = $('connDot');
  const text = $('connText');
  dot.className = 'conn-dot conn-' + rosLink.state;
  let label = STATE_LABEL[rosLink.state] || rosLink.state;
  if (rosLink.state === 'reconnecting' && rosLink.nextRetryAt) {
    const s = Math.max(0, Math.round((rosLink.nextRetryAt - Date.now()) / 1000));
    label += ` (${s}s)`;
  }
  text.textContent = label;
}

rosLink.addEventListener('statechange', renderConnState);
setInterval(renderConnState, 1000);

$('connEditBtn').addEventListener('click', () => {
  $('connEditor').hidden = !$('connEditor').hidden;
});
$('connCancelBtn').addEventListener('click', () => {
  $('connEditor').hidden = true;
});
$('connApplyBtn').addEventListener('click', () => {
  const url = $('connUrlInput').value.trim();
  if (!url) return;
  localStorage.setItem(STORAGE_KEY, url);
  $('connEditor').hidden = true;
  rosLink.connect(url);
});
$('connRetryBtn').addEventListener('click', () => rosLink.reconnectNow());

rosLink.connect();

// ---------------------------------------------------------------------
// Tabs
// ---------------------------------------------------------------------
document.querySelectorAll('.tab-btn').forEach((btn) => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab-btn').forEach((b) => b.classList.remove('active'));
    document.querySelectorAll('.tab-panel').forEach((p) => p.classList.remove('active'));
    btn.classList.add('active');
    $(btn.dataset.tab).classList.add('active');
  });
});

// ---------------------------------------------------------------------
// Sensor stat fields
// ---------------------------------------------------------------------
const fields = {
  leftHandleForce: new StatField($('leftHandleForce'), { format: (v) => round2(v), ageEl: $('leftHandleForceAge') }),
  rightHandleForce: new StatField($('rightHandleForce'), { format: (v) => round2(v), ageEl: $('rightHandleForceAge') }),
  leftLegLoad: new StatField($('leftLegLoad'), { format: (v) => round2(v), ageEl: $('leftLegLoadAge') }),
  rightLegLoad: new StatField($('rightLegLoad'), { format: (v) => round2(v), ageEl: $('rightLegLoadAge') }),
  leftWheel: new StatField($('leftWheel'), { ageEl: $('leftWheelAge') }),
  rightWheel: new StatField($('rightWheel'), { ageEl: $('rightWheelAge') }),
  imuX: new StatField($('imuX'), { format: (v) => round2(v), ageEl: $('imuXAge') }),
  imuY: new StatField($('imuY'), { format: (v) => round2(v), ageEl: $('imuYAge') }),
  imuZ: new StatField($('imuZ'), { format: (v) => round2(v), ageEl: $('imuZAge') }),
  handlePos: new StatField($('handlePos'), { staleMs: 60000, deadMs: 120000, ageEl: $('handlePosAge') }),
};

const plot = new WalkerPlot('myPlot');

setInterval(() => {
  const now = Date.now();
  Object.values(fields).forEach((f) => f.tick(now));
  plot.tick(now);
}, 500);

// ---------------------------------------------------------------------
// Topics
// ---------------------------------------------------------------------
function topic(name, messageType, latch = false) {
  return new ROSLIB.Topic({ ros: rosLink.ros, name, messageType, queue_length: 0, latch });
}

topic('/sensor/bwt901cl/Angle', 'geometry_msgs/msg/Vector3').subscribe((m) => {
  fields.imuX.update(m.x);
  fields.imuY.update(m.y);
  fields.imuZ.update(m.z);
});

topic('/left_wheel', 'walker_msgs/msg/EncoderStamped').subscribe((m) => fields.leftWheel.update(m.encoder));
topic('/right_wheel', 'walker_msgs/msg/EncoderStamped').subscribe((m) => fields.rightWheel.update(m.encoder));

const handleHeightTopic = topic('/handle_height', 'std_msgs/msg/Int32', true);
handleHeightTopic.subscribe((m) => fields.handlePos.update(m.data));

topic('/left_loads', 'walker_msgs/msg/StepStamped').subscribe((m) => {
  fields.leftLegLoad.update(m.load);
  const x = m.position.point.x, y = m.position.point.y;
  if (x !== 0 || y !== 0) plot.setFoot('left', -y, x, m.load);
});

topic('/right_loads', 'walker_msgs/msg/StepStamped').subscribe((m) => {
  fields.rightLegLoad.update(m.load);
  const x = m.position.point.x, y = m.position.point.y;
  if (x !== 0 || y !== 0) plot.setFoot('right', -y, x, m.load);
});

topic('/laser_poses', 'geometry_msgs/msg/PoseArray').subscribe((m) => {
  const xs = [], ys = [];
  for (const pose of m.poses) {
    xs.push(-pose.position.y);
    ys.push(pose.position.x);
  }
  plot.setLaser(xs, ys);
});

let leftHandLoad = 0, rightHandLoad = 0;
topic('/left_hand_loads', 'walker_msgs/msg/ForceStamped').subscribe((m) => {
  fields.leftHandleForce.update(m.force);
  leftHandLoad = m.force;
  plot.setHandleLoads(leftHandLoad, rightHandLoad);
});
topic('/right_hand_loads', 'walker_msgs/msg/ForceStamped').subscribe((m) => {
  fields.rightHandleForce.update(m.force);
  rightHandLoad = m.force;
  plot.setHandleLoads(leftHandLoad, rightHandLoad);
});

topic('/odom', 'nav_msgs/msg/Odometry').subscribe((m) => {
  const angThreshold = 0.15, linThreshold = 0.05;
  let ang = m.twist.twist.angular.x;
  let lin = m.twist.twist.linear.x;
  ang = Math.abs(ang) < angThreshold ? 0 : (ang / Math.abs(ang)) * -0.2;
  lin = Math.abs(lin) < linThreshold ? 0 : (lin / Math.abs(lin)) * -0.2;
  plot.setAdvance(ang, lin);
});

const userTopic = topic('/user_desc', 'walker_msgs/msg/UserDesc', true);
userTopic.subscribe((m) => {
  $('userDescData').textContent =
    `${m.user_id} (${m.gender}), ${m.age} años, ${m.height} cm, ${m.weight} kg, ` +
    `tinetti ${m.tinetti_score} - ${m.description}`;
});

// ---------------------------------------------------------------------
// Configuration (user data + handle height), published explicitly via the
// "Actualizar configuración" button. The button turns red whenever the form
// differs from what was last published.
// ---------------------------------------------------------------------
function readUser() {
  return {
    user_id: $('userId').value,
    gender: $('genderTog').checked ? 'Femenino' : 'Masculino',
    age: parseInt($('ageRange').value, 10),
    height: parseInt($('heightRange').value, 10),
    weight: parseInt($('weightRange').value, 10),
    tinetti_score: parseInt($('tinettiRange').value, 10),
    description: $('conditionText').value,
  };
}
const readHandle = () => parseInt($('handleRange').value, 10);

let lastUser = null;   // JSON of the last published user_desc
let lastHandle = null; // last published handle_height

function updatePending() {
  const dirty = JSON.stringify(readUser()) !== lastUser || readHandle() !== lastHandle;
  $('handleApplyBtn').classList.toggle('btn-pending', dirty);
}

function publishUser() {
  const user = readUser();
  userTopic.publish(new ROSLIB.Message(user));
  lastUser = JSON.stringify(user);
  updatePending();
}

function publishHandle() {
  const h = readHandle();
  handleHeightTopic.publish(new ROSLIB.Message({ data: h }));
  lastHandle = h;
  updatePending();
}

[['ageRange', 'ageValue'], ['heightRange', 'heightValue'], ['weightRange', 'weightValue'],
 ['tinettiRange', 'tinettiValue'], ['handleRange', 'handleValue']].forEach(([rangeId, outId]) => {
  const range = $(rangeId), out = $(outId);
  out.textContent = range.value;
  range.addEventListener('input', () => { out.textContent = range.value; });
});

$('genderTog').addEventListener('change', () => {
  $('genderStatus').textContent = $('genderTog').checked ? 'Femenino' : 'Masculino';
});

$('tabSession').addEventListener('input', updatePending);
$('tabSession').addEventListener('change', updatePending);

$('handleApplyBtn').addEventListener('click', () => {
  publishUser();
  publishHandle();
});

updatePending();
