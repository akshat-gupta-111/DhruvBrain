import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const stage = document.getElementById('model-stage');
const canvas = document.getElementById('robot-canvas');
const scene = new THREE.Scene();
scene.background = null;
const camera = new THREE.PerspectiveCamera(38, 1, 0.1, 100);
camera.position.set(1.55, 0.95, 2.15);

const renderer = new THREE.WebGLRenderer({canvas, antialias:true, alpha:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.setClearColor(0x000000, 0);

scene.add(new THREE.HemisphereLight(0xc9e8ff, 0x17202b, 2.1));
const keyLight = new THREE.DirectionalLight(0xb4efff, 2.5);
keyLight.position.set(2, 3, 2); scene.add(keyLight);
const rimLight = new THREE.PointLight(0x32cde9, 22, 5);
rimLight.position.set(-1, 0.6, -1); scene.add(rimLight);
const redLight = new THREE.PointLight(0x8b0000, 1.8, 0.55);
redLight.position.set(0, 0.38, 0); scene.add(redLight);

const robot = new THREE.Group();
scene.add(robot);

// Geometry uses meters as the visual unit: 0.66 m high, 0.22 m diameter.
const height = 0.66;
const radius = 0.11;
const cylinderGeo = new THREE.CylinderGeometry(radius, radius, height, 64, 1, true);
const cylinderMat = new THREE.MeshPhysicalMaterial({
  color: 0x77d9eb, metalness: 0.12, roughness: 0.18,
  transparent: true, opacity: 0.20, side: THREE.DoubleSide,
  depthWrite: false, clearcoat: 0.7, clearcoatRoughness: 0.2
});
const cylinder = new THREE.Mesh(cylinderGeo, cylinderMat);
cylinder.position.y = height / 2;
robot.add(cylinder);

const edgeMat = new THREE.LineBasicMaterial({color:0x6edcf0, transparent:true, opacity:0.85});
const topRing = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.002, 8, 64), new THREE.MeshBasicMaterial({color:0x80e8f7}));
topRing.rotation.x = Math.PI / 2; topRing.position.y = height; robot.add(topRing);
const bottomRing = new THREE.Mesh(new THREE.TorusGeometry(radius, 0.002, 8, 64), new THREE.MeshBasicMaterial({color:0x80e8f7}));
bottomRing.rotation.x = Math.PI / 2; robot.add(bottomRing);
const verticalEdges = new THREE.BufferGeometry().setFromPoints([
  new THREE.Vector3(-radius, 0, 0), new THREE.Vector3(-radius, height, 0),
  new THREE.Vector3(radius, 0, 0), new THREE.Vector3(radius, height, 0),
  new THREE.Vector3(0, 0, -radius), new THREE.Vector3(0, height, -radius),
  new THREE.Vector3(0, 0, radius), new THREE.Vector3(0, height, radius)
]);
robot.add(new THREE.LineSegments(verticalEdges, edgeMat));

// Dark-red IMU dot is at the center of the cylinder, 38 cm above the base.
const sensorY = 0.38;
const sensor = new THREE.Group();
sensor.position.set(0, sensorY, 0);
const sensorDot = new THREE.Mesh(
  new THREE.SphereGeometry(0.013, 24, 24),
  new THREE.MeshBasicMaterial({color: 0x8b0000})
);
sensor.add(sensorDot);
const sensorGlow = new THREE.Mesh(
  new THREE.SphereGeometry(0.021, 24, 24),
  new THREE.MeshBasicMaterial({color: 0x8b0000, transparent: true, opacity: 0.18, depthWrite: false})
);
sensor.add(sensorGlow);
robot.add(sensor);

// A small front marker makes heading changes easier to see.
const nose = new THREE.Mesh(new THREE.BoxGeometry(0.018, 0.014, 0.025), new THREE.MeshBasicMaterial({color:0x53d9ee}));
nose.position.set(0, height + 0.005, radius * 0.55);
robot.add(nose);

const floorGrid = new THREE.GridHelper(1.0, 20, 0x263846, 0x17232d);
floorGrid.position.y = -0.004;
scene.add(floorGrid);
const baseShadow = new THREE.Mesh(new THREE.CircleGeometry(0.19, 48), new THREE.MeshBasicMaterial({color:0x070a0e, transparent:true, opacity:0.7}));
baseShadow.rotation.x = -Math.PI/2; baseShadow.position.y = -0.006; scene.add(baseShadow);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.target.set(0, 0.32, 0);
controls.minDistance = 0.9;
controls.maxDistance = 4;
controls.update();

function resize() {
  const w = stage.clientWidth, h = stage.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(stage);
resize();

let targetRoll = 0, targetPitch = 0, targetYaw = 0;
let currentRoll = 0, currentPitch = 0, currentYaw = 0;
function radians(deg) { return deg * Math.PI / 180; }
function animate() {
  requestAnimationFrame(animate);
  // Apply the newest sensor pose directly; no smoothing filter.
  currentRoll = targetRoll; currentPitch = targetPitch; currentYaw = targetYaw;
  robot.rotation.order = 'YXZ';
  robot.rotation.x = radians(currentPitch);
  robot.rotation.z = -radians(currentRoll);
  robot.rotation.y = -radians(currentYaw);
  controls.update();
  renderer.render(scene, camera);
}
animate();

const socket = io();
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const terminalState = document.getElementById('terminal-state');
const terminal = document.getElementById('terminal');
const logCount = document.getElementById('log-count');
let lineCount = 0;
let followLogs = true;

function setStatus(connected, source) {
  statusDot.classList.toggle('connected', !!connected);
  statusText.textContent = connected ? `IMU CONNECTED · ${source || 'I2C'}` : 'IMU DISCONNECTED';
  terminalState.textContent = connected ? 'STREAM ACTIVE' : 'NO SENSOR DATA';
  terminalState.style.color = connected ? '#48e0a0' : '#ff6674';
}
socket.on('connect', () => { terminalState.textContent = 'SOCKET CONNECTED'; });
socket.on('disconnect', () => setStatus(false, 'socket'));
socket.on('status', data => setStatus(data.connected, data.source));
socket.on('imu', data => {
  if (!data) return;
  targetRoll = Number(data.roll || 0);
  targetPitch = Number(data.pitch || 0);
  targetYaw = Number(data.yaw || 0);
  setText('roll', targetRoll.toFixed(2), '°');
  setText('pitch', targetPitch.toFixed(2), '°');
  setText('yaw', targetYaw.toFixed(2), '°');
  setText('ax', Number(data.ax || 0).toFixed(4), 'g');
  setText('ay', Number(data.ay || 0).toFixed(4), 'g');
  setText('az', Number(data.az || 0).toFixed(4), 'g');
  setText('gx', Number(data.gx || 0).toFixed(2), '°/s');
  setText('gy', Number(data.gy || 0).toFixed(2), '°/s');
  setText('gz', Number(data.gz || 0).toFixed(2), '°/s');
  document.getElementById('sample-time').textContent = data.timestamp ? new Date(data.timestamp).toLocaleTimeString() : '—';
  meter('roll-meter', targetRoll, 90);
  meter('pitch-meter', targetPitch, 90);
  meter('yaw-meter', targetYaw, 180);
});
function setText(id, value, unit) {
  const el = document.getElementById(id);
  el.innerHTML = `${value}<span>${unit}</span>`;
}
function meter(id, value, range) {
  const width = Math.min(100, Math.max(0, (Math.abs(value) / range) * 100));
  document.getElementById(id).style.width = `${width}%`;
}
function addLog(line) {
  const el = document.createElement('div');
  el.className = 'terminal-line';
  const safeLine = String(line);
  const low = safeLine.toLowerCase();
  const colorClass = low.includes('[error]') ? 'log-error' : low.includes('[warn]') ? 'log-warn' : low.includes('[info]') ? 'log-info' : '';
  const span = document.createElement('span');
  span.className = colorClass;
  span.textContent = safeLine;
  el.appendChild(span);
  terminal.appendChild(el);
  lineCount++;
  while (terminal.children.length > 250) terminal.removeChild(terminal.firstChild);
  logCount.textContent = `${Math.min(lineCount,250)} lines`;
  if (followLogs) terminal.scrollTop = terminal.scrollHeight;
}
socket.on('log', data => { if (data && data.line) addLog(data.line); });
(window.INITIAL_LOGS || []).forEach(addLog);
terminal.addEventListener('scroll', () => {
  followLogs = terminal.scrollTop + terminal.clientHeight >= terminal.scrollHeight - 12;
});
document.getElementById('clear-logs').addEventListener('click', () => {
  terminal.innerHTML = '';
  lineCount = 0;
  logCount.textContent = '0 lines';
});
