/**
 * 视觉引导工业机器人智能操控系统 — 前端主控
 * ==============================================
 * FANUC LR Mate 200iD 六轴工业机器人 + 虚拟相机 + 数字孪生
 *
 * 开发模式: 直连 localhost 端口
 * 生产模式: 通过 Nginx 反向代理 (/api/*)
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

// 自动检测运行环境：开发(localhost)用直连，生产用相对路径
const IS_DEV = window.location.hostname === 'localhost'
  || window.location.hostname === '127.0.0.1'
  || window.location.protocol === 'file:';

const AGENT_URL = IS_DEV ? 'http://127.0.0.1:8803' : '/api/agent';
const ROBOT_URL = IS_DEV ? 'http://127.0.0.1:8802' : '/api/robot';
const VISION_URL = IS_DEV ? 'http://127.0.0.1:8801' : '/api/vision';

let isExecuting = false;
let abortController = null;

// ══════════════════════════════════════════════════════════════
// SCENE SETUP
// ══════════════════════════════════════════════════════════════

const container = document.getElementById('canvas-container');
const canvas = document.getElementById('robot-canvas');
const scene = new THREE.Scene();

// 程序化天空渐变背景
const bgCanvas = document.createElement('canvas');
bgCanvas.width = 2; bgCanvas.height = 512;
const bgCtx = bgCanvas.getContext('2d');
const bgGrad = bgCtx.createLinearGradient(0, 0, 0, 512);
bgGrad.addColorStop(0, '#1a2233');
bgGrad.addColorStop(0.5, '#151d2b');
bgGrad.addColorStop(1, '#0d1117');
bgCtx.fillStyle = bgGrad; bgCtx.fillRect(0, 0, 2, 512);
const bgTex = new THREE.CanvasTexture(bgCanvas);
scene.background = bgTex;
scene.fog = new THREE.Fog(0x111822, 800, 2500);

const camera = new THREE.PerspectiveCamera(45, 2, 10, 5000);
camera.position.set(800, 700, 900);
camera.lookAt(300, 400, 0);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.2;
renderer.outputColorSpace = THREE.SRGBColorSpace;

// OrbitControls
let controls;
try {
  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(300, 200, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;
  controls.target.set(300, 400, 0);
  controls.minDistance = 200;
  controls.maxDistance = 3000;
  controls.maxPolarAngle = Math.PI * 0.7;
  controls.update();
} catch (e) {
  console.warn('OrbitControls not available');
}

// ══════════════════════════════════════════════════════════════
// INDUSTRIAL LIGHTING (3-point + ambient)
// ══════════════════════════════════════════════════════════════

scene.add(new THREE.AmbientLight(0x334466, 2.0));

const keyLight = new THREE.DirectionalLight(0xffeedd, 5);
keyLight.position.set(600, 900, 500);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(2048, 2048);
keyLight.shadow.camera.near = 10;
keyLight.shadow.camera.far = 3000;
keyLight.shadow.camera.left = -1200;
keyLight.shadow.camera.right = 1200;
keyLight.shadow.camera.top = 1200;
keyLight.shadow.camera.bottom = -1200;
keyLight.shadow.bias = -0.0001;
keyLight.shadow.normalBias = 0.02;
scene.add(keyLight);

const fillLight = new THREE.DirectionalLight(0x8899cc, 1.5);
fillLight.position.set(-400, 300, -300);
scene.add(fillLight);

const rimLight = new THREE.DirectionalLight(0xffffff, 2.0);
rimLight.position.set(0, 200, -500);
scene.add(rimLight);

// ══════════════════════════════════════════════════════════════
// INDUSTRIAL GROUND (checker plate texture)
// ══════════════════════════════════════════════════════════════

const groundCanvas = document.createElement('canvas');
groundCanvas.width = 512; groundCanvas.height = 512;
const gctx = groundCanvas.getContext('2d');
gctx.fillStyle = '#1a1d2e'; gctx.fillRect(0, 0, 512, 512);
const tileSize = 64;
for (let row = 0; row < 8; row++) {
  for (let col = 0; col < 8; col++) {
    if ((row + col) % 2 === 0) {
      gctx.fillStyle = '#1e2235';
    } else {
      gctx.fillStyle = '#161927';
    }
    gctx.fillRect(col * tileSize, row * tileSize, tileSize, tileSize);
  }
}
// 细线网格
gctx.strokeStyle = '#2a3040'; gctx.lineWidth = 1;
for (let i = 0; i <= 8; i++) {
  gctx.beginPath(); gctx.moveTo(i * tileSize, 0); gctx.lineTo(i * tileSize, 512); gctx.stroke();
  gctx.beginPath(); gctx.moveTo(0, i * tileSize); gctx.lineTo(512, i * tileSize); gctx.stroke();
}
const groundTex = new THREE.CanvasTexture(groundCanvas);
groundTex.wrapS = THREE.RepeatWrapping;
groundTex.wrapT = THREE.RepeatWrapping;
groundTex.repeat.set(4, 4);

const groundGeo = new THREE.PlaneGeometry(2000, 2000);
const groundMat = new THREE.MeshStandardMaterial({
  map: groundTex, roughness: 0.85, metalness: 0.05
});
const ground = new THREE.Mesh(groundGeo, groundMat);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -1;
ground.receiveShadow = true;
scene.add(ground);

// Grid helper
const gridHelper = new THREE.GridHelper(1000, 20, 0x334466, 0x1a1a2e);
gridHelper.position.y = 0;
scene.add(gridHelper);

// ══════════════════════════════════════════════════════════════
// SAFETY FENCE (industrial guard rail)
// ══════════════════════════════════════════════════════════════

function createFencePost(x, z, h) {
  const postGeo = new THREE.CylinderGeometry(5, 6, h, 8);
  const postMat = new THREE.MeshStandardMaterial({ color: 0xddbb33, roughness: 0.3, metalness: 0.7 });
  const post = new THREE.Mesh(postGeo, postMat);
  post.position.set(x, h / 2, z);
  post.castShadow = true;
  post.receiveShadow = true;
  return post;
}

function createFenceRail(x1, z1, x2, z2, y) {
  const dx = x2 - x1, dz = z2 - z1;
  const len = Math.sqrt(dx * dx + dz * dz);
  const railGeo = new THREE.CylinderGeometry(3, 3, len, 8);
  const rail = new THREE.Mesh(railGeo, new THREE.MeshStandardMaterial({
    color: 0xddbb33, roughness: 0.3, metalness: 0.7
  }));
  rail.position.set((x1 + x2) / 2, y, (z1 + z2) / 2);
  rail.rotation.z = Math.PI / 2;
  rail.rotation.y = Math.atan2(dz, dx);
  rail.castShadow = true;
  return rail;
}

const fenceGroup = new THREE.Group();
const postPositions = [
  [-120, -250], [650, -250], [650, 250], [-120, 250]
];
const postHeight = 120;
const railYs = [40, 80, 120];

postPositions.forEach(([px, pz]) => {
  fenceGroup.add(createFencePost(px, pz, postHeight));
});

for (let i = 0; i < postPositions.length; i++) {
  const [x1, z1] = postPositions[i];
  const [x2, z2] = postPositions[(i + 1) % postPositions.length];
  railYs.forEach(y => {
    fenceGroup.add(createFenceRail(x1, z1, x2, z2, y));
  });
}

// E-Stop button on fence
const estopGeo = new THREE.SphereGeometry(8, 16, 16);
const estopMat = new THREE.MeshStandardMaterial({
  color: 0xff0000, roughness: 0.2, metalness: 0.1, emissive: 0xff0000, emissiveIntensity: 0.6
});
const estop = new THREE.Mesh(estopGeo, estopMat);
estop.position.set(-120, 100, -250);
fenceGroup.add(estop);

scene.add(fenceGroup);

// ══════════════════════════════════════════════════════════════
// STATUS LIGHT TOWER
// ══════════════════════════════════════════════════════════════

function createStatusTower(x, z) {
  const tower = new THREE.Group();
  const poleGeo = new THREE.CylinderGeometry(3, 3, 180, 8);
  const pole = new THREE.Mesh(poleGeo, new THREE.MeshStandardMaterial({
    color: 0x333333, roughness: 0.3, metalness: 0.5
  }));
  pole.position.y = 90;
  tower.add(pole);

  const colors = [0x00ff00, 0xffff00, 0xff0000];
  const labels = ['green', 'yellow', 'red'];
  colors.forEach((c, i) => {
    const lightGeo = new THREE.SphereGeometry(10, 16, 16);
    const lightMat = new THREE.MeshStandardMaterial({
      color: c, roughness: 0.1, metalness: 0.1, emissive: c, emissiveIntensity: 0.5
    });
    const light = new THREE.Mesh(lightGeo, lightMat);
    light.position.y = 140 + i * 28;
    light.name = `status-light-${labels[i]}`;
    tower.add(light);
  });
  tower.position.set(x, 0, z);
  return tower;
}

const statusTower = createStatusTower(680, -200);
scene.add(statusTower);

// Control cabinet
const cabinetGeo = new THREE.BoxGeometry(60, 160, 40);
const cabinet = new THREE.Mesh(cabinetGeo, new THREE.MeshStandardMaterial({
  color: 0x445566, roughness: 0.25, metalness: 0.8
}));
cabinet.position.set(680, 80, -130);
cabinet.castShadow = true;
cabinet.receiveShadow = true;
scene.add(cabinet);

// ══════════════════════════════════════════════════════════════
// INDUSTRIAL WORK TABLE + TRAYS
// ══════════════════════════════════════════════════════════════

const tableGroup = new THREE.Group();
scene.add(tableGroup);

// Table top (stainless steel plate)
const tableTopGeo = new THREE.BoxGeometry(700, 15, 500);
const tableTopMat = new THREE.MeshStandardMaterial({
  color: 0x8899aa, roughness: 0.2, metalness: 0.85
});
const tableTop = new THREE.Mesh(tableTopGeo, tableTopMat);
tableTop.position.set(300, 210, 0);
tableTop.castShadow = true;
tableTop.receiveShadow = true;
tableGroup.add(tableTop);

// Table edge trim
const edgeGeo = new THREE.BoxGeometry(710, 4, 510);
const edgeMat = new THREE.MeshStandardMaterial({
  color: 0x556677, roughness: 0.15, metalness: 0.95
});
const edge = new THREE.Mesh(edgeGeo, edgeMat);
edge.position.set(300, 202, 0);
edge.castShadow = true;
tableGroup.add(edge);

// 4 legs (rectangular tube)
for (const [lx, lz] of [[-25, -220], [-25, 220], [625, -220], [625, 220]]) {
  const legGeo = new THREE.BoxGeometry(35, 210, 35);
  const legMat = new THREE.MeshStandardMaterial({
    color: 0x3a4455, roughness: 0.25, metalness: 0.8
  });
  const leg = new THREE.Mesh(legGeo, legMat);
  leg.position.set(lx, 105, lz);
  leg.castShadow = true;
  leg.receiveShadow = true;
  tableGroup.add(leg);
}

// Cross braces
for (const [bx, bz, bw, bd] of [[150, -220, 300, 35], [150, 220, 300, 35], [625, 0, 35, 440]]) {
  const braceGeo = new THREE.BoxGeometry(bw, 10, bd);
  const brace = new THREE.Mesh(braceGeo, new THREE.MeshStandardMaterial({
    color: 0x3a4455, roughness: 0.3, metalness: 0.7
  }));
  brace.position.set(bx, 100, bz);
  brace.castShadow = true;
  tableGroup.add(brace);
}

// Tray models
function createTray(x, z, colorHex, label) {
  const group = new THREE.Group();
  const baseGeo = new THREE.BoxGeometry(90, 4, 90);
  const baseMat = new THREE.MeshStandardMaterial({ color: colorHex, roughness: 0.35, metalness: 0.4 });
  const base = new THREE.Mesh(baseGeo, baseMat);
  base.position.y = 0;
  base.castShadow = true;
  base.receiveShadow = true;
  group.add(base);

  const wallGeo = new THREE.BoxGeometry(90, 12, 5);
  const wallMat = new THREE.MeshStandardMaterial({ color: colorHex, roughness: 0.25, metalness: 0.6 });
  for (const [wx, wz, ry] of [[0, -42.5, 0], [0, 42.5, 0], [-42.5, 0, Math.PI / 2], [42.5, 0, Math.PI / 2]]) {
    const wall = new THREE.Mesh(wallGeo, wallMat);
    wall.position.set(wx, 8, wz);
    wall.rotation.y = ry;
    wall.castShadow = true;
    group.add(wall);
  }

  const labelCanvas = document.createElement('canvas');
  labelCanvas.width = 256; labelCanvas.height = 64;
  const ctx = labelCanvas.getContext('2d');
  ctx.fillStyle = '#111111'; ctx.fillRect(0, 0, 256, 64);
  ctx.font = 'bold 30px Arial'; ctx.fillStyle = '#ffffff';
  ctx.textAlign = 'center'; ctx.fillText(label, 128, 44);
  const labelTex = new THREE.CanvasTexture(labelCanvas);
  const labelPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(70, 18),
    new THREE.MeshBasicMaterial({ map: labelTex, transparent: true })
  );
  labelPlane.rotation.x = -Math.PI / 2;
  labelPlane.position.y = 14;
  group.add(labelPlane);

  group.position.set(x, 218, z);
  scene.add(group);
  return group;
}

const trayLeft = createTray(220, -120, 0xdd4444, 'Tray A');
const trayCenter = createTray(400, 0, 0xdddd44, 'Tray B');
const trayRight = createTray(220, 120, 0x44dd44, 'Tray C');

// ══════════════════════════════════════════════════════════════
// WORKSPACE OBJECTS (random distribution, varied shapes)
// ══════════════════════════════════════════════════════════════

const sceneObjects = [];
const objMeta = []; // { worldPos, color, shape, label }

function randomInRange(min, max) {
  return min + Math.random() * (max - min);
}

function generateWorkspaceObjects(count = 6) {
  // Clear existing
  sceneObjects.forEach(o => scene.remove(o));
  sceneObjects.length = 0;
  objMeta.length = 0;

  const zoneX = [280, 520];
  const zoneZ = [-150, 150];
  const minDist = 85;
  const shapes = ['cylinder', 'box', 'hexagon'];
  const colorNames = ['红色', '蓝色', '绿色', '黄色', '橙色', '紫色', '白色', '灰色'];
  const colorHexes = [0xdd3333, 0x3388dd, 0x33dd33, 0xdddd33, 0xdd8833, 0xaa33dd, 0xeeeeee, 0x999999];

  const placed = []; // { x, z }

  for (let i = 0; i < count; i++) {
    let px, pz, valid;
    let attempts = 0;
    do {
      px = randomInRange(zoneX[0] + 30, zoneX[1] - 30);
      pz = randomInRange(zoneZ[0] + 30, zoneZ[1] - 30);
      valid = placed.every(p => {
        const dx = p.x - px, dz = p.z - pz;
        return Math.sqrt(dx * dx + dz * dz) >= minDist;
      });
      attempts++;
    } while (!valid && attempts < 50);

    if (!valid) continue;
    placed.push({ x: px, z: pz });

    const ci = i % colorNames.length;
    const shapeIdx = i % shapes.length;
    const shape = shapes[shapeIdx];
    const colorName = colorNames[ci];
    const colorHex = colorHexes[ci];

    let geo;
    if (shape === 'cylinder') {
      geo = new THREE.CylinderGeometry(18, 18, 28, 32);
    } else if (shape === 'box') {
      geo = new THREE.BoxGeometry(32, 26, 32);
    } else {
      geo = new THREE.CylinderGeometry(20, 20, 24, 6);
    }

    const mat = new THREE.MeshStandardMaterial({
      color: colorHex, roughness: 0.3, metalness: 0.15
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(px, 228, pz);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.name = `object-${i}`;
    scene.add(mesh);
    sceneObjects.push(mesh);
    objMeta.push({
      worldPos: [px, pz, 228],
      color: colorName,
      shape: shape,
      label: `${colorName}${shape === 'cylinder' ? '圆柱' : shape === 'box' ? '方块' : '六角件'}`,
      mesh: mesh,
    });
  }
}

generateWorkspaceObjects(7);

function updateSceneObjects(objects) {
  // Called from vision detection result - sync 3D scene objects
  sceneObjects.forEach(o => scene.remove(o));
  sceneObjects.length = 0;
  objMeta.length = 0;
  if (!objects || !Array.isArray(objects)) return;

  const colorMap = {
    '红色': 0xdd3333, '橙色': 0xdd8833, '黄色': 0xdddd33,
    '绿色': 0x33dd33, '蓝色': 0x3388dd, '紫色': 0xaa33dd,
    '白色': 0xeeeeee, '黑色': 0x222222, '灰色': 0x999999,
  };

  objects.forEach((obj, i) => {
    const wxyz = obj.world_xyz || [400, 0, 350];
    const colorHex = colorMap[obj.color] || 0x888888;
    const geo = new THREE.CylinderGeometry(18, 18, 28, 32);
    const mat = new THREE.MeshStandardMaterial({ color: colorHex, roughness: 0.3, metalness: 0.15 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(wxyz[0], 228, wxyz[1]);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    mesh.name = `detected-${i}`;
    scene.add(mesh);
    sceneObjects.push(mesh);
    objMeta.push({
      worldPos: [wxyz[0], wxyz[1], 228],
      color: obj.color,
      shape: 'cylinder',
      label: obj.label || obj.color,
      mesh: mesh,
    });
  });
}

function moveWorkpieceAfterPick(args) {
  // Move or hide a workpiece after pick/place operations.
  // args: { tool, x, y, z, target_x?, target_y?, target_z? }
  const pickX = args.x, pickY = args.y, pickZ = args.z;

  // Find closest scene object to pick position (within threshold)
  let bestIdx = -1, bestDist = Infinity;
  for (let i = 0; i < sceneObjects.length; i++) {
    const obj = sceneObjects[i];
    const dx = obj.position.x - pickX;
    const dz = obj.position.z - pickY;
    const dist = Math.sqrt(dx * dx + dz * dz);
    if (dist < bestDist) { bestDist = dist; bestIdx = i; }
  }

  if (bestIdx < 0 || bestDist >= 60) return;

  const mesh = sceneObjects[bestIdx];
  const meta = objMeta.find(m => m.mesh === mesh);

  if (args.tool === 'pick_object') {
    // Object picked up — hide it (in gripper)
    mesh.visible = false;
    if (meta) meta.worldPos = null;
    const coordInfo = document.getElementById('coord-info');
    if (coordInfo && meta) {
      coordInfo.innerHTML = `${meta.color || ''}工件已抓取 → 在夹爪中`;
    }
    return;
  }

  if (args.tool === 'pick_and_place') {
    const targetX = args.target_x ?? pickX;
    const targetY = args.target_y ?? pickY;
    const targetZ = args.target_z ?? pickZ;
    const newThreeX = targetX;
    const newThreeY = targetZ;
    const newThreeZ = targetY;

    mesh.visible = true;
    const startPos = mesh.position.clone();
    const endPos = new THREE.Vector3(newThreeX, newThreeY, newThreeZ);
    const startTime = performance.now();
    const duration = 800;

    function animMove(time) {
      const t = Math.min((time - startTime) / duration, 1.0);
      const ease = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
      mesh.position.lerpVectors(startPos, endPos, ease);
      if (t < 1.0) {
        requestAnimationFrame(animMove);
      } else {
        mesh.position.copy(endPos);
      }
    }
    requestAnimationFrame(animMove);

    if (meta) meta.worldPos = [targetX, targetY, targetZ];
    const coordInfo = document.getElementById('coord-info');
    if (coordInfo && meta) {
      coordInfo.innerHTML = `${meta.color || ''}工件已移动 → 世界坐标 (${targetX.toFixed(0)}, ${targetY.toFixed(0)}, ${targetZ.toFixed(0)})`;
    }
  }
}

// ══════════════════════════════════════════════════════════════
// FANUC LR Mate 200iD — REAL STL MESH MODEL (URDF hierarchy)
// ══════════════════════════════════════════════════════════════
// URDF convention: Z-up (CAD standard). Three.js uses Y-up.
// robotAssembly wrapper converts Z-up → Y-up via -π/2 X rotation.
// Mesh units: meters → scale ×1000 to mm.
//
// Joint origins (from URDF, in mm, URDF Z-up coords):
//   J1: parent=Base,  origin(0, 0, 42.741),    axis(0,0,1)    rot about Z
//   J2: parent=J1,    origin(50, 0, 287.26),    axis(0,-1,0)   rot about -Y
//   J3: parent=J2,    origin(0, 0, 330),        axis(0,-1,0)   rot about -Y
//   J4: parent=J3,    origin(88, 0, 35),        axis(-1,0,0)   rot about -X
//   J5: parent=J4,    origin(245, 0, 0),        axis(0,-1,0)   rot about -Y
//   J6: parent=J5,    origin(50, 0, 0), rpy(π/2,0,0), axis(-1,0,0) rot about -X

const robotGroup = new THREE.Group();
robotGroup.position.set(-90, 0, 0);
scene.add(robotGroup);

// ── Industrial Pedestal (Y-up space, outside robotAssembly) ──
const pedestalHeight = 340;
const pedestalGeo = new THREE.CylinderGeometry(90, 110, pedestalHeight, 8);
const pedestal = new THREE.Mesh(pedestalGeo, new THREE.MeshStandardMaterial({
  color: 0x3a4048, roughness: 0.2, metalness: 0.9
}));
pedestal.position.y = pedestalHeight / 2;
pedestal.castShadow = true; pedestal.receiveShadow = true;
robotGroup.add(pedestal);

// Base mounting plate (at ground level, bolted to floor)
const mountPlateGeo = new THREE.CylinderGeometry(120, 120, 12, 32);
const mountPlate = new THREE.Mesh(mountPlateGeo, new THREE.MeshStandardMaterial({
  color: 0x2a2d35, roughness: 0.15, metalness: 0.95
}));
mountPlate.position.y = 6;
mountPlate.castShadow = true; mountPlate.receiveShadow = true;
robotGroup.add(mountPlate);

// Top flange (where robot base mounts)
const topFlangeGeo = new THREE.CylinderGeometry(85, 95, 15, 32);
const topFlange = new THREE.Mesh(topFlangeGeo, new THREE.MeshStandardMaterial({
  color: 0x4a5058, roughness: 0.12, metalness: 0.95
}));
topFlange.position.y = pedestalHeight - 7;
topFlange.castShadow = true;
robotGroup.add(topFlange);

// ── Robot Assembly wrapper: Z-up → Y-up conversion ──
const robotAssembly = new THREE.Group();
robotAssembly.rotation.set(-Math.PI / 2, 0, 0);
robotAssembly.position.y = pedestalHeight;
robotGroup.add(robotAssembly);

// ── URDF Joint pivots (inside robotAssembly, in URDF Z-up coords) ──
const jointPivots = [];
const linkMeshes = [];
const jointAxes = [
  new THREE.Vector3(0, 0, 1),    // J1
  new THREE.Vector3(0, -1, 0),   // J2
  new THREE.Vector3(0, -1, 0),   // J3
  new THREE.Vector3(-1, 0, 0),   // J4
  new THREE.Vector3(0, -1, 0),   // J5
  new THREE.Vector3(-1, 0, 0),   // J6
];

const jointOrigins = [
  [0, 0, 42.741],
  [50, 0, 287.26],
  [0, 0, 330],
  [88.001, 0, 35.027],
  [245.4, 0, 0],
  [50, 0, 0],
];

// TCP & gripper (attached after J6)
let gripperGroup, leftFinger, rightFinger, tcpDot, tcpAxesGroup;
const MESH_SCALE = 1000; // meters → mm

// ══════════════════════════════════════════════════════════════
// Build URDF robot hierarchy from STL meshes
// ══════════════════════════════════════════════════════════════

const stlLoader = new STLLoader();
let robotReady = false;
const robotReadyCallbacks = [];

function onRobotReady() {
  robotReady = true;
  robotReadyCallbacks.forEach(fn => fn());
}

// Convex STL meshes (clean, correct dimensions).
// j6.stl is CORRUPTED (2.75m span) → procedural geometry instead.
const meshFiles = [
  'models/convex/base.stl',
  'models/convex/j1.stl',
  'models/convex/j2.stl',
  'models/convex/j3.stl',
  'models/convex/j4.stl',
  'models/convex/j5.stl',
  null, // j6: procedural
];

let meshesLoaded = 0;
const loadedGeometries = new Array(7).fill(null);

// Procedural fallback geometry for each link when STL fails to load
function createProceduralLink(idx, group, material) {
  let geo;
  switch (idx) {
    case 0: // Base - wide short cylinder
      geo = new THREE.CylinderGeometry(80, 90, 30, 24);
      break;
    case 1: // J1 body - tall cylinder
      geo = new THREE.CylinderGeometry(60, 70, 120, 24);
      break;
    case 2: // J2 upper arm - elongated box
      geo = new THREE.BoxGeometry(50, 200, 50);
      break;
    case 3: // J3 forearm - box
      geo = new THREE.BoxGeometry(45, 180, 45);
      break;
    case 4: // J4 wrist - short cylinder
      geo = new THREE.CylinderGeometry(35, 40, 60, 16);
      break;
    case 5: // J5 wrist - short cylinder
      geo = new THREE.CylinderGeometry(30, 35, 50, 16);
      break;
    default:
      return;
  }
  const mesh = new THREE.Mesh(geo, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  group.add(mesh);
}

meshFiles.forEach((file, idx) => {
  if (!file) {
    meshesLoaded++;
    if (meshesLoaded === 7) buildRobotHierarchy();
    return;
  }
  stlLoader.load(file, (geometry) => {
    geometry.scale(MESH_SCALE, MESH_SCALE, MESH_SCALE);
    geometry.computeVertexNormals();
    loadedGeometries[idx] = geometry;
    meshesLoaded++;
    if (meshesLoaded === 7) buildRobotHierarchy();
  }, undefined, (err) => {
    console.warn(`Failed to load ${file}:`, err);
    // loadedGeometries[idx] stays null — procedural fallback created in buildRobotHierarchy
    meshesLoaded++;
    if (meshesLoaded === 7) buildRobotHierarchy();
  });
});

function createProceduralJ6(group, material) {
  // Flange disc
  const flangeGeo = new THREE.CylinderGeometry(22, 22, 8, 32);
  const flange = new THREE.Mesh(flangeGeo, material);
  flange.position.z = 4;
  flange.castShadow = true;
  group.add(flange);

  // Wrist connector ring
  const ringGeo = new THREE.TorusGeometry(18, 3, 8, 32);
  const ring = new THREE.Mesh(ringGeo, material);
  ring.position.z = -2;
  ring.castShadow = true;
  group.add(ring);

  // Bolt holes on flange face
  for (let i = 0; i < 6; i++) {
    const angle = (i / 6) * Math.PI * 2;
    const boltGeo = new THREE.CylinderGeometry(2, 2, 6, 8);
    const bolt = new THREE.Mesh(boltGeo, new THREE.MeshStandardMaterial({
      color: 0x888888, roughness: 0.1, metalness: 0.95
    }));
    bolt.position.set(Math.cos(angle) * 16, Math.sin(angle) * 16, 8);
    group.add(bolt);
  }
}

function buildRobotHierarchy() {
  const linkGroups = [];
  const fanucYellow = new THREE.MeshStandardMaterial({
    color: 0xf5a623, roughness: 0.25, metalness: 0.7
  });
  const darkMetal = new THREE.MeshStandardMaterial({
    color: 0x3a3d42, roughness: 0.2, metalness: 0.85
  });
  const flangeMetal = new THREE.MeshStandardMaterial({
    color: 0x555555, roughness: 0.15, metalness: 0.9
  });

  for (let i = 0; i < 7; i++) {
    const group = new THREE.Group();
    if (loadedGeometries[i]) {
      const mat = (i === 3 || i === 5) ? darkMetal : fanucYellow;
      const mesh = new THREE.Mesh(loadedGeometries[i], mat);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
    } else if (i === 6) {
      createProceduralJ6(group, flangeMetal);
    } else {
      // Procedural fallback for failed STL loads
      const mat = (i === 3 || i === 5) ? darkMetal : fanucYellow;
      createProceduralLink(i, group, mat);
    }
    linkGroups.push(group);
  }

  // Build nested hierarchy inside robotAssembly (URDF Z-up coords)
  // robotAssembly → baseGroup (fixed)
  // baseGroup → j1Pivot → j1Group → j2Pivot → j2Group → ...
  robotAssembly.add(linkGroups[0]); // Base mesh, fixed

  const j1Pivot = new THREE.Group();
  j1Pivot.position.set(0, 0, 42.741);
  robotAssembly.add(j1Pivot);
  jointPivots.push(j1Pivot);
  j1Pivot.add(linkGroups[1]);

  const j2Pivot = new THREE.Group();
  j2Pivot.position.set(50, 0, 287.26);
  j1Pivot.add(j2Pivot);
  jointPivots.push(j2Pivot);
  j2Pivot.add(linkGroups[2]);

  const j3Pivot = new THREE.Group();
  j3Pivot.position.set(0, 0, 330);
  j2Pivot.add(j3Pivot);
  jointPivots.push(j3Pivot);
  j3Pivot.add(linkGroups[3]);

  const j4Pivot = new THREE.Group();
  j4Pivot.position.set(88.001, 0, 35.027);
  j3Pivot.add(j4Pivot);
  jointPivots.push(j4Pivot);
  j4Pivot.add(linkGroups[4]);

  const j5Pivot = new THREE.Group();
  j5Pivot.position.set(245.4, 0, 0);
  j4Pivot.add(j5Pivot);
  jointPivots.push(j5Pivot);
  j5Pivot.add(linkGroups[5]);

  const j6Pivot = new THREE.Group();
  j6Pivot.position.set(50, 0, 0);
  j6Pivot.rotation.set(Math.PI / 2, 0, 0);
  j5Pivot.add(j6Pivot);
  jointPivots.push(j6Pivot);
  j6Pivot.add(linkGroups[6]);

  // ── Gripper (attached after J6, in URDF Z-up space) ──
  gripperGroup = new THREE.Group();

  const palmGeo = new THREE.BoxGeometry(22, 10, 16);
  const palmMat = new THREE.MeshStandardMaterial({
    color: 0x555555, roughness: 0.12, metalness: 0.9
  });
  const palm = new THREE.Mesh(palmGeo, palmMat);
  palm.position.z = 40;
  gripperGroup.add(palm);

  const fingerGeo = new THREE.BoxGeometry(5, 38, 7);
  const fingerMat = new THREE.MeshStandardMaterial({
    color: 0xcccccc, roughness: 0.2, metalness: 0.7
  });
  leftFinger = new THREE.Mesh(fingerGeo, fingerMat);
  leftFinger.position.set(-8, 0, 64);
  leftFinger.name = 'leftFinger';
  gripperGroup.add(leftFinger);

  rightFinger = new THREE.Mesh(fingerGeo, fingerMat);
  rightFinger.position.set(8, 0, 64);
  rightFinger.name = 'rightFinger';
  gripperGroup.add(rightFinger);

  for (const f of [leftFinger, rightFinger]) {
    const padGeo = new THREE.BoxGeometry(3, 15, 5);
    const pad = new THREE.Mesh(padGeo, new THREE.MeshStandardMaterial({
      color: 0x111111, roughness: 0.9, metalness: 0.0
    }));
    pad.position.set(0, -10, 0);
    f.add(pad);
  }

  tcpDot = new THREE.Mesh(
    new THREE.SphereGeometry(5, 16, 16),
    new THREE.MeshStandardMaterial({
      color: 0xff3333, emissive: 0xff0000, emissiveIntensity: 2.5
    })
  );
  tcpDot.position.set(0, 0, 85);
  gripperGroup.add(tcpDot);

  // TCP coordinate axes
  tcpAxesGroup = new THREE.Group();
  function createAxis(dir, color, length) {
    const arrowGeo = new THREE.CylinderGeometry(1.5, 1.5, length, 8);
    const arrow = new THREE.Mesh(arrowGeo, new THREE.MeshStandardMaterial({
      color, emissive: color, emissiveIntensity: 0.6, roughness: 0.2
    }));
    if (Math.abs(dir.x) > 0.5) arrow.rotation.z = -Math.PI / 2 * Math.sign(dir.x);
    if (Math.abs(dir.z) > 0.5) arrow.rotation.x = Math.PI / 2 * Math.sign(dir.z);
    arrow.position.copy(dir.clone().multiplyScalar(length / 2));
    return arrow;
  }
  tcpAxesGroup.add(createAxis(new THREE.Vector3(1, 0, 0), 0xff3333, 50));
  tcpAxesGroup.add(createAxis(new THREE.Vector3(0, 1, 0), 0x33ff33, 50));
  tcpAxesGroup.add(createAxis(new THREE.Vector3(0, 0, 1), 0x3366ff, 50));
  gripperGroup.add(tcpAxesGroup);

  j6Pivot.add(gripperGroup);

  linkMeshes.push(...linkGroups);

  onRobotReady();
}

// ── Trajectory path line ──
const trajectoryLineMat = new THREE.LineBasicMaterial({
  color: 0x00ffcc, transparent: true, opacity: 0.7, linewidth: 1
});
let trajectoryLine = new THREE.Line(new THREE.BufferGeometry(), trajectoryLineMat);
trajectoryLine.visible = false;
scene.add(trajectoryLine);

function showTrajectoryPath(points) {
  if (!points || points.length < 2) {
    trajectoryLine.visible = false;
    return;
  }
  const pts3 = points.map(p => {
    if (Array.isArray(p)) return new THREE.Vector3(p[0], p[1], p[2]);
    return new THREE.Vector3(p.x || 0, p.y || 0, p.z || 0);
  });
  trajectoryLine.geometry.dispose();
  trajectoryLine.geometry = new THREE.BufferGeometry().setFromPoints(pts3);
  trajectoryLine.visible = true;
  // Auto-hide after 5s
  clearTimeout(trajectoryLine._hideTimeout);
  trajectoryLine._hideTimeout = setTimeout(() => { trajectoryLine.visible = false; }, 5000);
}

// ══════════════════════════════════════════════════════════════
// URDF KINEMATICS (nested pivot hierarchy)
// ══════════════════════════════════════════════════════════════
// Joint origins are fixed (set during buildRobotHierarchy).
// applyJointsToRobot only rotates each pivot about its joint axis.
// TCP world position is computed by traversing the nested hierarchy.

let tcpWorldPos = new THREE.Vector3();

// Store current joint angles for animation
let currentJoints = [0, 0, 0, 0, 0, 0];
let targetJoints = [0, 0, 0, 0, 0, 0];
let gripperOpen = 0; // 0=closed, 1=open

function updateRobotJoints(joints, instant = true) {
  if (!joints || joints.length < 6) return;
  if (!robotReady) {
    robotReadyCallbacks.push(() => updateRobotJoints(joints, instant));
    return;
  }
  targetJoints = [...joints];
  if (instant) {
    currentJoints = [...joints];
    applyJointsToRobot(currentJoints);
  }
}

function applyJointsToRobot(joints) {
  // URDF nested hierarchy: each pivot is already at the correct position
  // relative to its parent (set during buildRobotHierarchy).
  // We only apply rotation about the joint axis — children inherit automatically.
  for (let i = 0; i < 6; i++) {
    const jointQuat = new THREE.Quaternion().setFromAxisAngle(jointAxes[i], joints[i]);
    if (i === 5) {
      // J6 has fixed rpy(π/2,0,0) from URDF — combine with joint rotation
      const rpyQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(Math.PI / 2, 0, 0));
      jointPivots[i].quaternion.copy(jointQuat).multiply(rpyQuat);
    } else {
      jointPivots[i].quaternion.copy(jointQuat);
    }
  }

  // Compute TCP world position by traversing the nested hierarchy
  const tcpLocal = new THREE.Vector3(0, 0, 85);
  if (gripperGroup) {
    gripperGroup.localToWorld(tcpLocal);
  }
  tcpWorldPos.copy(tcpLocal);

  // Update gripper fingers (X-axis opening, Z-axis aligned fingers)
  const gripOffset = gripperOpen * 5;
  if (leftFinger) leftFinger.position.x = -8 - gripOffset;
  if (rightFinger) rightFinger.position.x = 8 + gripOffset;

  updateJointPanel(joints);
  updateTCPPanel();
}

// ══════════════════════════════════════════════════════════════
// PANEL UPDATES
// ══════════════════════════════════════════════════════════════

function updateJointPanel(joints) {
  const degs = joints.map(j => (j * 180 / Math.PI).toFixed(1));
  const rads = joints.map(j => j.toFixed(3));
  for (let i = 0; i < 6; i++) {
    const jd = document.getElementById(`j${i + 1}-deg`);
    const jr = document.getElementById(`j${i + 1}-rad`);
    if (jd) jd.textContent = degs[i];
    if (jr) jr.textContent = rads[i];
  }
  const stateDisplay = document.getElementById('robot-state-display');
  if (stateDisplay) {
    stateDisplay.textContent = `关节: [${degs.map(d => d + '°').join(', ')}]`;
  }
}

function updateTCPPanel() {
  const pos = tcpWorldPos;
  const quat = new THREE.Quaternion();
  if (gripperGroup) {
    gripperGroup.getWorldQuaternion(quat);
  }
  const euler = new THREE.Euler().setFromQuaternion(quat, 'XYZ');

  const setEl = (id, val) => { const el = document.getElementById(id); if (el) el.textContent = val; };
  setEl('tcp-x', pos.x.toFixed(1));
  setEl('tcp-y', pos.y.toFixed(1));
  setEl('tcp-z', pos.z.toFixed(1));
  setEl('tcp-rx', (euler.x * 180 / Math.PI).toFixed(1));
  setEl('tcp-ry', (euler.y * 180 / Math.PI).toFixed(1));
  setEl('tcp-rz', (euler.z * 180 / Math.PI).toFixed(1));
  setEl('tcp-gripper', gripperOpen > 0.5 ? '张开' : '闭合');
}

function updatePipelineStatus(msg) {
  const el = document.getElementById("pipeline-status");
  if (el) el.textContent = msg;
}

// ══════════════════════════════════════════════════════════════
// PICK & PLACE ANIMATION
// ══════════════════════════════════════════════════════════════

let animFrameId = null;
let animationQueue = [];
let isAnimating = false;

function animateToJoints(targetJ, duration = 800) {
  return new Promise(resolve => {
    const startJoints = [...currentJoints];
    const startTime = performance.now();

    function animStep(time) {
      const elapsed = time - startTime;
      const t = Math.min(elapsed / duration, 1.0);
      // Ease in-out
      const ease = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
      const interp = startJoints.map((s, i) => s + (targetJ[i] - s) * ease);
      applyJointsToRobot(interp);
      currentJoints = interp;

      if (t < 1.0) {
        animFrameId = requestAnimationFrame(animStep);
      } else {
        applyJointsToRobot(targetJ);
        currentJoints = [...targetJ];
        resolve();
      }
    }
    animFrameId = requestAnimationFrame(animStep);
  });
}

async function animateGripper(open, duration = 400) {
  const startOpen = gripperOpen;
  const startTime = performance.now();
  return new Promise(resolve => {
    function step(time) {
      const t = Math.min((time - startTime) / duration, 1.0);
      gripperOpen = startOpen + (open - startOpen) * t;
      leftFinger.position.x = -8 - gripperOpen * 5;
      rightFinger.position.x = 8 + gripperOpen * 5;
      const gripEl = document.getElementById('tcp-gripper');
      if (gripEl) gripEl.textContent = gripperOpen > 0.5 ? '张开' : '闭合';
      if (t < 1.0) {
        animFrameId = requestAnimationFrame(step);
      } else {
        gripperOpen = open;
        leftFinger.position.x = -8 - open * 5;
        rightFinger.position.x = 8 + open * 5;
        resolve();
      }
    }
    animFrameId = requestAnimationFrame(step);
  });
}

async function executePickAndPlaceAnimation(pickXYZ, placeXYZ) {
  if (isAnimating) return;
  isAnimating = true;
  updatePipelineStatus('执行中...');

  try {
    // Get robot status for current joints
    const resp = await fetch(`${ROBOT_URL}/status`);
    const status = await resp.json();
    const startJoints = status.joints || currentJoints;

    // Phase 1: Move to pre-grasp (above object +80mm)
    updatePipelineStatus('IK求解中...');
    const preGraspRes = await fetch(`${ROBOT_URL}/execute`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        action: 'move',
        target: { x: pickXYZ[0], y: pickXYZ[1], z: pickXYZ[2] + 80 }
      })
    });
    const preGraspData = await preGraspRes.json();
    if (preGraspData.sequence && preGraspData.sequence[0]) {
      const traj = preGraspData.sequence[0].trajectory;
      for (let i = 0; i < traj.length; i += 3) {
        await animateToJoints(traj[i].joints, 150);
      }
      await animateToJoints(traj[traj.length - 1].joints, 300);
    }

    // Phase 2: Descend to grasp
    const graspRes = await fetch(`${ROBOT_URL}/execute`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        action: 'move',
        target: { x: pickXYZ[0], y: pickXYZ[1], z: pickXYZ[2] - 5 }
      })
    });
    const graspData = await graspRes.json();
    if (graspData.sequence && graspData.sequence[0]) {
      const traj = graspData.sequence[0].trajectory;
      for (let i = 0; i < traj.length; i += 3) {
        await animateToJoints(traj[i].joints, 100);
      }
      await animateToJoints(traj[traj.length - 1].joints, 200);
    }

    // Phase 3: Close gripper
    updatePipelineStatus('执行中...');
    await animateGripper(0, 300);

    // Phase 4: Lift
    const liftRes = await fetch(`${ROBOT_URL}/execute`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        action: 'move',
        target: { x: pickXYZ[0], y: pickXYZ[1], z: pickXYZ[2] + 80 }
      })
    });
    const liftData = await liftRes.json();
    if (liftData.sequence && liftData.sequence[0]) {
      const traj = liftData.sequence[0].trajectory;
      for (let i = 0; i < traj.length; i += 3) {
        await animateToJoints(traj[i].joints, 150);
      }
      await animateToJoints(traj[traj.length - 1].joints, 300);
    }

    // Phase 5: Move to place target
    const placePreRes = await fetch(`${ROBOT_URL}/execute`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        action: 'move',
        target: { x: placeXYZ[0], y: placeXYZ[1], z: placeXYZ[2] + 50 }
      })
    });
    const placePreData = await placePreRes.json();
    if (placePreData.sequence && placePreData.sequence[0]) {
      const traj = placePreData.sequence[0].trajectory;
      for (let i = 0; i < traj.length; i += 3) {
        await animateToJoints(traj[i].joints, 200);
      }
      await animateToJoints(traj[traj.length - 1].joints, 400);
    }

    // Phase 6: Descend to place
    const placeRes = await fetch(`${ROBOT_URL}/execute`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        action: 'move',
        target: { x: placeXYZ[0], y: placeXYZ[1], z: placeXYZ[2] }
      })
    });
    const placeData = await placeRes.json();
    if (placeData.sequence && placeData.sequence[0]) {
      const traj = placeData.sequence[0].trajectory;
      for (let i = 0; i < traj.length; i += 3) {
        await animateToJoints(traj[i].joints, 100);
      }
      await animateToJoints(traj[traj.length - 1].joints, 200);
    }

    // Phase 7: Open gripper
    await animateGripper(1, 300);

    // Done
    updatePipelineStatus('完成');

  } catch (err) {
    console.error('Animation error:', err);
    updatePipelineStatus('执行失败');
  } finally {
    isAnimating = false;
  }
}

// Also support trajectory animation from SSE events
function animateTrajectorySeq(trajectory) {
  if (isAnimating || !trajectory || trajectory.length === 0) return Promise.resolve();

  return new Promise(resolve => {
    isAnimating = true;
    let step = 0;
    const fps = 20;
    const interval = 1000 / fps;
    let lastTime = 0;

    function anim(time) {
      if (!lastTime) lastTime = time;
      if (time - lastTime >= interval) {
        if (step < trajectory.length) {
          updateRobotJoints(trajectory[step].joints, true);
          const progressFill = document.getElementById('progress-fill');
          if (progressFill) progressFill.style.width = (step / trajectory.length * 100) + '%';
          step++;
          lastTime = time;
        } else {
          const progressFill = document.getElementById('progress-fill');
          if (progressFill) { progressFill.style.width = '100%';
            setTimeout(() => { progressFill.style.width = '0%'; }, 1000); }
          isAnimating = false;
          resolve();
          return;
        }
      }
      animFrameId = requestAnimationFrame(anim);
    }
    animFrameId = requestAnimationFrame(anim);
  });
}

// ══════════════════════════════════════════════════════════════
// OFFLINE CAMERA (Virtual Industrial Camera for YOLO)
// ══════════════════════════════════════════════════════════════

const offscreenCanvas = document.createElement('canvas');
offscreenCanvas.width = 640;
offscreenCanvas.height = 480;
const offscreenRenderer = new THREE.WebGLRenderer({ canvas: offscreenCanvas, antialias: true, alpha: false });
offscreenRenderer.shadowMap.enabled = true;
offscreenRenderer.toneMapping = THREE.ACESFilmicToneMapping;
offscreenRenderer.toneMappingExposure = 1.0;
offscreenRenderer.outputColorSpace = THREE.SRGBColorSpace;

const virtualCamera = new THREE.PerspectiveCamera(55, 640 / 480, 10, 3000);
virtualCamera.position.set(350, 520, 0);
virtualCamera.lookAt(350, 210, 0);

function captureVirtualFrame() {
  // Temporarily hide fence for camera view
  fenceGroup.visible = false;
  statusTower.visible = false;
  cabinet.visible = false;
  offscreenRenderer.render(scene, virtualCamera);
  fenceGroup.visible = true;
  statusTower.visible = true;
  cabinet.visible = true;
  return offscreenCanvas.toDataURL('image/jpeg', 0.85).split(',')[1];
}

// Expose for tools
window.captureVirtualFrame = captureVirtualFrame;

// ══════════════════════════════════════════════════════════════
// SCENE CONTROLS
// ══════════════════════════════════════════════════════════════

function resetView() {
  camera.position.set(800, 700, 900);
  if (controls) {
    controls.target.set(300, 400, 0);
    controls.update();
  }
}

let gridVisible = true;
function toggleGrid() {
  gridVisible = !gridVisible;
  gridHelper.visible = gridVisible;
}

async function resetRobot() {
  try {
    await fetch(`${ROBOT_URL}/reset`, { method: 'POST' });
    updateRobotJoints([0, 0, 0, 0, 0, 0], true);
    const gripEl = document.getElementById('tcp-gripper');
    if (gripEl) gripEl.textContent = '松开';
  } catch (e) { console.warn('Robot reset failed:', e); }
}

async function readyRobot() {
  try {
    const resp = await fetch(`${ROBOT_URL}/ready`, { method: 'POST' });
    const data = await resp.json();
    if (data.joints) updateRobotJoints(data.joints, true);
    const gripEl = document.getElementById('tcp-gripper');
    if (gripEl) gripEl.textContent = '松开';
  } catch (e) { console.warn('Robot ready failed:', e); }
}

// ══════════════════════════════════════════════════════════════
// RENDER LOOP
// ══════════════════════════════════════════════════════════════

let frameCount = 0;
let lastFpsTime = performance.now();
let currentFps = 0;

function renderLoop() {
  requestAnimationFrame(renderLoop);
  if (controls) controls.update();
  renderer.render(scene, camera);

  // FPS tracking
  frameCount++;
  const now = performance.now();
  if (now - lastFpsTime >= 1000) {
    currentFps = Math.round(frameCount * 1000 / (now - lastFpsTime));
    frameCount = 0;
    lastFpsTime = now;
  }

  // Animate status lights
  const time = Date.now() * 0.001;
  const greenLight = scene.getObjectByName('status-light-green');
  if (greenLight) {
    greenLight.material.emissiveIntensity = 0.3 + Math.sin(time * 2) * 0.3;
  }
}
renderLoop();

function onResize() {
  const rect = container.getBoundingClientRect();
  if (rect.width > 0 && rect.height > 0) {
    camera.aspect = rect.width / rect.height;
    camera.updateProjectionMatrix();
    renderer.setSize(rect.width, rect.height);
  }
}
window.addEventListener('resize', onResize);
setTimeout(onResize, 100);
setTimeout(onResize, 500);

// Initial pose
updateRobotJoints([0, 0, 0, 0, 0, 0], true);
setTimeout(async () => {
  try {
    const resp = await fetch(`${ROBOT_URL}/ready`);
    const data = await resp.json();
    if (data.joints) updateRobotJoints(data.joints, true);
  } catch (e) {}
}, 300);

// ══════════════════════════════════════════════════════════════
// SCENE HEALTH MONITOR
// ══════════════════════════════════════════════════════════════

const healthOverlay = document.createElement('div');
healthOverlay.id = 'scene-health';
healthOverlay.style.cssText = [
  'position: absolute',
  'bottom: 10px',
  'left: 10px',
  'background: rgba(0,0,0,0.75)',
  'color: #aab',
  'font-family: monospace',
  'font-size: 11px',
  'padding: 8px 12px',
  'border-radius: 4px',
  'z-index: 10',
  'line-height: 1.6',
  'pointer-events: none',
  'white-space: pre'
].join(';');
if (container) container.appendChild(healthOverlay);

function updateSceneHealth() {
  const lights = scene.children.filter(function(c) { return c.isLight; }).length;
  const meshes = scene.children.filter(function(c) { return c.isMesh; }).length;
  const pos = camera.position;
  const warnings = [];

  if (lights === 0) warnings.push('[!] 无光源');
  if (Math.abs(pos.x) < 1 && Math.abs(pos.y) < 1 && Math.abs(pos.z) < 1) warnings.push('[!] 相机位于原点');
  if (!robotReady) warnings.push('[!] 机器人未加载');

  // Update #status-scene indicator
  const statusScene = document.getElementById('status-scene');
  if (statusScene) {
    if (lights === 0) {
      statusScene.className = 'status-dot error';
    } else if (warnings.length > 0) {
      statusScene.className = 'status-dot warning';
    } else {
      statusScene.className = 'status-dot connected';
    }
  }

  healthOverlay.textContent = [
    '相机: (' + pos.x.toFixed(0) + ', ' + pos.y.toFixed(0) + ', ' + pos.z.toFixed(0) + ')',
    '光源: ' + lights + ' | 网格: ' + meshes,
    'FPS: ' + currentFps + ' | 机器人: ' + (robotReady ? '就绪' : '加载中...'),
    warnings.length > 0 ? warnings.join('\n') : ''
  ].join('\n');
}

setInterval(updateSceneHealth, 2000);

// ══════════════════════════════════════════════════════════════
// MODE SWITCHING (agent | jog | program)
// ══════════════════════════════════════════════════════════════

function switchMode(mode) {
  // Show/hide mode panels
  var panels = ['agent', 'jog', 'program'];
  panels.forEach(function(m) {
    var panel = document.getElementById('mode-' + m + '-panel');
    if (panel) panel.style.display = (m === mode) ? '' : 'none';
    var tab = document.getElementById('tab-' + m);
    if (tab) tab.classList.toggle('active', m === mode);
  });

  // Update mode title
  var titleEl = document.getElementById('mode-title');
  if (titleEl) {
    var names = { agent: '智能操控', jog: '手动操控', program: '程序管理' };
    titleEl.textContent = names[mode] || mode;
  }
}

// Default mode: agent
switchMode('agent');

// ══════════════════════════════════════════════════════════════
// CHAT & SSE AGENT INTERFACE
// ══════════════════════════════════════════════════════════════

const chatLog = document.getElementById('chat-log');
const userInput = document.getElementById('user-input');
const sendBtn = document.getElementById('send-btn');
const stopBtn = document.getElementById('stop-btn');
const progressBar = document.getElementById('progress-bar');
const visionCanvas = document.getElementById('vision-canvas');
const visionPlaceholder = document.getElementById('vision-placeholder');

let toolCallCount = 0;
let pendingPickArgs = null; // { tool, x, y, z, target_x, target_y, target_z }

// ══════════════════════════════════════════════════════════════
// JOG & TEACH STATE
// ══════════════════════════════════════════════════════════════

let jogMode = 'joint'; // joint | cart | teach
let jogStepDeg = 1;    // degrees for joint jog
let jogStepMM = 10;    // mm for cartesian jog
let jogSpeed = 30;     // percent
let teachPoints = [];  // { name, joints, tcp, timestamp }
let programActions = []; // recorded agent actions for TP generation

function escapeHtml(str) {
  const div = document.createElement('div');
  div.textContent = String(str);
  return div.innerHTML;
}

function displayVisionImage(b64) {
  if (!b64) return;
  const img = new Image();
  img.onload = () => {
    const ctx = visionCanvas.getContext('2d');
    visionCanvas.width = img.width;
    visionCanvas.height = img.height;
    ctx.drawImage(img, 0, 0);
    visionCanvas.style.display = 'block';
    if (visionPlaceholder) visionPlaceholder.style.display = 'none';
  };
  img.src = 'data:image/jpeg;base64,' + b64;
}

function appendMessage(type, content, meta) {
  const div = document.createElement('div');
  div.className = 'message ' + type;
  let html = '<div class="msg-content">';

  if (type === 'user') {
    html += `<p>${escapeHtml(String(content))}</p>`;
  } else if (type === 'tool_call') {
    const toolName = meta && meta.name ? meta.name : '';
    const toolArgs = meta && meta.args ? JSON.stringify(meta.args) : '';
    html += `<strong>调用工具: ${escapeHtml(toolName)}</strong>`;
    if (toolArgs) html += `<pre>${escapeHtml(toolArgs)}</pre>`;

    toolCallCount++;
    // Record action for program generation
    programActions.push({
      type: 'tool_call',
      tool: toolName,
      args: meta && meta.args ? meta.args : {},
      timestamp: Date.now(),
    });

    // Store pending pick args for workpiece movement on success
    if (toolName === 'pick_and_place' || toolName === 'pick_object' || toolName === 'place_object') {
      pendingPickArgs = { tool: toolName, ...((meta && meta.args) || {}) };
    }

    // Update pipeline
    if (toolName === 'analyze_scene') updatePipelineStatus('YOLO检测中...');
    else if (toolName === 'get_robot_status') updatePipelineStatus('LLM解析中...');
    else if (toolName === 'pick_and_place' || toolName === 'pick_object') updatePipelineStatus('目标定位中...');

  } else if (type === 'tool_result') {
    html += `<strong>执行结果</strong>`;
    const contentStr = typeof content === 'string' ? content : JSON.stringify(content, null, 2);
    html += `<pre>${escapeHtml(contentStr.substring(0, 1500))}</pre>`;

    // Record result for program generation and handle vision results
    try {
      const result = typeof content === 'string' ? JSON.parse(content) : content;
      if (result && programActions.length > 0) {
        const lastAction = programActions[programActions.length - 1];
        if (lastAction.type === 'tool_call') {
          lastAction.result = result;
          lastAction.success = result.success !== false;
        }
      }
      if (result && result.annotated_b64) {
        displayVisionImage(result.annotated_b64);
      }
      if (result && result.objects) {
        updateSceneObjects(result.objects);
        updatePipelineStatus('检测完成');
      }
    } catch (e) {
      // JSON parse errors are non-fatal
    }

  } else if (type === 'thinking') {
    html += `<strong>思考</strong><p>${escapeHtml(content)}</p>`;
    updatePipelineStatus('LLM解析中...');
  } else if (type === 'final') {
    html += `<strong>回答</strong><p>${escapeHtml(content)}</p>`;
    updatePipelineStatus('完成');
    // Auto-generate program on task completion
    if (programActions.length > 0) setTimeout(generateProgram, 300);
  } else if (type === 'error') {
    html += `<strong>错误</strong><p>${escapeHtml(content)}</p>`;
  } else if (type === 'system') {
    html += content;
  }

  html += '</div>';
  div.innerHTML = html;
  chatLog.appendChild(div);
  chatLog.scrollTop = chatLog.scrollHeight;
  return div;
}

async function sendMessage() {
  const message = userInput.value.trim();
  if (!message || isExecuting) return;

  isExecuting = true;
  sendBtn.disabled = true;
  stopBtn.style.display = 'inline-block';
  progressBar.style.display = 'block';
  const progressFill = document.getElementById('progress-fill');
  if (progressFill) progressFill.style.width = '0%';
  userInput.value = '';
  toolCallCount = 0;
  programActions = [];
  pendingPickArgs = null;

  updatePipelineStatus('执行中...');

  appendMessage('user', message);

  const typingDiv = document.createElement('div');
  typingDiv.className = 'message thinking';
  typingDiv.innerHTML = '<div class="typing-indicator"><span></span><span></span><span></span></div>';
  chatLog.appendChild(typingDiv);
  chatLog.scrollTop = chatLog.scrollHeight;

  try {
    abortController = new AbortController();
    // Upload virtual camera frame to agent service
    const vframe = captureVirtualFrame();
    if (vframe) {
      try {
        await fetch(`${AGENT_URL}/capture`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ image_b64: vframe }),
          signal: abortController.signal,
        });
      } catch (e) { console.warn('Virtual frame upload failed:', e); }
    }
    const url = `${AGENT_URL}/chat/stream?message=${encodeURIComponent(message)}&image_path=data/virtual_frame.jpg`;
    const response = await fetch(url, { signal: abortController.signal });

    if (!response.ok) throw new Error(`HTTP ${response.status}`);

    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let buffer = '';

    while (true) {
      const { done, value } = await reader.read();
      if (done) break;
      buffer += decoder.decode(value, { stream: true });
      const lines = buffer.split('\n');
      buffer = lines.pop() || '';

      for (const line of lines) {
        if (line.startsWith('data: ')) {
          try {
            const event = JSON.parse(line.slice(6));
            if (typingDiv.parentNode) typingDiv.remove();
            appendMessage(event.type, event.content, event.meta);

            // Handle animation from tool results
            if (event.type === 'tool_result' && event.meta && event.meta.name) {
              try {
                const result = typeof event.content === 'string'
                  ? JSON.parse(event.content) : event.content;
                if (result.sequence) {
                  for (const phase of result.sequence) {
                    if (phase.trajectory && phase.trajectory.length > 0) {
                      await animateTrajectorySeq(phase.trajectory);
                    }
                    // Handle gripper phase
                    if (phase.phase === 'gripper') {
                      const shouldClose = phase.action === 'close';
                      gripperOpen = shouldClose ? 0 : 1;
                      applyJointsToRobot(currentJoints);
                      const gripEl = document.getElementById('tcp-gripper');
                      if (gripEl) gripEl.textContent = shouldClose ? '闭合' : '张开';
                    }
                  }
                }
                if (result.final_state && result.final_state.joints) {
                  updateRobotJoints(result.final_state.joints, true);
                  if (result.final_state.gripper !== undefined) {
                    gripperOpen = result.final_state.gripper > 0.5 ? 0 : 1;
                    applyJointsToRobot(result.final_state.joints);
                  }
                  const gripEl = document.getElementById('tcp-gripper');
                  if (gripEl) gripEl.textContent =
                    (result.final_state.gripper > 0.5) ? '闭合' : '张开';
                }

                // Move workpiece in 3D scene after successful pick-and-place
                if (result.success && pendingPickArgs && sceneObjects.length > 0) {
                  moveWorkpieceAfterPick(pendingPickArgs);
                  pendingPickArgs = null;
                }
              } catch (e) { console.warn('Tool result parse error:', e); }
            }
          } catch (e) { console.warn('SSE parse error:', e); }
        }
      }
    }

    // Refresh robot state
    try {
      const statusResp = await fetch(`${ROBOT_URL}/status`);
      const status = await statusResp.json();
      if (status.joints) updateRobotJoints(status.joints, true);
    } catch (e) {}

  } catch (err) {
    if (err.name !== 'AbortError') {
      appendMessage('error', `连接失败: ${err.message}`);
      updatePipelineStatus('连接失败');
    }
  } finally {
    if (typingDiv.parentNode) typingDiv.remove();
    isExecuting = false;
    sendBtn.disabled = false;
    stopBtn.style.display = 'none';
    progressBar.style.display = 'none';
    abortController = null;
  }
}

function stopExecution() {
  if (abortController) { abortController.abort(); abortController = null; }
  isExecuting = false;
  sendBtn.disabled = false;
  stopBtn.style.display = 'none';
  progressBar.style.display = 'none';
}

function handleKeyDown(e) {
  if (e.key === 'Enter' && !e.shiftKey) {
    e.preventDefault();
    sendMessage();
  }
}

function quickCommand(cmd) {
  userInput.value = cmd;
  sendMessage();
}

// ══════════════════════════════════════════════════════════════
// COORDINATE VISUALIZATION (click to select object)
// ══════════════════════════════════════════════════════════════

const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
let selectedHighlight = null;

function createHighlightRing() {
  const ringGeo = new THREE.TorusGeometry(22, 2, 16, 32);
  const ringMat = new THREE.MeshBasicMaterial({ color: 0xffff00, transparent: true, opacity: 0.8 });
  const ring = new THREE.Mesh(ringGeo, ringMat);
  ring.rotation.x = -Math.PI / 2;
  ring.visible = false;
  scene.add(ring);
  return ring;
}
selectedHighlight = createHighlightRing();

canvas.addEventListener('click', (e) => {
  const rect = canvas.getBoundingClientRect();
  mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
  mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;

  raycaster.setFromCamera(mouse, camera);
  const hits = raycaster.intersectObjects(sceneObjects, false);
  if (hits.length > 0) {
    const obj = hits[0].object;
    selectedHighlight.position.copy(obj.position);
    selectedHighlight.position.y += 2;
    selectedHighlight.visible = true;

    // Find metadata
    const meta = objMeta.find(m => m.mesh === obj);
    const coordInfo = document.getElementById('coord-info');
    if (coordInfo && meta) {
      const wp = meta.worldPos;
      coordInfo.innerHTML = `
        <strong>${meta.label}</strong><br>
        颜色: ${meta.color}<br>
        世界坐标: (${wp[0].toFixed(0)}, ${wp[1].toFixed(0)}, ${wp[2].toFixed(0)}) mm<br>
        相机帧 → 世界帧 → 基座帧 → TCP
      `;
    }
  } else {
    selectedHighlight.visible = false;
    const coordInfo = document.getElementById('coord-info');
    if (coordInfo) coordInfo.innerHTML = '<span class="muted">点击工件查看坐标</span>';
  }
});

// ══════════════════════════════════════════════════════════════
// SERVICE MONITORING
// ══════════════════════════════════════════════════════════════

async function checkServices() {
  const services = [
    { id: 'status-vision', url: `${VISION_URL}/health` },
    { id: 'status-robot', url: `${ROBOT_URL}/health` },
    { id: 'status-agent', url: `${AGENT_URL}/health` },
  ];
  for (const svc of services) {
    const el = document.getElementById(svc.id);
    if (!el) continue;
    try {
      const resp = await fetch(svc.url, { signal: AbortSignal.timeout(3000) });
      el.classList.toggle('connected', resp.ok);
      el.classList.toggle('disconnected', !resp.ok);
    } catch {
      el.classList.add('disconnected');
      el.classList.remove('connected');
    }
  }
}
checkServices();
setInterval(checkServices, 10000);

// ══════════════════════════════════════════════════════════════
// JOG CONTROL
// ══════════════════════════════════════════════════════════════

function setJogMode(mode) {
  jogMode = mode;
  const jm = document.getElementById('jog-mode-joint');
  const cm = document.getElementById('jog-mode-cart');
  const tm = document.getElementById('jog-mode-teach');
  if (jm) jm.classList.toggle('active', mode === 'joint');
  if (cm) cm.classList.toggle('active', mode === 'cart');
  if (tm) tm.classList.toggle('active', mode === 'teach');
  const jp = document.getElementById('jog-joint-panel');
  const cp = document.getElementById('jog-cart-panel');
  const tp = document.getElementById('jog-teach-panel');
  if (jp) jp.style.display = mode === 'joint' ? '' : 'none';
  if (cp) cp.style.display = mode === 'cart' ? '' : 'none';
  if (tp) tp.style.display = mode === 'teach' ? '' : 'none';
  const label = document.getElementById('jog-mode-label');
  if (label) label.textContent = mode === 'joint' ? '关节模式' : mode === 'cart' ? '笛卡尔模式' : '示教模式';
}

function updateJogStep() {
  const sel = document.getElementById('jog-step');
  const val = sel ? parseFloat(sel.value) : 1;
  jogStepDeg = val;
  jogStepMM = val * 10;
}

function updateJogSpeed() {
  const slider = document.getElementById('jog-speed');
  jogSpeed = slider ? parseInt(slider.value) : 30;
  const val = document.getElementById('jog-speed-val');
  if (val) val.textContent = jogSpeed + '%';
}

async function jogJoint(index, direction) {
  const deltaRad = direction * jogStepDeg * Math.PI / 180;
  const newJoints = [...currentJoints];
  newJoints[index] += deltaRad;
  try {
    const resp = await fetch(`${ROBOT_URL}/jog`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        type: 'joint', index: index,
        delta_deg: direction * jogStepDeg,
        speed: jogSpeed,
      }),
    });
    const data = await resp.json();
    await animateToJoints(data.joints || newJoints, 200);
  } catch (e) {
    await animateToJoints(newJoints, 200);
  }
  updateJogDisplay();
}

async function jogCartesian(axis, delta) {
  const deltaMM = delta * jogStepMM;
  try {
    const resp = await fetch(`${ROBOT_URL}/jog`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        type: 'cartesian', axis: axis,
        delta_mm: deltaMM, speed: jogSpeed,
      }),
    });
    const data = await resp.json();
    if (data.joints) await animateToJoints(data.joints, 300);
  } catch (e) {
    console.warn('Cartesian jog failed:', e);
  }
  updateJogDisplay();
}

function updateJogDisplay() {
  for (let i = 0; i < 6; i++) {
    const el = document.getElementById('jog-j' + (i + 1) + '-val');
    if (el) el.textContent = (currentJoints[i] * 180 / Math.PI).toFixed(1) + '°';
  }
  const tcp = tcpWorldPos;
  function cel(id, val) {
    const e = document.getElementById(id);
    if (e) e.textContent = val.toFixed(1);
  }
  cel('jog-cart-x', tcp.x);
  cel('jog-cart-y', tcp.y);
  cel('jog-cart-z', tcp.z);
}

// ══════════════════════════════════════════════════════════════
// TEACH POINTS
// ══════════════════════════════════════════════════════════════

function recordTeachPoint() {
  const name = 'P[' + (teachPoints.length + 1) + ']';
  teachPoints.push({
    name: name,
    joints: [...currentJoints],
    tcp: tcpWorldPos.clone(),
    timestamp: new Date().toLocaleTimeString(),
  });
  renderTeachPoints();
}

function renderTeachPoints() {
  const list = document.getElementById('teach-points-list');
  if (!list) return;
  if (teachPoints.length === 0) {
    list.innerHTML = '<span class="teach-empty">未记录示教点</span>';
    return;
  }
  list.innerHTML = teachPoints.map(function(tp, i) {
    return '<div class="teach-point-item" onclick="moveToTeachPoint(' + i + ')">' +
      '<span class="tp-name">' + tp.name + '</span>' +
      '<span class="tp-pos">X:' + tp.tcp.x.toFixed(0) + ' Y:' + tp.tcp.y.toFixed(0) + ' Z:' + tp.tcp.z.toFixed(0) + '</span>' +
      '</div>';
  }).join('');
}

async function moveToTeachPoint(index) {
  const tp = teachPoints[index];
  if (!tp) return;
  try {
    const resp = await fetch(`${ROBOT_URL}/execute`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        action: 'move',
        target: { joints: tp.joints },
      }),
    });
    const data = await resp.json();
    if (data.sequence && data.sequence[0] && data.sequence[0].trajectory) {
      const traj = data.sequence[0].trajectory;
      for (let i = 0; i < traj.length; i += 2) {
        await animateToJoints(traj[i].joints, 150);
      }
      await animateToJoints(traj[traj.length - 1].joints, 400);
    }
  } catch (e) {
    await animateToJoints(tp.joints, 500);
  }
}

async function playTeachPoints() {
  if (teachPoints.length === 0) return;
  updatePipelineStatus('示教回放中...');
  for (let i = 0; i < teachPoints.length; i++) {
    await moveToTeachPoint(i);
    await new Promise(function(r) { setTimeout(r, 500); });
  }
  updatePipelineStatus('回放完成');
}

function clearTeachPoints() {
  teachPoints = [];
  renderTeachPoints();
}

// ══════════════════════════════════════════════════════════════
// PROGRAM GENERATOR (FANUC TP Style)
// ══════════════════════════════════════════════════════════════

function generateProgram() {
  var lines = [];
  var now = new Date().toLocaleString();

  lines.push('; ========================================');
  lines.push('; STYLE.MN — 视觉引导主程序');
  lines.push('; 自动生成于: ' + now);
  lines.push('; FANUC LR Mate 200iD / R-30iB Mate Plus');
  lines.push('; ========================================');
  lines.push('');
  lines.push('   1:  UFRAME_NUM=1 ;');
  lines.push('   2:  UTOOL_NUM=1 ;');
  lines.push('   3:  PAYLOAD[1] ;');
  lines.push('   4:  J P[1:home] 100% FINE ;');
  lines.push('   5:  SET SEGMENT 1 ;');
  lines.push('');

  if (teachPoints.length > 0) {
    lines.push('   ; --- 示教点 --- ;');
    teachPoints.forEach(function(tp, i) {
      var jd = tp.joints.map(function(j) { return (j * 180 / Math.PI).toFixed(1); });
      lines.push('   ; P[' + (i + 2) + '] 关节: [' + jd.join(', ') + '] ;');
    });
    lines.push('');
  }

  if (programActions.length > 0) {
    lines.push('   ; --- Agent 自主操作序列 --- ;');
    programActions.forEach(function(action, i) {
      if (action.type === 'tool_call') {
        lines.push('   ; --- 步骤 ' + (i + 1) + ': ' + action.tool + ' --- ;');
        if (action.tool === 'analyze_scene') {
          lines.push('   ; 视觉场景分析 ;');
          if (action.result && action.result.count !== undefined) {
            lines.push('   ; 检测到 ' + action.result.count + ' 个物体 ;');
          }
        } else if (action.tool === 'get_robot_status') {
          lines.push('   ; 查询机器人状态 ;');
        } else if (action.tool === 'reset_robot') {
          lines.push('   ; 机器人复位 ;');
          lines.push('   J P[1:home] 100% FINE ;');
        } else if (action.tool === 'pick_and_place') {
          var a = action.args;
          lines.push('   ; 抓取: (' + (a.x||0).toFixed(0) + ', ' + (a.y||0).toFixed(0) + ', ' + (a.z||0).toFixed(0) + ') ;');
          lines.push('   ; 放置: (' + (a.target_x||0).toFixed(0) + ', ' + (a.target_y||0).toFixed(0) + ', ' + (a.target_z||0).toFixed(0) + ') ;');
          lines.push('   CALL S01PICK1 ;');
          lines.push('   CALL S01DROP1 ;');
        } else if (action.tool === 'pick_object') {
          var b = action.args;
          lines.push('   ; 抓取: (' + (b.x||0).toFixed(0) + ', ' + (b.y||0).toFixed(0) + ', ' + (b.z||0).toFixed(0) + ') ;');
          lines.push('   CALL S01PICK1 ;');
        } else if (action.tool === 'place_object') {
          var c = action.args;
          lines.push('   ; 放置: (' + (c.x||0).toFixed(0) + ', ' + (c.y||0).toFixed(0) + ', ' + (c.z||0).toFixed(0) + ') ;');
          lines.push('   CALL S01DROP1 ;');
        } else if (action.tool === 'move_robot_to') {
          var d = action.args;
          lines.push('   ; 移动到: (' + (d.x||0).toFixed(0) + ', ' + (d.y||0).toFixed(0) + ', ' + (d.z||0).toFixed(0) + ') ;');
        }
      }
    });
    lines.push('');
  }

  lines.push('   ; ======================================== ;');
  lines.push('   ; 子程序模板 (S01PICK1) ;');
  lines.push('   ; ======================================== ;');
  lines.push('   ; S01PICK1: ;');
  lines.push('   ;   1:  J P[10] 100% CNT50 ;     // 接近点 ;');
  lines.push('   ;   2:  L P[11] 20% FINE ;       // 抓取点 ;');
  lines.push('   ;   3:  CALL GRIPPART ;          // 闭合夹爪 ;');
  lines.push('   ;   4:  L P[12] 50% CNT30 ;     // 退避点 ;');
  lines.push('   ; ======================================== ;');
  lines.push('   ; 子程序模板 (S01DROP1) ;');
  lines.push('   ;   1:  J P[20] 100% CNT50 ;     // 接近点 ;');
  lines.push('   ;   2:  L P[21] 20% FINE ;       // 放置点 ;');
  lines.push('   ;   3:  CALL DROPPART ;          // 释放夹爪 ;');
  lines.push('   ;   4:  L P[22] 50% CNT30 ;     // 退避点 ;');
  lines.push('   ; ======================================== ;');
  lines.push('');
  lines.push('   ; --- 回原点 --- ;');
  lines.push('   CALL MOV_HOME ;');
  lines.push('   ; --- 结束 --- ;');
  lines.push('   END ;');

  var codeEl = document.getElementById('program-code');
  if (codeEl) codeEl.textContent = lines.join('\n');
}

function exportProgram() {
  var codeEl = document.getElementById('program-code');
  if (!codeEl || !codeEl.textContent) return;
  var blob = new Blob([codeEl.textContent], { type: 'text/plain' });
  var url = URL.createObjectURL(blob);
  var a = document.createElement('a');
  a.href = url;
  a.download = 'STYLE_MN.LS';
  a.click();
  URL.revokeObjectURL(url);
}

async function sendProgramToRobot() {
  var codeEl = document.getElementById('program-code');
  if (!codeEl || !codeEl.textContent) return;
  try {
    var resp = await fetch(`${ROBOT_URL}/program/upload`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ program: codeEl.textContent }),
    });
    var data = await resp.json();
    alert(data.success ? '程序已发送到虚拟控制器' : '发送失败: ' + (data.error || '未知错误'));
  } catch (e) {
    alert('发送程序失败: ' + e.message);
  }
}

// ══════════════════════════════════════════════════════════════
// WINDOW EXPORTS (ES module → HTML onclick)
// ══════════════════════════════════════════════════════════════

window.sendMessage = sendMessage;
window.stopExecution = stopExecution;
window.handleKeyDown = handleKeyDown;
window.quickCommand = quickCommand;
window.resetView = resetView;
window.toggleGrid = toggleGrid;
window.resetRobot = resetRobot;
window.readyRobot = readyRobot;
window.updateRobotJoints = updateRobotJoints;
window.captureVirtualFrame = captureVirtualFrame;
window.setJogMode = setJogMode;
window.updateJogStep = updateJogStep;
window.updateJogSpeed = updateJogSpeed;
window.jogJoint = jogJoint;
window.jogCartesian = jogCartesian;
window.recordTeachPoint = recordTeachPoint;
window.moveToTeachPoint = moveToTeachPoint;
window.playTeachPoints = playTeachPoints;
window.clearTeachPoints = clearTeachPoints;
window.generateProgram = generateProgram;
window.exportProgram = exportProgram;
window.sendProgramToRobot = sendProgramToRobot;
window.switchMode = switchMode;
