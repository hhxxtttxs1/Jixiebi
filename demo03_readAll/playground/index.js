import {
  WebGLRenderer,
  PerspectiveCamera,
  Scene,
  Mesh,
  PlaneGeometry,
  ShadowMaterial,
  DirectionalLight,
  PCFSoftShadowMap,
  // sRGBEncoding,
  Color,
  AmbientLight,
  Box3,
  LoadingManager,
  MathUtils,
  MeshPhysicalMaterial,
  DoubleSide,
  ACESFilmicToneMapping,
  CanvasTexture,
  Float32BufferAttribute,
  RepeatWrapping,
} from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import URDFLoader from 'urdf-loader';
// 导入控制工具函数
import { setupKeyboardControls, setupControlPanel } from './robotControls.js';

// 定义旋转矩阵函数
function rotateX(theta) {
    return [
        [1, 0, 0],
        [0, Math.cos(theta), -Math.sin(theta)],
        [0, Math.sin(theta), Math.cos(theta)]
    ];
}

function rotateY(theta) {
    return [
        [Math.cos(theta), 0, Math.sin(theta)],
        [0, 1, 0],
        [-Math.sin(theta), 0, Math.cos(theta)]
    ];
}

function rotateZ(theta) {
    return [
        [Math.cos(theta), -Math.sin(theta), 0],
        [Math.sin(theta), Math.cos(theta), 0],
        [0, 0, 1]
    ];
}

// 矩阵乘法
function multiplyMatrices(a, b) {
    const result = [];
    for (let i = 0; i < a.length; i++) {
        result[i] = [];
        for (let j = 0; j < b[0].length; j++) {
            let sum = 0;
            for (let k = 0; k < a[0].length; k++) {
                sum += a[i][k] * b[k][j];
            }
            result[i][j] = sum;
        }
    }
    return result;
}

// 矩阵与向量乘法
function multiplyMatrixVector(matrix, vector) {
    const result = [0, 0, 0];
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            result[i] += matrix[i][j] * vector[j];
        }
    }
    return result;
}

// 向量加法
function addVectors(a, b) {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

// 机械臂运动学正解函数
function forwardKinematics(jointAngles) {
    // 提取关节角度（转换为弧度）
    const theta1 = jointAngles[0] * Math.PI / 180;
    const theta2 = jointAngles[1] * Math.PI / 180;
    const theta3 = jointAngles[2] * Math.PI / 180;
    const theta4 = jointAngles[3] * Math.PI / 180;
    const theta5 = jointAngles[4] * Math.PI / 180;
    const theta6 = jointAngles[5] * Math.PI / 180;
    
    // 初始化位置
    let position = [0, 0, 0];
    let orientation = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]; // 初始姿态为单位矩阵
    
    // 关节1: 腰部旋转 (绕X轴)
    const trans1 = [-0.013, 0, 0.0265];
    const rot1 = multiplyMatrices(rotateX(theta1), rotateY(-Math.PI/2));
    position = addVectors(position, multiplyMatrixVector(orientation, trans1));
    orientation = multiplyMatrices(orientation, rot1);
    
    // 关节2: 大臂控制 (绕Y轴)
    const trans2 = [0.081, 0, 0];
    const rot2 = multiplyMatrices(rotateY(Math.PI/2), rotateY(theta2));
    position = addVectors(position, multiplyMatrixVector(orientation, trans2));
    orientation = multiplyMatrices(orientation, rot2);
    
    // 关节3: 小臂控制 (绕Y轴)
    const trans3 = [0, 0, 0.118];
    const rot3 = rotateY(theta3);
    position = addVectors(position, multiplyMatrixVector(orientation, trans3));
    orientation = multiplyMatrices(orientation, rot3);
    
    // 关节4: 腕部控制 (绕Y轴)
    const trans4 = [0, 0, 0.118];
    const rot4 = rotateY(theta4);
    position = addVectors(position, multiplyMatrixVector(orientation, trans4));
    orientation = multiplyMatrices(orientation, rot4);
    
    // 关节5: 腕部旋转 (绕Z轴)
    const trans5 = [0, 0, 0.0635];
    const rot5 = rotateZ(theta5);
    position = addVectors(position, multiplyMatrixVector(orientation, trans5));
    orientation = multiplyMatrices(orientation, rot5);
    
    // 关节6: 爪子控制 (绕X轴)
    const trans6 = [0, -0.0132, 0.021];
    const rot6 = rotateX(theta6);
    position = addVectors(position, multiplyMatrixVector(orientation, trans6));
    orientation = multiplyMatrices(orientation, rot6);
    
    return position;
}

// 更新夹爪坐标显示
function updateEndEffectorPosition(position) {
    const positionX = document.getElementById('positionX');
    const positionY = document.getElementById('positionY');
    const positionZ = document.getElementById('positionZ');
    
    if (positionX && positionY && positionZ) {
        positionX.textContent = position[0].toFixed(3);
        positionY.textContent = position[1].toFixed(3);
        positionZ.textContent = position[2].toFixed(3);
    }
}

// 声明为全局变量
let scene, camera, renderer, controls;
// 将robot设为全局变量，便于其他模块访问
window.robot = null;
let keyboardUpdate;

init();
render();

function init() {

  scene = new Scene();
  scene.background = new Color(0x263238);

  camera = new PerspectiveCamera();
  camera.position.set(5, 5, 5);
  camera.lookAt(0, 0, 0);

  renderer = new WebGLRenderer({ antialias: true });
  // renderer.outputEncoding = sRGBEncoding;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = PCFSoftShadowMap;
  renderer.physicallyCorrectLights = true;
  renderer.toneMapping = ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.5;
  document.body.appendChild(renderer.domElement);

  const directionalLight = new DirectionalLight(0xffffff, 1.0);
  directionalLight.castShadow = true;
  directionalLight.shadow.mapSize.setScalar(1024);
  directionalLight.position.set(5, 30, 5);
  scene.add(directionalLight);

  // Add second directional light for better reflections
  const directionalLight2 = new DirectionalLight(0xffffff, 0.8);
  directionalLight2.position.set(-2, 10, -5);
  scene.add(directionalLight2);

  const ambientLight = new AmbientLight(0xffffff, 0.3);
  scene.add(ambientLight);

  // Create reflective floor (MuJoCo style)
  const groundMaterial = new MeshPhysicalMaterial({
    color: 0x808080,
    metalness: 0.7,
    roughness: 0.3,
    reflectivity: 0.1,
    clearcoat: 0.3,
    side: DoubleSide,
    transparent: true,     // 启用透明度
    opacity: 0.7,          // 设置透明度为0.7（可以根据需要调整，1.0为完全不透明）
  });
  
  // 创建格子纹理的地面
  const gridSize = 60;
  const divisions = 60;
  
  // 创建网格地面
  const ground = new Mesh(new PlaneGeometry(gridSize, gridSize, divisions, divisions), groundMaterial);
  
  // 添加格子纹理
  const geometry = ground.geometry;
  const positionAttribute = geometry.getAttribute('position');
  
  // 创建格子纹理的UV坐标
  const uvs = [];
  const gridScale = 0.01; // 控制格子的密度
  
  for (let i = 0; i < positionAttribute.count; i++) {
    const x = positionAttribute.getX(i);
    const y = positionAttribute.getY(i);
    
    uvs.push(x * gridScale, y * gridScale);
  }
  
  geometry.setAttribute('uv', new Float32BufferAttribute(uvs, 2));
  
  // 更新材质，添加格子纹理
  groundMaterial.map = createGridTexture();
  groundMaterial.roughnessMap = createGridTexture();
  
  ground.rotation.x = -Math.PI / 2;
  ground.receiveShadow = true;
  scene.add(ground);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.minDistance = 4;
  controls.target.y = 1;
  controls.update();

  // 根据URL hash或默认加载模型
  function loadModelFromHash() {
    // 获取URL hash（去掉#号）
    let modelToLoad = 'genkiarm';
    
    // 加载模型
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);

    loader.load(`/URDF/${modelToLoad}.urdf`, result => {
      window.robot = result;
    });

    // 等待模型加载完成
    manager.onLoad = () => {
      window.robot.rotation.x = - Math.PI / 2;
      window.robot.rotation.z = - Math.PI;
      window.robot.traverse(c => {
        c.castShadow = true;
      });
      console.log(window.robot.joints);
      // 记录关节限制信息到控制台，便于调试
      logJointLimits(window.robot);
      
      window.robot.updateMatrixWorld(true);

      const bb = new Box3();
      bb.setFromObject(window.robot);

      window.robot.scale.set(15, 15, 15);
      window.robot.position.y -= bb.min.y;
      scene.add(window.robot);

      // Initialize keyboard controls
      keyboardUpdate = setupKeyboardControls(window.robot);
    };
  }

  // 初始加载模型
  loadModelFromHash();

  onResize();
  window.addEventListener('resize', onResize);

  // Setup UI for control panel
  setupControlPanel();

  // Setup WebSocket connection for receiving servo angles from Python
  setupWebSocketConnection();
  
  }

/**
 * 设置WebSocket连接，接收Python发送的舵机角度数据
 */
function setupWebSocketConnection() {
  // WebSocket服务器地址，默认使用localhost:8000
  const wsUrl = 'ws://localhost:8000';
  let ws;
  let reconnectInterval;
  
  function connect() {
    console.log('正在尝试连接WebSocket服务器...');
    
    try {
      ws = new WebSocket(wsUrl);
      
      ws.onopen = function() {
        console.log('WebSocket连接已建立');
        // 连接成功后清除重连定时器
        if (reconnectInterval) {
          clearInterval(reconnectInterval);
          reconnectInterval = null;
        }
      };
      
      ws.onmessage = function(event) {
        try {
          // 解析接收到的JSON数据
          const data = JSON.parse(event.data);
          
          // 检查数据格式是否正确
          if (data.servos && Array.isArray(data.servos) && data.servos.length === 6) {
            console.log('接收到舵机角度数据:', data.servos);
            
            // 更新3D模型的关节角度
            updateRobotJoints(data.servos);
          } else {
            console.warn('接收到的数据格式不正确:', data);
          }
        } catch (error) {
          console.error('解析WebSocket数据时出错:', error);
        }
      };
      
      ws.onerror = function(error) {
        console.error('WebSocket错误:', error);
      };
      
      ws.onclose = function(event) {
        console.log('WebSocket连接已关闭，代码:', event.code, '原因:', event.reason);
        // 尝试重新连接
        if (!reconnectInterval) {
          reconnectInterval = setInterval(connect, 3000);
        }
      };
    } catch (error) {
      console.error('创建WebSocket连接时出错:', error);
      // 尝试重新连接
      if (!reconnectInterval) {
        reconnectInterval = setInterval(connect, 3000);
      }
    }
  }
  
  // 开始连接
  connect();
}

/**
 * 根据接收到的舵机角度数据更新机器人关节
 * @param {Array} servoAngles - 6个舵机的角度数组，范围-90到90度
 */
function updateRobotJoints(servoAngles) {
  if (!window.robot || !window.robot.joints) {
    console.warn('机器人模型尚未加载完成');
    return;
  }
  
  // 获取关节名称数组
  const jointNames = Object.keys(window.robot.joints).filter(name => 
    window.robot.joints[name].jointType !== 'fixed'
  );
  
  // 确保关节数量匹配
  if (jointNames.length < 6) {
    console.warn('机器人关节数量不足6个');
    return;
  }
  
  // 遍历更新每个关节
  servoAngles.forEach((angle, index) => {
    if (index < jointNames.length) {
      const jointName = jointNames[index];
      const joint = window.robot.joints[jointName];
      
      // 将角度转换为弧度
      const angleRad = MathUtils.degToRad(angle);
      
      // 更新关节角度
      if (joint.setJointValue) {
        joint.setJointValue(angleRad);
      }
    }
  });
  
  // 更新机器人的矩阵世界
  if (window.robot.updateMatrixWorld) {
    window.robot.updateMatrixWorld(true);
  }
  
  // 计算并更新夹爪坐标
  const endEffectorPosition = forwardKinematics(servoAngles);
  updateEndEffectorPosition(endEffectorPosition);
}

/**
 * 输出关节限制信息到控制台
 * @param {Object} robot - 机器人对象
 */
function logJointLimits(robot) {
  if (!robot || !robot.joints) return;
  
  console.log("Robot joint limits:");
  Object.entries(robot.joints).forEach(([name, joint]) => {
    console.log(`Joint: ${name}`);
    console.log(`  Type: ${joint.jointType}`);
    
    if (joint.jointType !== 'fixed' && joint.jointType !== 'continuous') {
      console.log(`  Limits: ${joint.limit.lower.toFixed(4)} to ${joint.limit.upper.toFixed(4)} rad`);
      console.log(`  Current value: ${Array.isArray(joint.jointValue) ? joint.jointValue.join(', ') : joint.jointValue}`);
    } else if (joint.jointType === 'continuous') {
      console.log(`  No limits (continuous joint)`);
    } else {
      console.log(`  No limits (fixed joint)`);
    }
  });
}

function onResize() {
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(window.devicePixelRatio);

  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
}

function render() {
  requestAnimationFrame(render);
  
  // Update joint positions based on keyboard input
  if (keyboardUpdate) {
    keyboardUpdate();
  }
  
  renderer.render(scene, camera);
}

// 添加创建格子纹理的函数
function createGridTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 512;
  
  const context = canvas.getContext('2d');
  
  // 填充底色
  context.fillStyle = '#808080';
  context.fillRect(0, 0, canvas.width, canvas.height);
  
  // 绘制格子线
  context.lineWidth = 1;
  context.strokeStyle = '#606060';
  
  const cellSize = 32; // 每个格子的大小
  
  for (let i = 0; i <= canvas.width; i += cellSize) {
    context.beginPath();
    context.moveTo(i, 0);
    context.lineTo(i, canvas.height);
    context.stroke();
  }
  
  for (let i = 0; i <= canvas.height; i += cellSize) {
    context.beginPath();
    context.moveTo(0, i);
    context.lineTo(canvas.width, i);
    context.stroke();
  }
  
  // 修复: 使用已导入的 CanvasTexture，而不是 THREE.CanvasTexture
  const texture = new CanvasTexture(canvas);
  // 修复: 使用已导入的 RepeatWrapping，而不是 THREE.RepeatWrapping
  texture.wrapS = RepeatWrapping;
  texture.wrapT = RepeatWrapping;
  texture.repeat.set(10, 10);
  
  return texture;
}
