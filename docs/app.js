import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

/*
 * Intrinsic Core VI / OMTS visual demonstrator.
 * Visual approximation only. NOT Gazebo, ICON, FoundationPose, MoveIt or real I/O.
 * No requests are sent to a robot, PLC or CNC. All state lives in this page.
 */
const $ = (id) => document.getElementById(id);
const STEPS = [
  {title:'Quan sát phôi',short:'Camera RGB-D',desc:'Camera ảo quét khu vực cấp phôi; vòng quan sát chỉ minh họa dữ liệu cảm biến.',duration:3.6,skill:'capture_images'},
  {title:'Xác định vị trí',short:'Ước lượng tư thế 6D',desc:'Hiển thị tọa độ phôi giả lập; không chạy mô hình FoundationPose.',duration:3.5,skill:'estimate_pose'},
  {title:'Tính đường đi',short:'Quỹ đạo tránh chướng ngại',desc:'Hiển thị các điểm trung gian lập sẵn; chưa chạy bộ giải MoveIt 2.',duration:3.4,skill:'move_robot'},
  {title:'Gắp phôi',short:'Tiếp cận và kẹp',desc:'Tay robot tiếp cận phôi, đóng kẹp và nâng vật khỏi khay cấp liệu.',duration:6.0,skill:'move_to_contact'},
  {title:'Đưa vào máy CNC',short:'Nạp phôi / kẹp ê-tô',desc:'Mở cửa CNC, đưa phôi vào ê-tô, kẹp chặt và rút tay robot.',duration:6.0,skill:'dio_set_output'},
  {title:'Kích hoạt gia công',short:'Chu trình gia công',desc:'Đóng cửa, kiểm tra khóa liên động minh họa và chạy chu trình gia công giả lập.',duration:5.5,skill:'machine_cycle'},
  {title:'Lấy sản phẩm',short:'Tháo phôi gia công',desc:'Mở cửa, nhả ê-tô, gắp chi tiết đã gia công và đưa ra khỏi máy.',duration:6.0,skill:'unload_machine'},
  {title:'Đặt vào khay',short:'Xuất thành phẩm',desc:'Di chuyển sản phẩm đến khay đầu ra, nhả kẹp rồi về vị trí chờ.',duration:6.2,skill:'return_infeed'},
];
const totalDuration=STEPS.reduce((s,x)=>s+x.duration,0);
const pathTimes=STEPS.map((_,i)=>STEPS.slice(0,i).reduce((s,x)=>s+x.duration,0));
const V={home:[-.64,1.87,.14],view:[-1.66,1.77,-.23],feedHigh:[-2.43,1.52,-.25],feedGrip:[-2.43,1.03,-.25],feedLift:[-2.31,1.77,-.25],bridge:[-.35,2.02,-.26],cncHigh:[.99,1.42,-.44],cncGrip:[.99,1.00,-.44],cncSafe:[.15,1.89,-.27],outHigh:[-2.25,1.59,1.30],outGrip:[-2.25,1.00,1.30]};
const INPUT=new THREE.Vector3(-2.43,.86,-.25), CNC=new THREE.Vector3(.99,.84,-.44), OUTPUT=new THREE.Vector3(-2.25,.85,1.30);
const lerp=(a,b,t)=>a+(b-a)*Math.max(0,Math.min(1,t));
const ease=(t)=>{t=THREE.MathUtils.clamp(t,0,1);return t*t*(3-2*t);};
const point=(arr)=>new THREE.Vector3(...arr);
function atKeyframes(list,p){for(let i=1;i<list.length;i++){if(p<=list[i][0]){let k=ease((p-list[i-1][0])/(list[i][0]-list[i-1][0]));return point(list[i-1][1]).lerp(point(list[i][1]),k);}}return point(list[list.length-1][1]);}
function snapshot(s,p){
  let pose=point(V.home),piece='input',jaw=1,door=0,vise=0,spindle=false,processed=false,vision=0;
  if(s===0){pose=atKeyframes([[0,V.home],[.65,V.view],[1,V.view]],p);vision=.35+p*.5;}
  if(s===1){pose=point(V.view);vision=1;}
  if(s===2){pose=point(V.view);vision=1;}
  if(s===3){pose=atKeyframes([[0,V.view],[.2,V.feedHigh],[.45,V.feedGrip],[.58,V.feedGrip],[.76,V.feedLift],[1,V.feedLift]],p);piece=p>=.50?'held':'input';jaw=p>=.50?0:1;vision=p<.18?1:0;}
  if(s===4){pose=atKeyframes([[0,V.feedLift],[.29,V.bridge],[.56,V.cncHigh],[.76,V.cncGrip],[.87,V.cncGrip],[1,V.cncSafe]],p);door=1;piece=p>=.84?'cnc':'held';jaw=p>=.84?1:0;vise=p>=.88?1:0;}
  if(s===5){pose=atKeyframes([[0,V.cncSafe],[.23,V.home],[1,V.home]],p);piece='cnc';jaw=1;vise=1;door=1-ease(p/.18);spindle=p>.22&&p<.86;processed=p>.85;}
  if(s===6){pose=atKeyframes([[0,V.home],[.25,V.cncSafe],[.44,V.cncHigh],[.55,V.cncGrip],[.67,V.cncGrip],[.86,V.cncHigh],[1,V.cncSafe]],p);door=ease(p/.19);vise=p<.26?1:1-ease((p-.26)/.16);piece=p>=.63?'held':'cnc';jaw=p>=.63?0:1;processed=true;}
  if(s===7){pose=atKeyframes([[0,V.cncSafe],[.22,V.bridge],[.48,V.outHigh],[.69,V.outGrip],[.79,V.outGrip],[.91,V.outHigh],[1,V.home]],p);piece=p>=.77?'output':'held';jaw=p>=.77?1:0;door=1-ease((p-.09)/.27);processed=true;}
  return {pose,piece,jaw,door,vise,spindle,processed,vision};
}
const view=$('viewport'),err=$('sceneLoadError');
let scene,camera,renderer,controls,robot,part,doorObj,clamps,spindle,spindleLight,visionBox,visionRing,pathLine,scanBeam,scanLight,baseStep=null,grid;
let simStep=0,stepProgress=0,playing=false,speed=1,cycles=0,lastFrame=0,fpsAt=0,fpsFrames=0;
const STEP_LIST=$('stepList');let lastStepEvent=-1;

function mat(color,metalness=.1,roughness=.55,other={}){return new THREE.MeshStandardMaterial({color,metalness,roughness,...other});}
const M={
  dark:mat('#172337',.4,.52), panel:mat('#273a52',.42,.45), blue:mat('#4cccdc',.54,.34),
  turquoise:mat('#3ae5dd',.4,.25,{emissive:'#08666c',emissiveIntensity:.35}),
  joint:mat('#102738',.6,.3), metal:mat('#8aafc3',.8,.26), floor:mat('#102034',.1,.95),
  table:mat('#294254',.5,.42), amber:mat('#ebaa56',.57,.35), finished:mat('#98c8d7',.91,.2),
  glass:mat('#71cbe5',.0,.18,{transparent:true,opacity:.12,side:THREE.DoubleSide,depthWrite:false}),
  guide:mat('#48cecb',.0,.9,{transparent:true,opacity:.45,emissive:'#0b454e',emissiveIntensity:.15})
};
function addBox(parent,w,h,d,x,y,z,material,shadow=true){
  const o=new THREE.Mesh(new THREE.BoxGeometry(w,h,d),material);o.position.set(x,y,z);o.castShadow=shadow;o.receiveShadow=shadow;parent.add(o);return o;
}
function addCylinder(parent,rt,rb,h,x,y,z,material,n=28){
  const o=new THREE.Mesh(new THREE.CylinderGeometry(rt,rb,h,n),material);o.position.set(x,y,z);o.castShadow=true;o.receiveShadow=true;parent.add(o);return o;
}
function rod(parent,a,b,r,material){const cyl=new THREE.Mesh(new THREE.CylinderGeometry(r,r,1,16),material);cyl.castShadow=true;parent.add(cyl);placeRod(cyl,a,b);return cyl;}
function placeRod(cyl,a,b){const mid=a.clone().add(b).multiplyScalar(.5),delta=b.clone().sub(a);cyl.position.copy(mid);cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),delta.clone().normalize());cyl.scale.y=delta.length();}
function makeLabel(name,position,accent='#7abacc'){
  const canvas=document.createElement('canvas');canvas.width=512;canvas.height=100;
  const c=canvas.getContext('2d');c.fillStyle='rgba(10,24,39,.84)';c.fillRect(0,0,512,100);
  c.fillStyle=accent;c.fillRect(0,0,8,100);c.fillStyle='#d8ebf5';c.font='600 35px Arial';c.fillText(name,24,62);
  const texture=new THREE.CanvasTexture(canvas);texture.colorSpace=THREE.SRGBColorSpace;
  const sprite=new THREE.Sprite(new THREE.SpriteMaterial({map:texture,transparent:true,depthTest:false,depthWrite:false}));sprite.scale.set(1.9,.38,1);sprite.position.set(...position);scene.add(sprite);return sprite;
}
function buildRoom(){
  scene.background=new THREE.Color('#0b1928');
  scene.fog=new THREE.Fog('#0b1928',12,27);
  const ambient=new THREE.HemisphereLight('#c9e8ff','#18253b',2.15);scene.add(ambient);
  const sun=new THREE.DirectionalLight('#f3ffff',3.0);sun.position.set(-2.5,7,4);sun.castShadow=true;sun.shadow.mapSize.set(2048,2048);sun.shadow.camera.left=-8;sun.shadow.camera.right=8;sun.shadow.camera.top=8;sun.shadow.camera.bottom=-8;sun.shadow.normalBias=.02;scene.add(sun);
  const warm=new THREE.PointLight('#58bde0',16,8);warm.position.set(-3.2,3,-1.2);scene.add(warm);
  const floor=new THREE.Mesh(new THREE.PlaneGeometry(13,11),M.floor);floor.rotation.x=-Math.PI/2;floor.position.y=-.018;floor.receiveShadow=true;scene.add(floor);
  grid=new THREE.GridHelper(13,26,'#3b5572','#1e3044');grid.position.y=.003;grid.material.transparent=true;grid.material.opacity=.38;scene.add(grid);
  addBox(scene,12,.045,.04,0,.02,-4.0,M.panel);
  addBox(scene,.04,.045,8,5.6,.02,0,M.panel);
  // Platform markings / clearance pads
  const pad=addBox(scene,2.45,.022,2.2,-1.07,.02,0,mat('#142f3e',.05,.9));pad.receiveShadow=true;
  for(const x of [-2.95,-1.58,-.31,.64,2.78]){const strip=addBox(scene,.015,.006,2.7,x,.034,-.10,M.guide,false);strip.material.transparent=true;}
}
function makeTable(x,z,w=1.43,d=1.22,y=.69){
  const g=new THREE.Group();scene.add(g);
  addBox(g,w,.13,d,x,y,z,M.table);
  for(const i of [-1,1])for(const j of [-1,1])addBox(g,.08,y-.04,.08,x+i*(w/2-.13),(y-.04)/2,z+j*(d/2-.13),M.dark);
  for(let n=-2;n<=2;n++){const line=addBox(g,.012,.008,d-.12,x+n*(w-.15)/6,y+.072,z,M.metal,false);line.material=M.metal;}
}
function makeWorkpiece(){
  part=new THREE.Group();scene.add(part);
  const body=addCylinder(part,.16,.16,.19,0,0,0,M.amber,32);
  const ring=addCylinder(part,.18,.18,.052,0,.105,0,M.metal,32);
  addCylinder(part,.095,.095,.07,0,.155,0,M.amber,32);
  part.userData={body,ring};part.position.copy(INPUT);
}
function buildCNC(){
  // CNC front is toward the robot (-X). Transparent side allows inspection.
  addBox(scene,1.63,.18,1.72,1.79,.11,-.44,M.dark);
  const cncY=1.35,cz=-.44;
  addBox(scene,1.55,.24,1.77,1.79,2.27,cz,M.panel);
  addBox(scene,.17,1.94,.17,2.5,1.27,-1.25,M.panel);
  addBox(scene,.17,1.94,.17,2.5,1.27,.37,M.panel);
  addBox(scene,.16,1.95,.12,.91,1.27,-1.25,M.panel);
  addBox(scene,.16,1.95,.12,.91,1.27,.37,M.panel);
  addBox(scene,.14,1.85,1.66,2.53,1.28,cz,M.panel);
  addBox(scene,1.54,1.85,.04,1.78,1.28,-1.27,M.glass,false);
  addBox(scene,1.54,1.85,.04,1.78,1.28,.40,M.glass,false);
  doorObj=new THREE.Group();scene.add(doorObj);
  addBox(doorObj,.075,1.76,.97,0,1.26,0,M.panel);
  addBox(doorObj,.09,1.18,.79,-.009,1.38,0,M.glass,false);
  addBox(doorObj,.11,.12,.12,-.085,1.12,.35,M.metal);
  doorObj.position.set(.84,0,cz-.3);
  addBox(scene,1.30,.15,1.08,1.63,.59,cz,M.dark);
  clamps=[
    addBox(scene,.42,.20,.17,1.14,.72,cz-.35,M.metal),
    addBox(scene,.42,.20,.17,1.14,.72,cz+.35,M.metal)
  ];
  addBox(scene,.4,.30,.36,1.25,1.8,cz,M.dark);
  spindle=addCylinder(scene,.078,.078,.52,1.13,1.37,cz,M.metal,20);
  addCylinder(spindle,.12,.12,.07,0,.12,0,M.turquoise,24);
  spindleLight=new THREE.PointLight('#ffa74f',0,2.2);spindleLight.position.set(1.13,1.13,cz);scene.add(spindleLight);
  const indicator=addBox(scene,.08,.08,.08,.80,2.18,.35,M.turquoise);indicator.material.emissiveIntensity=.85;
  makeLabel('CNC / CELL-01',[1.91,2.72,-.40],'#f7b86b');
}
function buildFeed(){
  makeTable(INPUT.x,INPUT.z,1.49,1.34,.70);
  makeTable(OUTPUT.x,OUTPUT.z,1.47,1.17,.69);
  addBox(scene,1.1,.09,.86,INPUT.x,.77,INPUT.z,M.dark);
  for(let i=-1;i<=1;i++){for(let j=-1;j<=1;j++){if(i===0&&j===0)continue;const bolt=addCylinder(scene,.04,.04,.015,INPUT.x+i*.34,.83,INPUT.z+j*.27,M.panel,12);}}
  const tray=addBox(scene,1.17,.12,.87,OUTPUT.x,.75,OUTPUT.z,M.dark);
  const trayMat=mat('#40778e',.55,.54);
  for(const z of [OUTPUT.z-.43,OUTPUT.z+.43])addBox(scene,1.18,.08,.035,OUTPUT.x,.83,z,trayMat);
  for(const x of [OUTPUT.x-.57,OUTPUT.x+.57])addBox(scene,.035,.08,.86,x,.83,OUTPUT.z,trayMat);
  makeLabel('INFEED / PHÔI',[INPUT.x,1.34,INPUT.z-1.01],'#edbb79');
  makeLabel('OUTFEED / KHAY',[OUTPUT.x,1.35,OUTPUT.z+.81],'#68deb8');
}
function buildCameraRig(){
  const stand=mat('#60778b',.65,.4);
  addBox(scene,.12,2.55,.12,-3.20,1.26,-1.30,stand);
  rod(scene,new THREE.Vector3(-3.20,2.5,-1.30),new THREE.Vector3(-2.42,2.5,-.79),.055,stand);
  const head=addBox(scene,.43,.19,.26,-2.39,2.48,-.75,M.metal);
  const lens=addCylinder(scene,.09,.09,.10,-2.39,2.34,-.75,M.blue,18);
  scanLight=new THREE.SpotLight('#4ae6e7',0,3.9,Math.PI/6,.8,.7);scanLight.position.set(-2.39,2.33,-.75);scanLight.target.position.copy(INPUT);scene.add(scanLight,scanLight.target);
  const origin=new THREE.Vector3(-2.39,2.30,-.75);
  const beamMaterial=new THREE.MeshBasicMaterial({color:'#52eaf0',transparent:true,opacity:.08,depthWrite:false,side:THREE.DoubleSide});
  const dir=INPUT.clone().sub(origin),length=dir.length();
  scanBeam=new THREE.Mesh(new THREE.ConeGeometry(.40,length,28,1,true),beamMaterial);
  scanBeam.position.copy(origin.clone().add(INPUT).multiplyScalar(.5));
  scanBeam.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),dir.clone().normalize());scanBeam.visible=false;scene.add(scanBeam);
  const bbox=new THREE.BoxGeometry(.47,.43,.47);
  visionBox=new THREE.LineSegments(new THREE.EdgesGeometry(bbox),new THREE.LineBasicMaterial({color:'#57e9f2',transparent:true,opacity:.85}));
  visionBox.position.copy(INPUT);visionBox.position.y=.9;scene.add(visionBox);
  visionRing=new THREE.Mesh(new THREE.TorusGeometry(.4,.012,10,60),new THREE.MeshBasicMaterial({color:'#5fdfe8'}));visionRing.rotation.x=-Math.PI/2;visionRing.position.copy(INPUT);visionRing.position.y=.85;scene.add(visionRing);
  makeLabel('RGB-D CAM-01',[-2.88,2.85,-1.15],'#5fdfe8');
}
function buildRobot(){
  const base=new THREE.Vector3(-1.05,0,0),baseJoint=new THREE.Vector3(base.x,.82,base.z);
  addCylinder(scene,.48,.53,.18,base.x,.16,base.z,M.panel,48);
  addCylinder(scene,.35,.34,.42,base.x,.43,base.z,M.dark,32);
  addCylinder(scene,.25,.31,.22,base.x,.70,base.z,M.turquoise,32);
  const upper=rod(scene,baseJoint,new THREE.Vector3(-1.2,1.3,0),.155,M.metal);
  const lower=rod(scene,new THREE.Vector3(-1.2,1.3,0),new THREE.Vector3(-1.1,1.8,0),.117,M.metal);
  const shoulder=addCylinder(scene,.22,.22,.18,base.x,.82,base.z,M.blue,28);
  const elbow=addCylinder(scene,.18,.18,.19,-1.2,1.3,0,M.turquoise,28);
  const wrist=addCylinder(scene,.135,.135,.15,-1.1,1.8,0,M.joint,28);
  const tool=new THREE.Group();scene.add(tool);
  addBox(tool,.26,.14,.22,0,.07,0,M.dark);
  const finger1=addBox(tool,.055,.22,.058,0,-.07,-.16,M.metal);
  const finger2=addBox(tool,.055,.22,.058,0,-.07,.16,M.metal);
  // Make the six-axis concept visible using mechanical modules; IK below is a
  // simplified two-link educational approximation, not a UR5e kinematics solver.
  robot={base,baseJoint,upper,lower,shoulder,elbow,wrist,tool,finger1,finger2};
  makeLabel('UR5e / ROBOT 01',[-.87,2.65,.6],'#5bdde7');
}
function animateRobot(tcp,jaw){
  const o=robot,base=o.baseJoint,delta=tcp.clone().sub(base);
  const h=Math.hypot(delta.x,delta.z)||.0001,dy=delta.y,L1=1.26,L2=1.29;
  const d=Math.min(L1+L2-.002,Math.max(.12,Math.hypot(h,dy))),ux=delta.x/h,uz=delta.z/h;
  const proj=(L1*L1-L2*L2+d*d)/(2*d),height=Math.sqrt(Math.max(.0001,L1*L1-proj*proj));
  const rad=(proj*h-height*dy)/d;
  const alt=(proj*dy+height*h)/d;
  const elbow=new THREE.Vector3(base.x+ux*rad,base.y+alt,base.z+uz*rad);
  placeRod(o.upper,base,elbow);placeRod(o.lower,elbow,tcp);
  o.elbow.position.copy(elbow);o.wrist.position.copy(tcp);
  o.tool.position.copy(tcp);o.tool.rotation.y=Math.atan2(delta.z,delta.x);
  o.finger1.position.z=lerp(-.072,-.16,jaw);
  o.finger2.position.z=lerp(.072,.16,jaw);
  o.shoulder.rotation.z=Math.sin(performance.now()*.0001)*.01;
}
function pathForStep(step){
  if(step===2)return [V.view,V.feedHigh,V.feedGrip,V.feedLift,V.bridge,V.cncHigh,V.cncGrip].map(point);
  if(step===3)return [V.view,V.feedHigh,V.feedGrip,V.feedLift].map(point);
  if(step===4)return [V.feedLift,V.bridge,V.cncHigh,V.cncGrip,V.cncSafe].map(point);
  if(step===6)return [V.home,V.cncSafe,V.cncHigh,V.cncGrip,V.cncHigh,V.cncSafe].map(point);
  if(step===7)return [V.cncSafe,V.bridge,V.outHigh,V.outGrip,V.outHigh,V.home].map(point);
  return [];
}
function buildPath(step){
  if(pathLine){scene.remove(pathLine);pathLine.geometry.dispose();pathLine.material.dispose();pathLine=null;}
  const points=pathForStep(step);if(points.length<2)return;
  const curve=new THREE.CatmullRomCurve3(points,false,'catmullrom',.2);
  const pts=curve.getPoints(100),geometry=new THREE.BufferGeometry().setFromPoints(pts);
  const material=new THREE.LineDashedMaterial({color:'#43dfaf',transparent:true,opacity:.83,dashSize:.18,gapSize:.09,linewidth:2});
  pathLine=new THREE.Line(geometry,material);pathLine.computeLineDistances();scene.add(pathLine);
}
function drawState(){
  if(!robot)return;
  const s=snapshot(simStep,stepProgress);
  animateRobot(s.pose,s.jaw);
  if(s.piece==='input')part.position.copy(INPUT);
  else if(s.piece==='cnc')part.position.copy(CNC);
  else if(s.piece==='output')part.position.copy(OUTPUT);
  else part.position.copy(s.pose).add(new THREE.Vector3(0,-.16,0));
  part.userData.body.material=s.processed?M.finished:M.amber;
  doorObj.position.z=-.74+1.22*s.door;
  clamps[0].position.z=-.44-lerp(.35,.19,s.vise);
  clamps[1].position.z=-.44+lerp(.35,.19,s.vise);
  spindle.rotation.y+=s.spindle?.13:0;
  spindle.position.y=1.37+(s.spindle?.12*Math.sin(performance.now()*.02):0);
  spindleLight.intensity=s.spindle?2.0:0;
  visionBox.visible=$('showVision').checked&&s.vision>.05;
  visionRing.visible=visionBox.visible;
  scanBeam.visible=visionBox.visible;
  scanBeam.material.opacity=.035+.08*s.vision;
  scanLight.intensity=visionBox.visible?10:0;
  visionRing.scale.setScalar(1+.07*Math.sin(performance.now()*.007));
  if(pathLine)pathLine.visible=$('showPath').checked;
  $('tcpPosition').textContent=[s.pose.x,s.pose.y,s.pose.z].map(x=>x.toFixed(2)).join(' / ');
  $('poseStatus').textContent=simStep<1?'Đang quét':(simStep===1?'6-DoF giả lập':'Đã định vị');
  $('gripperStatus').textContent=s.jaw>.5?'Mở':'Đang kẹp';
  $('doorStatus').textContent=s.door>.86?'Mở':s.door<.14?'Đóng':'Đang trượt';
  $('viseStatus').textContent=s.vise>.9?'Đã kẹp':s.vise<.1?'Mở':'Đang chuyển';
  $('cncStatus').textContent=s.spindle?'Đang gia công':(simStep>=6?'Đã gia công':simStep===4?'Đang cấp phôi':'Sẵn sàng');
  $('cncDetail').textContent=s.spindle?'Trục chính hoạt động · GIẢ LẬP':(s.door>.8?'Cửa mở · máy dừng':'Cửa đóng · máy dừng');
  const interlockOK=!(s.spindle&&s.door>.13) && !(simStep===4&&stepProgress>.75&&s.door<.85);
  $('safetyStatus').textContent=interlockOK?'Đạt (minh họa)':'Dừng mô phỏng';
  $('safetyStatus').className=interlockOK?'ok':'';
  if(!interlockOK)playing=false;
}
function updateSteps(){
  STEPS.forEach((step,i)=>{
    const el=STEP_LIST.children[i];if(!el)return;
    el.classList.toggle('active',i===simStep);
    el.classList.toggle('done',i<simStep);
    el.setAttribute('aria-current',i===simStep?'step':'false');
    el.querySelector('.step-index').textContent=i<simStep?'✓':String(i+1).padStart(2,'0');
  });
  $('phaseTitle').textContent=String(simStep+1).padStart(2,'0')+' · '+STEPS[simStep].title;
  $('phaseDescription').textContent=STEPS[simStep].desc;
  $('stepCounter').textContent=String(simStep+1).padStart(2,'0')+' / 08';
}
function sec(t){const m=Math.floor(t/60),s=Math.floor(t%60);return String(m).padStart(2,'0')+':'+String(s).padStart(2,'0');}
function updateProgress(){
  const used=pathTimes[simStep]+stepProgress*STEPS[simStep].duration;
  $('elapsedTime').textContent=sec(used);
  $('progressFill').style.width=(100*used/totalDuration).toFixed(2)+'%';
  $('progressTrack').setAttribute('aria-valuenow',Math.round(used/totalDuration*100));
}
function logEvent(message,kind='success'){
  const row=document.createElement('div');row.className='event-row'+(kind==='warn'?' warn':'');
  const time=document.createElement('time');time.textContent=sec(pathTimes[simStep]+stepProgress*STEPS[simStep].duration);
  const text=document.createElement('span'),em=document.createElement('em');em.textContent=kind==='warn'?'!':'●';
  text.append(em,document.createTextNode(' '+message));row.append(time,text);
  $('eventLog').prepend(row);while($('eventLog').children.length>20)$('eventLog').lastChild.remove();
}
function goStep(i,retainPlay=true){
  simStep=Math.min(Math.max(0,i),STEPS.length-1);stepProgress=0;
  if(!retainPlay)playing=false;
  updateSteps();updateProgress();buildPath(simStep);drawState();
  logEvent(STEPS[simStep].title+' · '+STEPS[simStep].skill);
  syncButton();
}
function advance(delta){
  let t=delta*speed;
  while(t>0&&playing){
    const remain=(1-stepProgress)*STEPS[simStep].duration;
    if(t<remain){stepProgress+=t/STEPS[simStep].duration;t=0;break;}
    t-=remain;
    if(simStep===STEPS.length-1){
      stepProgress=1;cycles++;$('completedCount').textContent=String(cycles).padStart(2,'0');
      logEvent('Hoàn tất chu trình; sản phẩm ở khay đầu ra.');
      if($('loopCycle').checked){goStep(0);t=Math.min(t,totalDuration*2);}else{playing=false;t=0;}
    }else{goStep(simStep+1);}
  }
  updateProgress();drawState();syncButton();
}
function syncButton(){
  $('playBtn').innerHTML=playing?'Ⅱ <span>TẠM DỪNG</span>':'▶ <span>'+(simStep===7&&stepProgress===1?'CHẠY LẠI':'CHẠY MÔ PHỎNG')+'</span>';
  $('playBtn').setAttribute('aria-label',playing?'Tạm dừng mô phỏng':'Chạy mô phỏng');
}
function initUI(){
  for(let i=0;i<STEPS.length;i++){
    const step=STEPS[i],el=document.createElement('button');el.type='button';el.className='step-row';
    el.innerHTML='<span class="step-index">'+String(i+1).padStart(2,'0')+'</span><span class="step-text"><strong>'+step.title+'</strong><small>'+step.short+'</small></span><span class="step-state"></span>';
    el.addEventListener('click',()=>goStep(i,false));STEP_LIST.appendChild(el);
  }
  $('totalTime').textContent=sec(totalDuration);$('completedCount').textContent='00';
  $('playBtn').addEventListener('click',()=>{
    if(!playing&&simStep===7&&stepProgress===1)goStep(0);
    playing=!playing;syncButton();if(playing)logEvent('Bắt đầu chạy minh họa');
  });
  $('resetBtn').addEventListener('click',()=>{cycles=0;$('completedCount').textContent='00';goStep(0,false);$('eventLog').replaceChildren();logEvent('Đã khởi động lại mô phỏng');});
  $('nextBtn').addEventListener('click',()=>{if(simStep===7){goStep(0,false);}else{goStep(simStep+1,false);}});
  $('clearLog').addEventListener('click',()=>$('eventLog').replaceChildren());
  $('speed').addEventListener('input',(e)=>{speed=Number(e.target.value);$('speedValue').textContent=speed.toFixed(1)+'×';});
  $('showPath').addEventListener('change',drawState);
  $('showVision').addEventListener('change',drawState);
  document.querySelectorAll('[data-camera]').forEach(btn=>btn.addEventListener('click',()=>{
    document.querySelectorAll('[data-camera]').forEach(b=>b.classList.remove('active'));
    btn.classList.add('active');const key=btn.dataset.camera;
    const presets={iso:{p:[5.1,4.2,6.8],t:[-.55,.88,.12]},top:{p:[-.5,9.2,.15],t:[-.5,.45,.15]},cnc:{p:[3.9,3.0,3.7],t:[.65,1.1,-.46]}};
    camera.position.set(...presets[key].p);controls.target.set(...presets[key].t);controls.update();
    $('cameraLabel').textContent={iso:'CAM-01 / ISOMETRIC',top:'CAM-02 / TOP VIEW',cnc:'CAM-03 / CNC'}[key];
  }));
  updateSteps();updateProgress();syncButton();
}
function init3D(){
  scene=new THREE.Scene();camera=new THREE.PerspectiveCamera(44,1,.1,80);
  camera.position.set(5.1,4.2,6.8);
  renderer=new THREE.WebGLRenderer({antialias:true,alpha:false,powerPreference:'high-performance'});
  renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.25;
  renderer.shadowMap.enabled=true;renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  view.prepend(renderer.domElement);
  controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=.085;
  controls.target.set(-.55,.88,.12);controls.minDistance=3.4;controls.maxDistance=17;
  controls.maxPolarAngle=Math.PI*.495;controls.minPolarAngle=.13;
  buildRoom();makeTable;buildCNC();buildFeed();buildCameraRig();buildRobot();makeWorkpiece();
  function resize(){const w=view.clientWidth,h=view.clientHeight;if(w===0||h===0)return;renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,1.7));renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();}
  resize();new ResizeObserver(resize).observe(view);window.addEventListener('resize',resize);
  view.addEventListener('webglcontextlost',(e)=>{e.preventDefault();playing=false;err.hidden=false;err.textContent='WebGL bị ngắt. Tải lại trang để khôi phục mô phỏng.';});
  controls.update();drawState();
}
function loop(now){
  const dt=lastFrame?Math.min((now-lastFrame)/1000,.08):0;lastFrame=now;
  if(playing)advance(dt);else drawState();
  controls.update();if(renderer)renderer.render(scene,camera);
  fpsFrames++;if(now-fpsAt>1000){$('fpsValue').textContent=Math.round(fpsFrames*1000/(now-fpsAt));fpsFrames=0;fpsAt=now;}
  requestAnimationFrame(loop);
}
try{
  initUI();init3D();goStep(0,false);
  window.__omts3dReady=true;requestAnimationFrame(loop);
}catch(error){
  console.error('[OMTS VI demo] Could not initialize 3D scene:',error);
  err.hidden=false;err.textContent='Không thể khởi tạo WebGL 3D: '+(error?.message||'Lỗi không xác định')+'. Hãy thử trình duyệt hỗ trợ WebGL.';
}
