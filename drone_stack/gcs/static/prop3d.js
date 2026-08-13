/* Live 3D drone widget - a WebGL viewport that mirrors the real aircraft.
 *
 * This is purely visual (no physics). Two things are driven by live telemetry
 * pushed from app.js via window.setPropTelemetry(t):
 *   1. prop spin speed  - scales with armed state + ground speed
 *   2. body attitude     - roll / pitch / heading are applied to the model so
 *                          the on-screen drone banks and yaws exactly like the
 *                          physical one reported by the Pixhawk.
 *
 * IMPORTANT (why this file is careful about lighting): the GLB's materials are
 * near-black carbon / dark metal with NO textures (baseColor ~0.02-0.24). With
 * three.js physically-correct lighting, plain lights leave that reading as a
 * flat BLACK BLOB / black box. The fix that actually reveals a dark PBR body is
 * a bright environment map (image-based lighting) plus a rim/back light for
 * edge highlights, and a boosted envMapIntensity per material. The canvas
 * itself is kept fully transparent (alpha) so it never paints a black
 * rectangle over the map - only the lit drone is visible, the frosted-glass
 * backdrop behind it is pure CSS (#prop3d-glass). */
import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const HOST_ID = "prop3d-canvas-host";
const GLB_URL = "/static/models/motorpropsjoinednew.glb";
const PROP_NAMES = ["front_left", "front_right", "back_left", "back_right"];

const IDLE_SPEED = 16.0; // rad/s once armed, even at zero ground speed
const SPEED_GAIN = 6.0; // additional rad/s per m/s of ground speed
const MAX_SPEED = 60.0; // clamp so high speed doesn't look absurd
const DEG = Math.PI / 180;

// ---- STATIC MODEL ORIENTATION (the "it keeps flipping" fix) --------------
// This GLB is a Shapr3D CAD export, which is authored Z-UP. Three.js is Y-UP.
// That mismatch is why past roll-only tweaks never fixed "upside down" - a roll
// (Z) rotation just spun the model without ever standing it upright. The real
// correction is -90 deg about X, which maps CAD +Z (up) onto Three +Y (up).
//   ORIENT_X : -90  -> Z-up CAD  ->  Y-up render   (upright)
//   ORIENT_Y :  90  -> heading/nose-forward yaw    (nose knob)
//   ORIENT_Z :   0  -> roll (leave 0)
// All three are overridable LIVE from the URL with no code edit / redeploy,
// e.g.  ...:8090/?ox=-90&oy=90&oz=0  - so orientation can be dialled in from
// the browser instead of round-tripping this file every time.
const _oq = new URLSearchParams(location.search);
const _ori = (name, def) => {
  const v = _oq.get(name);
  return (v == null || v === "" ? def : Number(v)) * DEG;
};
const ORIENT_X = _ori("ox", 0);
const ORIENT_Y = _ori("oy", 90);
const ORIENT_Z = _ori("oz", 0);
const HEADING_OFFSET = ORIENT_Y; // kept for the rest of the file

const propMeshes = []; // [{ mesh, dir }]
let targetSpeed = 0; // rad/s, updated by telemetry

// Desired vs. shown attitude (radians). We lerp shown -> target every frame so
// noisy telemetry looks like smooth, damped aircraft motion rather than jitter.
const target = { roll: 0, pitch: 0, yaw: 0 };
let modelGroup = null; // the yaw/pitch/roll pivot that holds the loaded model

function init() {
  const host = document.getElementById(HOST_ID);
  if (!host) return;

  const width = host.clientWidth || 240;
  const height = host.clientHeight || 220;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(32, width / height, 0.01, 100);
  camera.position.set(0.45, 0.32, 0.55);
  camera.lookAt(0, 0, 0);

  // ---- self-computing camera framing (why the model no longer clips) ----
  // We fit the model's bounding SPHERE (centred at the origin, rotation-
  // invariant) rather than sitting the camera at a hand-tuned distance. The
  // required distance is derived from the sphere radius and the SMALLER of the
  // vertical/horizontal FOV, with margin, so the drone stays fully inside the
  // panel at ANY attitude (roll/pitch/yaw) AND any panel aspect ratio. Recomputed
  // on resize because the binding FOV flips when the panel goes tall vs wide.
  const CAM_DIR = new THREE.Vector3(0.45, 0.32, 0.55).normalize();
  const FRAME_MARGIN = 1.06; // breathing room around the fitted sphere
  let fitRadius = 0.5; // normalised bounding-sphere radius; set exactly after load
  function frameCamera() {
    const vFov = camera.fov * DEG;
    const hFov = 2 * Math.atan(Math.tan(vFov / 2) * camera.aspect);
    const fov = Math.min(vFov, hFov);
    const dist = (fitRadius / Math.sin(fov / 2)) * FRAME_MARGIN;
    camera.position.copy(CAM_DIR).multiplyScalar(dist);
    camera.lookAt(0, 0, 0);
    camera.updateProjectionMatrix();
  }

  const renderer = new THREE.WebGLRenderer({
    antialias: true,
    alpha: true, // transparent framebuffer -> never a black box over the map
    premultipliedAlpha: false,
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(width, height);
  renderer.setClearColor(0x000000, 0); // fully transparent
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.35;
  host.appendChild(renderer.domElement);

  // ---- image-based lighting: what makes the dark body actually visible ----
  // A vertical gradient env (bright sky -> mid ground) gives the metal
  // something to reflect so edges and curvature read instead of a black blob.
  const pmrem = new THREE.PMREMGenerator(renderer);
  const envTex = pmrem.fromScene(makeGradientEnv(), 0.0, 0.1, 100).texture;
  scene.environment = envTex;

  scene.add(new THREE.AmbientLight(0xffffff, 1.2));
  const hemi = new THREE.HemisphereLight(0xbfdfff, 0x223044, 2.2);
  scene.add(hemi);
  const key = new THREE.DirectionalLight(0xffffff, 3.5);
  key.position.set(1.5, 2.5, 1.2);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xbcd4ff, 1.6);
  fill.position.set(-1.5, 0.6, -1.0);
  scene.add(fill);
  // Cool rim/back light: throws a bright edge highlight so the silhouette
  // separates cleanly from the blurred map behind it.
  const rim = new THREE.DirectionalLight(0x8fd0ff, 3.0);
  rim.position.set(-0.5, 1.0, -2.0);
  scene.add(rim);

  new GLTFLoader().load(
    GLB_URL,
    (gltf) => {
      const model = gltf.scene;

      // One-time name dump so the Shapr3D/Blender export naming is verifiable.
      const names = [];
      model.traverse((obj) => names.push(obj.name || "(unnamed)"));
      console.log("[prop3d] GLB object names:", names);

      // Boost reflectivity of the dark PBR materials so IBL reads on them.
      model.traverse((obj) => {
        if (!obj.isMesh || !obj.material) return;
        const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
        mats.forEach((m) => {
          if ("envMapIntensity" in m) m.envMapIntensity = 2.4;
          m.needsUpdate = true;
        });
      });

      let cw = true;
      PROP_NAMES.forEach((name) => {
        const obj = model.getObjectByName(name);
        if (obj) {
          // Adjacent rotors on a quad spin opposite ways (torque cancellation).
          propMeshes.push({ mesh: obj, dir: cw ? 1 : -1 });
          cw = !cw;
        } else {
          console.warn(`[prop3d] prop "${name}" not found in GLB`);
        }
      });

      // Center + normalise the model regardless of its export scale/origin,
      // then parent it under a pivot group we can rotate for attitude.
      // This GLB is a Shapr3D CAD export in MILLIMETRES (bounding diagonal
      // ~774 units). The camera far plane is 100, so pushing the camera out to
      // CAD-scale distance (radius*1.5 ~ 1160) drops the entire model outside
      // the view frustum -> it renders as NOTHING while the frosted panel still
      // shows. So we scale the model down to ~unit size and keep the camera at
      // a fixed, safe distance. This also makes the widget immune to whatever
      // units a future model happens to be exported in.
      // Apply the fixed heading offset BEFORE measuring the bounding box. The
      // GLB geometric centre is NOT at its local origin, so rotating it *after*
      // the recenter swings the whole model off-centre and out of the viewport
      // ("outside the box"). Rotate first, then recentre the rotated result so
      // it stays framed for ANY offset value.
      // Apply the full static orientation (see ORIENT_* at top of file).
      // Order YXZ: first stand the CAD Z-up mesh upright (X), then heading (Y),
      // then any roll (Z). Baked into the static model, NOT the attitude pivot,
      // so live roll/pitch/yaw still read correct.
      model.rotation.order = "YXZ";
      model.rotation.set(ORIENT_X, ORIENT_Y, ORIENT_Z);

      const box = new THREE.Box3().setFromObject(model);
      const center = box.getCenter(new THREE.Vector3());
      const radius = box.getSize(new THREE.Vector3()).length() || 1;
      model.position.sub(center); // recenter the already-rotated model on origin

      modelGroup = new THREE.Group();
      modelGroup.rotation.order = "YXZ"; // yaw, then pitch, then roll
      modelGroup.scale.setScalar(1 / radius); // normalise: radius -> ~1 unit
      modelGroup.add(model);
      scene.add(modelGroup);

      // Measure the ACTUAL normalised bounding-sphere radius (don't assume the
      // 1/diagonal scaling lands exactly on 0.5) and frame from it, so the whole
      // model is guaranteed inside the panel for any export and any attitude.
      modelGroup.updateMatrixWorld(true);
      fitRadius =
        new THREE.Box3()
          .setFromObject(modelGroup)
          .getBoundingSphere(new THREE.Sphere()).radius || 0.5;
      frameCamera();
    },
    undefined,
    // On failure we log loudly but paint NOTHING - the transparent canvas keeps
    // the map visible instead of leaving a black box.
    (err) => console.error("[prop3d] failed to load GLB:", err)
  );

  // Keep the viewport crisp if the panel is resized (theme/layout changes).
  const ro = new ResizeObserver(() => {
    const w = host.clientWidth || width;
    const h = host.clientHeight || height;
    if (!w || !h) return;
    camera.aspect = w / h;
    renderer.setSize(w, h);
    frameCamera(); // re-fit: the binding FOV changes with aspect
  });
  ro.observe(host);

  const clock = new THREE.Clock();
  (function animate() {
    requestAnimationFrame(animate);
    const dt = clock.getDelta();

    propMeshes.forEach(({ mesh, dir }) => {
      mesh.rotation.y += dir * targetSpeed * dt;
    });

    if (modelGroup) {
      // Critically-damped-ish smoothing toward the reported attitude.
      const k = Math.min(1, dt * 6);
      modelGroup.rotation.x += (target.pitch - modelGroup.rotation.x) * k;
      modelGroup.rotation.z += (target.roll - modelGroup.rotation.z) * k;
      modelGroup.rotation.y = lerpAngle(modelGroup.rotation.y, target.yaw, k);
    }

    renderer.render(scene, camera);
  })();
}

// Neutral vertical-gradient environment (sky brighter than ground). No colored
// studio walls, so it lights/reflects without tinting the model's true color.
function makeGradientEnv() {
  const env = new THREE.Scene();
  const geo = new THREE.SphereGeometry(50, 32, 16);
  const mat = new THREE.ShaderMaterial({
    side: THREE.BackSide,
    uniforms: {
      top: { value: new THREE.Color(0xdfeeff) },
      bottom: { value: new THREE.Color(0x2a3440) },
    },
    vertexShader: `varying vec3 vP; void main(){ vP = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0);} `,
    fragmentShader: `varying vec3 vP; uniform vec3 top; uniform vec3 bottom;
      void main(){ float h = normalize(vP).y * 0.5 + 0.5; gl_FragColor = vec4(mix(bottom, top, h), 1.0);} `,
  });
  env.add(new THREE.Mesh(geo, mat));
  return env;
}

// Shortest-path angular interpolation so yaw doesn't unwrap the long way round.
function lerpAngle(a, b, t) {
  let d = ((b - a + Math.PI) % (2 * Math.PI)) - Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  return a + d * t;
}

function propAngularSpeed(armed) {
  // Props spin at full speed whenever the aircraft is armed, and stop when
  // disarmed. Ground speed no longer modulates the visual spin rate.
  return armed ? MAX_SPEED : 0;
}

/* Called from app.js (a classic script) on every telemetry frame - the bridge
 * from the websocket-driven dashboard into this ES module. Accepts the whole
 * telemetry object; tolerates the older (armed, groundSpeed) call signature. */
window.setPropTelemetry = function setPropTelemetry(t, groundSpeedMs) {
  let armed, gs, roll, pitch, heading;
  if (t && typeof t === "object") {
    armed = t.armed;
    gs = t.ground_speed;
    roll = t.roll;
    pitch = t.pitch;
    heading = t.heading;
  } else {
    armed = t;
    gs = groundSpeedMs;
  }
  targetSpeed = propAngularSpeed(armed, gs);
  // Map aircraft angles (degrees) onto the model pivot. Signs chosen so the
  // on-screen drone banks/pitches in the same direction the real one does.
  if (roll != null) target.roll = -Number(roll) * DEG;
  if (pitch != null) target.pitch = Number(pitch) * DEG;
  if (heading != null) target.yaw = -Number(heading) * DEG;
};

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", init);
} else {
  init();
}
