/* Drone GCS front-end. Single WebSocket -> render every panel. */
"use strict";

let ws = null, latest = {}, netLatency = 0, seeded = false, wpMode = false;
let localWps = [];            // [{lat,lon,alt}]  (map-editable mission)
let lastConsoleId = 0;
let seededConsole = false;    // suppress toasts for the console backlog on first load
let radarMax = 12;            // radar range in metres (zoomable)
let dlvTargetMarker = null, dlvRouteLine = null;
let lastDlvPhase = null;      // for phase-change toasts (null = not yet seeded)
let cursorLatLng = null;      // last map cursor position, for "Test order"

/* ---------- NL command history (Up/Down recall, like a shell) ---------- */
const NL_HISTORY_KEY = "aerix_nl_history", NL_HISTORY_MAX = 50;
let nlHistory = [];
try { nlHistory = JSON.parse(localStorage.getItem(NL_HISTORY_KEY)) || []; } catch { nlHistory = []; }
let nlHistoryIdx = nlHistory.length;  // one past the newest entry == "not browsing"
let nlDraft = "";                     // what the user was typing before they pressed Up

/* ---------- WebSocket ---------- */
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.onclose = () => setTimeout(connect, 1000);
  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    if (d.pong !== undefined) { netLatency = Math.round(performance.now() - d.pong); return; }
    if (d.ack !== undefined) { onAck(d.ack, d.result); return; }
    latest = d; render(d);
  };
}
function send(cmd, params) {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({ cmd, params: params || {} }));
}
function onAck(cmd, res) {
  if (cmd === "nl") {
    nlLog((res.ok ? "✓ " : "✗ ") + (res.message || "no response"), res.ok);
    const actions = (res.data && res.data.actions) || [];
    actions.forEach((a) => nlLog("   • " + (a.intent || "") + " → " + a.message, a.ok));
  }
  if (cmd === "record") {
    $("btn-record").textContent = res.recording ? "Stop Recording" : "Record Log";
    $("btn-record").classList.toggle("danger", !!res.recording);
    if (!res.recording) loadLogList();
  }
}
setInterval(() => { if (ws && ws.readyState === 1) ws.send(JSON.stringify({ cmd: "ping", t: performance.now() })); }, 2000);

/* ---------- helpers ---------- */
const $ = (id) => document.getElementById(id);
const fmt = (x, d = 1) => (x === undefined || x === null) ? "--" : Number(x).toFixed(d);
function setPill(el, text, cls) { el.textContent = text; el.className = "pill " + (cls || ""); }

/* ---------- translucent centre-screen announcement ---------- */
let lastOverride = null;
let lastMode = null;         // last flight mode we announced (null = not yet seeded)
let _toastTimer = null;
function showToast(text, cls) {
  const el = $("mode-toast");
  if (!el) return;
  el.textContent = text;
  el.className = "mode-toast " + (cls || "");
  void el.offsetWidth;                       // restart the fade transition
  el.classList.add("show");
  clearTimeout(_toastTimer);
  _toastTimer = setTimeout(() => el.classList.remove("show"), 2600);
}

/* ---------- theme ---------- */
function initTheme() {
  const saved = localStorage.getItem("gcs-theme") || "dark";
  document.body.dataset.theme = saved;
  $("theme-toggle").textContent = saved === "dark" ? "☀" : "☽";
  $("theme-toggle").onclick = () => {
    const next = document.body.dataset.theme === "dark" ? "light" : "dark";
    document.body.dataset.theme = next;
    localStorage.setItem("gcs-theme", next);
    $("theme-toggle").textContent = next === "dark" ? "☀" : "☽";
  };
}

/* ---------- flight modes ---------- */
const MODES = [
  ["STABILIZE", "STABILIZE"], ["ALT HOLD", "ALT_HOLD"], ["LOITER", "LOITER"], ["POSHOLD", "POSHOLD"],
  ["GUIDED", "GUIDED"], ["AUTO", "AUTO"], ["RTL", "RTL"], ["SMART RTL", "SMART_RTL"],
  ["LAND", "LAND"], ["BRAKE", "BRAKE"], ["CIRCLE", "CIRCLE"], ["ACRO", "ACRO"],
  ["AUTOROTATE", "AUTOROTATE"], ["AUTOTUNE", "AUTOTUNE"], ["AUTO RTL", "AUTO_RTL"], ["AVOID ADSB", "AVOID_ADSB"],
  ["DRIFT", "DRIFT"], ["FLIP", "FLIP"], ["FLOWHOLD", "FLOWHOLD"], ["FOLLOW", "FOLLOW"],
  ["GUIDED NOGPS", "GUIDED_NOGPS"], ["OF LOITER", "OF_LOITER"], ["POSITION", "POSITION"], ["SPORT", "SPORT"],
  ["SYSTEMID", "SYSTEMID"], ["THROW", "THROW"], ["ZIGZAG", "ZIGZAG"],
];
function buildModeGrid() {
  const g = $("mode-grid");
  MODES.forEach(([label, mode]) => {
    const b = document.createElement("button");
    b.textContent = label; b.dataset.mode = mode;
    b.onclick = () => send("set_mode", { mode });
    g.appendChild(b);
  });
}

/* ---------- lidar field of view ---------- */
// The LiDAR's front face is the 0-degree reference: straight up on the radar,
// pointing at the nose of the airframe. `fov_deg` (250) is the arc actually
// scanned - 125 deg left + 125 deg right - and the rest, the wedge directly
// behind, is blanked in the driver so nothing back there ever reaches obstacle
// avoidance. Bearings here are the radar's own convention: 0 = front,
// positive clockwise (to the right).
const DEFAULT_LIDAR_FOV = { enabled: true, fov_deg: 250, half_deg: 125, blind_deg: 110 };

function fovRay(cx, cy, bearing, r) {
  const rad = (bearing - 90) * Math.PI / 180;
  return [cx + Math.cos(rad) * r, cy + Math.sin(rad) * r];
}

// Ignored rear wedge + the two FOV boundary spokes. Drawn UNDER the returns.
function drawFovMask(ctx, cx, cy, R, fov) {
  if (!fov.enabled || !(fov.blind_deg > 0.01)) return;
  const half = fov.half_deg;
  const a0 = (half - 90) * Math.PI / 180;          // +135 deg -> back-right
  const a1 = (360 - half - 90) * Math.PI / 180;    // -135 deg -> back-left

  ctx.save();
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.arc(cx, cy, R, a0, a1);
  ctx.closePath();
  ctx.fillStyle = "rgba(255,77,79,0.08)";
  ctx.fill();
  ctx.clip();
  ctx.strokeStyle = "rgba(255,77,79,0.18)";
  ctx.lineWidth = 1;
  for (let k = -2 * R; k < 2 * R; k += 10) {       // 45-deg hatching
    ctx.beginPath(); ctx.moveTo(cx + k, cy - R); ctx.lineTo(cx + k + 2 * R, cy + R); ctx.stroke();
  }
  ctx.restore();

  ctx.save();
  ctx.strokeStyle = "rgba(255,77,79,0.6)";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 4]);
  [half, 360 - half].forEach((b) => {
    const [x, y] = fovRay(cx, cy, b, R);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke();
  });
  ctx.restore();

  // FOV edge labels, pulled inside the ring so they clear the 45-deg spokes.
  ctx.save();
  ctx.font = "10px monospace";
  ctx.fillStyle = "#ff9a9a";
  ctx.textAlign = "center";
  const [rx, ry] = fovRay(cx, cy, half, R * 0.62);
  const [lx, ly] = fovRay(cx, cy, -half, R * 0.62);
  ctx.fillText("+" + fmt(half, 0) + "\u00b0", rx, ry);
  ctx.fillText("-" + fmt(half, 0) + "\u00b0", lx, ly);
  const [bx, by] = fovRay(cx, cy, 180, R * 0.55);
  ctx.fillText("IGNORED " + fmt(fov.blind_deg, 0) + "\u00b0", bx, by);
  ctx.restore();
}

// 0-degree front reference line + the active-sweep arc. Drawn OVER the returns
// so the mark stays readable in a dense scan.
function drawFovFront(ctx, cx, cy, R, fov) {
  const half = fov.enabled ? fov.half_deg : 180;
  ctx.save();

  // active sweep arc across the scanned window
  ctx.strokeStyle = "rgba(56,224,123,0.45)";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(cx, cy, R, (-half - 90) * Math.PI / 180, (half - 90) * Math.PI / 180);
  ctx.stroke();

  // the front line itself: centre -> rim, straight up = LiDAR front = 0 deg
  ctx.strokeStyle = "#38e07b";
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - R); ctx.stroke();
  ctx.fillStyle = "#38e07b";
  ctx.beginPath();                                  // arrow head at the rim
  ctx.moveTo(cx, cy - R - 7);
  ctx.lineTo(cx - 5, cy - R + 3);
  ctx.lineTo(cx + 5, cy - R + 3);
  ctx.closePath(); ctx.fill();

  ctx.font = "bold 10px monospace";
  ctx.textAlign = "center";
  ctx.fillText("LIDAR FRONT 0\u00b0", cx, cy - R * 0.55);

  ctx.textAlign = "left";
  ctx.font = "9px monospace";
  ctx.fillStyle = fov.enabled ? "#7fd8a5" : "#3a5566";
  const caption = fov.enabled
    ? "FOV " + fmt(fov.fov_deg, 0) + "\u00b0 \u00b7 REAR " + fmt(fov.blind_deg, 0) + "\u00b0 MASKED"
    : "FOV 360\u00b0 \u00b7 no mask";
  ctx.fillText(caption, 6, ctx.canvas.height - 6);
  ctx.restore();
}

/* ---------- radar ---------- */
function drawRadar(d) {
  const c = $("radar"), ctx = c.getContext("2d");
  const W = c.width, H = c.height, cx = W / 2, cy = H / 2;
  const maxR = radarMax, R = Math.min(cx, cy) - 16, scale = R / maxR;
  ctx.clearRect(0, 0, W, H);
  const step = maxR <= 6 ? 1 : (maxR <= 12 ? 2 : 4);
  ctx.strokeStyle = "#15303a"; ctx.fillStyle = "#3a5566"; ctx.font = "9px monospace";
  for (let m = step; m <= maxR; m += step) {
    ctx.beginPath(); ctx.arc(cx, cy, m * scale, 0, 7); ctx.stroke();
    ctx.fillText(m + "m", cx + 2, cy - m * scale + 9);
  }
  for (let a = 0; a < 360; a += 45) {
    const rad = (a - 90) * Math.PI / 180;
    const x1 = cx + Math.cos(rad) * R, y1 = cy + Math.sin(rad) * R;
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(x1, y1);
    ctx.strokeStyle = "#0f2029"; ctx.stroke();
    ctx.fillStyle = "#3a5566";
    ctx.fillText(a + "°", cx + Math.cos(rad) * (R + 8) - 6, cy + Math.sin(rad) * (R + 8) + 3);
  }
  const fov = d.lidar_fov || DEFAULT_LIDAR_FOV;
  drawFovMask(ctx, cx, cy, R, fov);
  const toScreen = (fwd, left) => [cx - left * scale, cy - fwd * scale];
  const scan = d.scan;
  if (scan && scan.ranges) {
    ctx.fillStyle = "#38e07b";
    const a0 = scan.angle_min, inc = scan.angle_increment;
    for (let i = 0; i < scan.ranges.length; i++) {
      const r = scan.ranges[i];
      if (r === null || !isFinite(r)) continue;
      const a = a0 + i * inc;
      const [px, py] = toScreen(r * Math.cos(a), r * Math.sin(a));
      ctx.fillRect(px - 1, py - 1, 2, 2);
    }
  }
  let nearest = null;
  (d.obstacles || []).forEach((o) => {
    if (nearest === null || o.distance < nearest) nearest = o.distance;
    const [px, py] = toScreen(o.x, o.y);
    ctx.strokeStyle = o.danger ? "#ff4d4f" : "#f2b34a";
    ctx.beginPath(); ctx.arc(px, py, 8, 0, 7); ctx.stroke();
    ctx.fillStyle = o.danger ? "#ff9a9a" : "#ffd9a0";
    ctx.fillText(`${o.cls} ${fmt(o.distance, 2)}m`, px + 9, py + 3);
  });
  ctx.fillStyle = "#e6f0f5";
  ctx.beginPath(); ctx.moveTo(cx, cy - 7); ctx.lineTo(cx - 5, cy + 6); ctx.lineTo(cx + 5, cy + 6); ctx.closePath(); ctx.fill();

  drawFovFront(ctx, cx, cy, R, fov);

  const av = d.avoidance || {};
  $("radar-title").textContent = (d.source || "SIM") + " · avoid";
  $("radar-state").textContent = av.status || "--";
  $("radar-near").textContent = nearest === null ? "clear" : ("near " + fmt(nearest, 1) + "m");
  const mult = av.status === "BRAKE" ? "0.0" : (av.status === "SLOW" ? "0.4" : "1.0");
  $("radar-mult").textContent = mult;
  $("brake-overlay").classList.toggle("show", av.status === "BRAKE");
  updateZoomLabel();
}
function updateZoomLabel() { $("rz-lvl").textContent = (12 / radarMax).toFixed(1) + "x"; }
function zoom(delta) {
  radarMax = Math.min(40, Math.max(2, radarMax + delta));
  updateZoomLabel();
  if (latest.scan !== undefined) drawRadar(latest);
}
function setupRadarControls() {
  $("rz-in").onclick = () => zoom(-2);
  $("rz-out").onclick = () => zoom(+2);
  $("rz-exp").onclick = () => {
    const on = document.body.classList.toggle("radar-expanded");
    const c = $("radar"); c.width = c.height = on ? 640 : 360;
    if (latest.scan !== undefined) drawRadar(latest);
  };
  // scroll-wheel zoom on the radar
  $("radar").addEventListener("wheel", (e) => {
    e.preventDefault();
    zoom(e.deltaY < 0 ? -1 : +1);
  }, { passive: false });
  updateZoomLabel();
}

/* ---------- safety confirm modal ---------- */
function confirmDanger(title, msg, onYes) {
  $("confirm-title").textContent = title;
  $("confirm-msg").textContent = msg;
  const ov = $("confirm-overlay"); ov.classList.add("show");
  const ok = $("confirm-ok"), cancel = $("confirm-cancel");
  const close = () => { ov.classList.remove("show"); ok.onclick = cancel.onclick = null; };
  ok.onclick = () => { close(); onYes(); };
  cancel.onclick = close;
  ov.onclick = (e) => { if (e.target === ov) close(); };
}

/* ---------- map ---------- */
let map, layers = {}, droneMarker, trailLine, wpLine, homeMarker, wpMarkers = [];
function initMap() {
  map = L.map("map", { zoomControl: false, attributionControl: false }).setView([12.9017, 77.654], 18);
  layers.dark = L.tileLayer("https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png", { maxZoom: 20 });
  layers.street = L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", { maxZoom: 19 });
  layers.sat = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", { maxZoom: 20 });
  layers.terrain = L.tileLayer("https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png", { maxZoom: 17 });
  layers.sat.addTo(map);
  trailLine = L.polyline([], { color: "#38e07b", weight: 2, opacity: 0.8 }).addTo(map);
  wpLine = L.polyline([], { color: "#22d3ee", weight: 1.5, dashArray: "5,5" }).addTo(map);
  // The delivery route is drawn separately from the editable mission line so a
  // Firebase order is visually distinct from waypoints someone dropped by hand.
  dlvRouteLine = L.polyline([], { color: "#22d3ee", weight: 2.5, opacity: 0.9 }).addTo(map);
  const arrow = L.divIcon({ className: "drone-icon", html: '<span class="drone-rot">▲</span>', iconSize: [20, 20] });
  droneMarker = L.marker([12.9017, 77.654], { icon: arrow }).addTo(map);

  document.querySelectorAll(".lyr").forEach((b) => b.onclick = () => {
    Object.values(layers).forEach((l) => map.removeLayer(l));
    layers[b.dataset.layer].addTo(map);
    document.querySelectorAll(".lyr").forEach((x) => x.classList.remove("on"));
    b.classList.add("on");
  });
  map.on("mousemove", (e) => {
    cursorLatLng = e.latlng;
    $("coord-cursor").textContent = `${e.latlng.lat.toFixed(6)}, ${e.latlng.lng.toFixed(6)}`;
  });
  map.on("click", (e) => { if (wpMode) addLocalWp(e.latlng.lat, e.latlng.lng); });
  $("tb-wp").onclick = () => { wpMode = !wpMode; $("tb-wp").classList.toggle("on", wpMode); };
  $("tb-clear").onclick = () => { localWps = []; renderWps(); send("clear_mission"); };
  $("tb-upload").onclick = uploadMission;
  $("tb-download").onclick = downloadPlan;
  $("ms-save").onclick = downloadPlan;
  $("ms-load").onclick = () => $("plan-file").click();
}
function addLocalWp(lat, lon) { localWps.push({ lat, lon, alt: 3 }); renderWps(); uploadMission(); }
function renderWps() {
  wpMarkers.forEach((m) => map.removeLayer(m));
  wpMarkers = [];
  localWps.forEach((w, i) => {
    const icon = L.divIcon({ className: "", html: `<div class="wp-icon"></div>`, iconSize: [12, 12] });
    const m = L.marker([w.lat, w.lon], { icon, draggable: true }).addTo(map);
    m.bindTooltip("WP" + (i + 1), { permanent: false });
    m.on("dragend", (e) => { const p = e.target.getLatLng(); localWps[i].lat = p.lat; localWps[i].lon = p.lng; renderWps(); uploadMission(); });
    wpMarkers.push(m);
  });
  wpLine.setLatLngs(localWps.map((w) => [w.lat, w.lon]));
}
function uploadMission() { if (localWps.length) send("upload_mission", { waypoints: localWps }); }
function updateMap(d) {
  const p = d.position || {};
  const fix = (d.telemetry || {}).gps_fix;
  const gpsValid = (fix === "2D" || fix === "3D" || fix === "RTK") && p.lat && p.lon;
  if (gpsValid) {
    droneMarker.setLatLng([p.lat, p.lon]);
    // rotate only the inner glyph (never the Leaflet-positioned outer icon)
    const rot = droneMarker._icon && droneMarker._icon.querySelector(".drone-rot");
    if (rot) rot.style.transform = `rotate(${p.heading || 0}deg)`;
    $("coord-drone").textContent = `${p.lat.toFixed(6)}, ${p.lon.toFixed(6)}  ${fmt(p.alt, 1)}m`;
    if (!map._centered) { map.setView([p.lat, p.lon], 18); map._centered = true; }
  } else {
    $("coord-drone").textContent = "NO GPS FIX";
  }
  if (gpsValid && p.home_lat && !homeMarker) {
    homeMarker = L.marker([p.home_lat, p.home_lon],
      { icon: L.divIcon({ className: "", html: "⌂", iconSize: [16, 16] }) }).addTo(map);
  }
  // clear a stale home marker if the source/home was reset
  if (!p.home_lat && homeMarker) { map.removeLayer(homeMarker); homeMarker = null; }
  trailLine.setLatLngs((d.trail || []).map((t) => [t[0], t[1]]));
  if (!seeded && d.mission && d.mission.waypoints && d.mission.waypoints.length) {
    localWps = d.mission.waypoints.filter((w) => w.lat && w.lon).map((w) => ({ lat: w.lat, lon: w.lon, alt: w.alt }));
    renderWps(); seeded = true;
  }
}
/* ---------- delivery (Firebase order -> flight) ----------------------------
   Everything here is a read-out of the delivery node's published state: the
   browser holds no delivery state of its own, so a reload or a second operator
   opening the dashboard sees exactly the same thing. The only outbound calls
   are the four operator decisions (accept / decline / abort / auto-accept).   */
const DLV_LABEL = {
  IDLE: "IDLE", PENDING: "ORDER PENDING", REJECTED: "REJECTED",
  ACCEPTED: "ACCEPTED", ENROUTE: "EN ROUTE", HOVERING: "HOVERING",
  RETURNING: "RETURNING", LANDED: "DELIVERED", ABORTED: "ABORTED",
};
const DLV_TOAST = {
  PENDING: ["NEW DELIVERY ORDER", "amber"],
  ACCEPTED: ["ORDER ACCEPTED — LAUNCHING", "ok"],
  ENROUTE: ["EN ROUTE TO DROP POINT", "ok"],
  HOVERING: ["HOLDING OVER DROP POINT", "ok"],
  RETURNING: ["RETURNING HOME", "amber"],
  LANDED: ["DELIVERY COMPLETE", "ok"],
  REJECTED: ["ORDER REJECTED", "bad"],
  ABORTED: ["DELIVERY ABORTED", "bad"],
};

/* The flight, as six steps. The timeline is the fastest way to answer "where
   is my order right now" without reading any numbers. */
const DLV_STEPS = ["ORDER", "LAUNCH", "EN ROUTE", "HOVER", "RETURN", "HOME"];
const DLV_STEP_OF = {
  IDLE: -1, PENDING: 0, REJECTED: 0, ACCEPTED: 1,
  ENROUTE: 2, HOVERING: 3, RETURNING: 4, LANDED: 5, ABORTED: -2,
};

function dlvAge(iso) {
  if (!iso) return "";
  const t = Date.parse(iso);
  if (isNaN(t)) return "";
  const s = Math.max(0, (Date.now() - t) / 1000);
  if (s < 60) return Math.round(s) + "s ago";
  if (s < 3600) return Math.round(s / 60) + "m ago";
  if (s < 86400) return Math.round(s / 3600) + "h ago";
  return Math.round(s / 86400) + "d ago";
}

function dlvEta(v, phase, groundSpeed) {
  // Hover time is known exactly; flight time is only ever an estimate, so it
  // is shown as one rather than dressed up with false precision.
  if (phase === "HOVERING") return fmt(v.hover_remaining_s, 0) + " s hold";
  if (!["ENROUTE", "RETURNING", "ACCEPTED"].includes(phase)) return "--";
  const remaining = v.remaining_m || 0;
  if (!remaining) return "--";
  const speed = groundSpeed > 0.6 ? groundSpeed : 4.0;   // cruise fallback
  let secs = remaining / speed;
  if (phase === "ENROUTE" || phase === "ACCEPTED") secs += (v.hover_seconds || 0);
  return "~" + (secs < 90 ? Math.round(secs) + " s" : Math.round(secs / 60) + " min");
}

function dlvTrack(phase) {
  const at = DLV_STEP_OF[phase] === undefined ? -1 : DLV_STEP_OF[phase];
  const aborted = at === -2;
  const bars = DLV_STEPS.map(function (_, i) {
    let cls = "dlv-step";
    if (aborted) cls += i === 0 ? " bad" : "";
    else if (at > i) cls += " done";
    else if (at === i) cls += " on";
    return '<div class="' + cls + '"></div>';
  }).join("");
  return '<div style="display:flex;gap:3px">' + bars + '</div>'
    + '<div class="dlv-track-labels">'
    + DLV_STEPS.map(function (l) { return "<span>" + l + "</span>"; }).join("")
    + '</div>';
}

function dlvRow(o, selected) {
  // A row is only clickable when the drone could actually fly it; a blocked
  // order still shows, with the reason, rather than being hidden.
  const cls = o.flown ? "done" : (o.dispatchable ? (selected ? "sel" : "") : "blocked");
  const who = o.recipient_id ? " · " + o.recipient_id : "";
  const coords = Number(o.target_lat).toFixed(5) + ", " + Number(o.target_lon).toFixed(5);
  const sub = o.blocked_reason
    ? o.blocked_reason
    : coords + (o.created_at ? " · " + dlvAge(o.created_at) : "");
  const pick = o.dispatchable && !o.flown ? o.order_id : "";
  return '<div class="dlv-row ' + cls + '" data-order="' + pick + '">'
    + '<span class="dlv-row-id">#' + String(o.order_id).slice(0, 14) + who + '</span>'
    + '<span class="dlv-row-dist">' + (o.distance_m ? fmt(o.distance_m, 0) + " m" : "") + '</span>'
    + '<span class="dlv-row-sub">' + sub + '</span>'
    + '</div>';
}

function updateDelivery(d) {
  const v = d.delivery || {};
  const phase = v.phase || "IDLE";
  const lower = phase.toLowerCase();
  const pending = phase === "PENDING";
  const inFlight = ["ACCEPTED", "ENROUTE", "HOVERING", "RETURNING"].includes(phase);
  const orders = v.orders || [], recent = v.recent || [];

  const badge = $("dlv-phase");
  if (badge) { badge.textContent = DLV_LABEL[phase] || phase; badge.className = "dlv-badge " + lower; }
  const hd = $("dlv-hd-phase");
  if (hd) hd.textContent = DLV_LABEL[phase] || phase;
  const link = $("dlv-link");
  if (link) { link.className = "link-dot " + (v.link || "disabled"); link.title = "order source: " + (v.link || "disabled"); }

  const count = $("dlv-count");
  if (count) {
    count.textContent = orders.length;
    count.classList.toggle("has", orders.length > 0);
    count.title = orders.length + " order(s) awaiting dispatch";
  }

  // Setup banner: the one thing that stops orders reaching the drone at all.
  const setup = $("dlv-setup");
  if (setup) {
    const broken = ["no-credentials", "error"].includes(v.link);
    setup.classList.toggle("on", broken);
    if (broken) {
      setup.innerHTML = v.link === "no-credentials"
        ? "Orders from the app cannot be read yet.<br>Install the Firebase key: "
          + "<code>scripts/firebase_setup.py &lt;key.json&gt;</code>"
        : "Order source error: " + (v.last_error || "unknown");
    }
  }

  // Inbox: every order the app has placed that the drone could still fly.
  const inbox = $("dlv-inbox");
  if (inbox) {
    const rows = orders.map(function (o) {
      return dlvRow(o, o.order_id === v.selected_order_id);
    });
    inbox.innerHTML = rows.length
      ? rows.join("")
      : (v.link === "online"
          ? '<div class="dlv-empty">No orders waiting. Place one in the app.</div>'
          : "");
  }

  $("dlv-order").textContent = v.order_id ? "#" + v.order_id : "no order";
  $("dlv-msg").textContent = v.message || "--";
  $("dlv-track").innerHTML = dlvTrack(phase);
  $("dlv-target").textContent = (v.target_lat || v.target_lon)
    ? Number(v.target_lat).toFixed(5) + ", " + Number(v.target_lon).toFixed(5) : "--";
  $("dlv-dist").textContent = v.distance_m ? fmt(v.remaining_m || v.distance_m, 0) + " m" : "--";
  $("dlv-wps").textContent = v.waypoints
    ? v.waypoints + (v.fc_mission_uploaded ? " ✓FC" : "") : "--";
  $("dlv-hover").textContent = phase === "HOVERING"
    ? fmt(v.hover_remaining_s, 0) + " s left"
    : (v.hover_seconds ? fmt(v.hover_seconds, 0) + " s" : "--");
  $("dlv-eta").textContent = dlvEta(v, phase, (d.telemetry || {}).ground_speed || 0);

  // Progress bar: the hover countdown while holding, distance covered while flying.
  const wrap = $("dlv-bar-wrap"), bar = $("dlv-bar");
  if (wrap && bar) {
    let pct = null;
    if (phase === "HOVERING" && v.hover_seconds > 0) {
      pct = 100 * (1 - (v.hover_remaining_s / v.hover_seconds));
    } else if (["ENROUTE", "RETURNING"].includes(phase) && v.distance_m > 0) {
      pct = 100 * (1 - Math.min(1, (v.remaining_m || 0) / v.distance_m));
    }
    wrap.classList.toggle("on", pct !== null);
    if (pct !== null) bar.style.width = Math.max(0, Math.min(100, pct)).toFixed(1) + "%";
  }

  $("dlv-accept").disabled = !pending;
  $("dlv-reject").disabled = !pending;
  $("dlv-abort").disabled = !inFlight;
  $("dlv-auto").classList.toggle("on", !!v.auto_accept);

  updateOrderBook(v, orders, recent, phase);
  updateDeliveryMap(d, v, phase);

  if (lastDlvPhase === null) { lastDlvPhase = phase; return; }   // don't toast on load
  if (phase !== lastDlvPhase) {
    lastDlvPhase = phase;
    const t = DLV_TOAST[phase];
    if (t) showToast(t[0], t[1]);
  }
}

/* The bottom panel is the audit view: every order the app has written, queued
   and historical, so "did my order arrive" is answerable at a glance. */
function updateOrderBook(v, orders, recent, phase) {
  const book = $("dlv-book");
  if (!book) return;
  $("tbl-delivery").innerHTML = "";
  const all = orders.concat(recent);
  if (!all.length) {
    book.innerHTML = '<div class="dlv-empty">'
      + (v.link === "online"
          ? "No orders in Firebase yet."
          : "Order source offline — " + (v.last_error || v.link))
      + '</div>';
    return;
  }
  const rows = all.map(function (o) {
    const live = o.order_id === v.order_id && phase !== "IDLE";
    const status = live ? "flying" : String(o.status || "").toLowerCase();
    const label = live ? (DLV_LABEL[phase] || phase) : (o.status || "--");
    const pick = o.dispatchable && !o.flown;
    // Repeat: re-fly the exact same drop point. Only offered once an order
    // is done with (not live, not already sitting in the dispatch queue) -
    // it re-injects a fresh order_id at the same lat/lon, which still has to
    // go through ACCEPT & FLY like any other order, so this cannot launch
    // anything by itself.
    const canRepeat = !live && !pick && o.target_lat && o.target_lon;
    const repeatBtn = canRepeat
      ? '<button class="repeat-btn" data-repeat="1" data-repeat-id="' + o.order_id
        + '" data-repeat-lat="' + o.target_lat + '" data-repeat-lon="' + o.target_lon
        + '" title="Fly this same drop point again">↻ Repeat</button>'
      : "";
    return '<tr class="' + (live ? "live " : "") + (pick ? "pick" : "") + '"'
      + ' data-order="' + (pick ? o.order_id : "") + '">'
      + "<td>#" + String(o.order_id).slice(0, 12) + "</td>"
      + "<td>" + (o.recipient_id || "--") + "</td>"
      + "<td>" + Number(o.target_lat).toFixed(5) + ", " + Number(o.target_lon).toFixed(5) + "</td>"
      + "<td>" + (o.distance_m ? fmt(o.distance_m, 0) + " m" : "--") + "</td>"
      + "<td>" + (dlvAge(o.created_at) || "--") + "</td>"
      + '<td><span class="pill ' + status + '">' + label + "</span></td>"
      + "<td>" + repeatBtn + "</td>"
      + "</tr>";
  }).join("");
  book.innerHTML = "<table><thead><tr>"
    + "<th>Order</th><th>Recipient</th><th>Target</th><th>Dist</th><th>Placed</th><th>Status</th><th></th>"
    + "</tr></thead><tbody>" + rows + "</tbody></table>";
}

function updateDeliveryMap(d, v, phase) {
  const hasTarget = (v.target_lat || v.target_lon)
    && ["PENDING", "ACCEPTED", "ENROUTE", "HOVERING", "RETURNING"].includes(phase);
  if (!hasTarget) {
    if (dlvTargetMarker) { map.removeLayer(dlvTargetMarker); dlvTargetMarker = null; }
    if (dlvRouteLine) dlvRouteLine.setLatLngs([]);
    return;
  }
  const ll = [v.target_lat, v.target_lon];
  const icon = L.divIcon({
    className: "",
    html: `<div class="dlv-target-icon ${phase === "HOVERING" ? "hovering" : ""}">◎</div>`,
    iconSize: [22, 22], iconAnchor: [11, 11],
  });
  if (!dlvTargetMarker) {
    dlvTargetMarker = L.marker(ll, { icon }).addTo(map);
  } else {
    dlvTargetMarker.setLatLng(ll); dlvTargetMarker.setIcon(icon);
  }
  dlvTargetMarker.bindTooltip(
    `DROP POINT${v.order_id ? " · #" + v.order_id : ""}`, { permanent: false });

  // Draw from wherever the aircraft actually is, so the line shrinks as it flies.
  const p = d.position || {};
  if (p.lat && p.lon) dlvRouteLine.setLatLngs([[p.lat, p.lon], ll]);
  else dlvRouteLine.setLatLngs([ll]);
}

function initDelivery() {
  $("dlv-accept").onclick = () => {
    const v = (latest.delivery || {});
    confirmDanger("ACCEPT DELIVERY & FLY?",
      `The drone will arm, take off, fly ${fmt(v.distance_m, 0)} m to the drop point, `
      + `hold for ${fmt(v.hover_seconds, 0)} s and return home. Ensure the area is clear.`,
      () => send("delivery_accept", { order_id: v.order_id || "" }));
  };
  $("dlv-reject").onclick = () => send("delivery_reject", { reason: "declined by operator" });
  $("dlv-abort").onclick = () => confirmDanger("ABORT DELIVERY?",
    "The drone will stop the delivery and return home immediately.",
    () => send("delivery_abort"));
  $("dlv-auto").onclick = () => {
    const on = !!(latest.delivery || {}).auto_accept;
    if (on) { send("delivery_set_auto", { enabled: false }); return; }
    confirmDanger("ENABLE AUTO-ACCEPT?",
      "Every new order will be flown WITHOUT asking. The drone can arm and take "
      + "off on its own from this point on.",
      () => send("delivery_set_auto", { enabled: true }));
  };
  // Test order: drops an order at the map cursor without touching Firebase, so
  // the whole chain can be demonstrated with no app and no credentials.
  $("dlv-inject").onclick = () => {
    if (!cursorLatLng) { showToast("MOVE THE CURSOR OVER THE MAP FIRST", "amber"); return; }
    const { lat, lng } = cursorLatLng;
    send("delivery_inject", { lat, lon: lng, order_id: "test-" + Date.now() });
  };

  // Ask Firebase now rather than waiting out the poll interval - the first
  // thing anyone does after placing an order in the app is look here.
  const refresh = $("dlv-refresh");
  if (refresh) refresh.onclick = () => {
    refresh.classList.add("spin");
    setTimeout(() => refresh.classList.remove("spin"), 600);
    send("delivery_refresh");
  };

  // Picking an order out of the queue. Delegated, because the rows are
  // re-rendered on every frame and per-row handlers would not survive.
  // Selecting only moves the offer; flying still needs ACCEPT & FLY.
  const pick = (ev) => {
    const repeatBtn = ev.target.closest("[data-repeat]");
    if (repeatBtn) {
      const lat = parseFloat(repeatBtn.getAttribute("data-repeat-lat"));
      const lon = parseFloat(repeatBtn.getAttribute("data-repeat-lon"));
      if (isNaN(lat) || isNaN(lon)) return;
      send("delivery_inject", {
        lat, lon,
        order_id: "repeat-" + repeatBtn.getAttribute("data-repeat-id") + "-" + Date.now(),
      });
      showToast("REPEAT ORDER QUEUED · same drop point · ACCEPT & FLY to launch", "good");
      return;
    }
    const row = ev.target.closest("[data-order]");
    if (!row) return;
    const id = row.getAttribute("data-order");
    if (!id) return;
    send("delivery_select", { order_id: id });
  };
  const inbox = $("dlv-inbox"); if (inbox) inbox.onclick = pick;
  const book = $("dlv-book");   if (book) book.onclick = pick;
}

function downloadPlan() {
  fetch("/api/mission/plan").then((r) => r.json()).then((plan) => {
    const blob = new Blob([JSON.stringify(plan, null, 2)], { type: "application/json" });
    const a = document.createElement("a"); a.href = URL.createObjectURL(blob);
    a.download = "mission.plan"; a.click();
  });
}
$("plan-file") && ($("plan-file").onchange = (e) => {
  const f = e.target.files[0]; if (!f) return;
  const rd = new FileReader();
  rd.onload = () => {
    try {
      const plan = JSON.parse(rd.result);
      fetch("/api/mission/plan", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(plan) })
        .then((r) => r.json()).then(() => { seeded = false; });
    } catch (err) { nlLog("✗ invalid .plan file", false); }
  };
  rd.readAsText(f);
});

/* ---------- panels ---------- */
function kv(rows) { return rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join(""); }
function updatePanels(d) {
  const t = d.telemetry || {}, av = d.avoidance || {}, m = d.mission || {}, h = d.health || {};
  // Transmitter link. The navigator stands down the moment the pilot moves the
  // mode switch, so whether that switch can reach the aircraft is worth seeing
  // on the ground, not discovering in the air.
  const rc = d.rc || {};
  const rcTxt = rc.connected
    ? `<span class="pill good">LINKED</span> ${rc.count || 0} ch`
    : '<span class="pill bad">NO RC</span>';
  $("tbl-telem").innerHTML = kv([
    ["Flight Mode", t.flight_mode], ["Armed", t.armed ? "YES" : "no"],
    ["Transmitter", rcTxt],
    ["GPS Fix", t.gps_fix], ["Satellites", t.satellites],
    ["Altitude", fmt(t.altitude, 1) + " m"], ["Ground Spd", fmt(t.ground_speed, 1) + " m/s"],
    ["Vert Spd", fmt(t.vert_speed, 1) + " m/s"], ["Heading", fmt(t.heading, 0) + "°"],
    ["Pitch", fmt(t.pitch, 1) + "°"], ["Roll", fmt(t.roll, 1) + "°"],
  ]);
  $("tbl-avoid").innerHTML = kv([
    ["Avoidance", av.enabled ? "ON" : "OFF"], ["Status", av.status],
    ["Sending Cmds", av.sending ? "yes" : "no"], ["Direction", av.direction],
    ["Obstacles", av.count], ["Closest", fmt(av.closest_m, 2) + " m"],
    ["TTC", fmt(av.ttc_s, 2) + " s"], ["CPA", fmt(av.cpa_m, 2) + " m"],
    ["Command", av.command], ["Reason", av.reason || "--"],
  ]);
  // Altitude hardlock + who is flying. Shown on every frame: during real
  // flight the operator needs to see the limit and the override state without
  // having to infer them from the aircraft's behaviour.
  const ceil = Number(m.alt_ceiling_m) || 0;
  const altNow = Number(t.altitude) || 0;
  const lockTxt = ceil
    ? `${fmt(altNow, 2)} m / ${fmt(ceil, 1)} m ceiling`
    : "--";
  const lockCls = ceil && altNow > ceil + 0.05 ? "bad"
                : ceil && altNow > ceil * 0.85 ? "warn" : "good";
  const rows = [
    ["Phase", m.phase], ["Waypoint", `${m.current_wp}/${m.total_wp}`],
    ["Altitude", `<span class="pill ${lockCls}">${lockTxt}</span>`],
    ["Control", m.pilot_override
      ? '<span class="pill bad">PILOT (transmitter)</span>'
      : '<span class="pill good">AUTO (ground station)</span>'],
    ["Avoiding", m.avoiding ? "yes" : "no"], ["Status", m.message || "--"],
  ];
  if (m.arm_refusal) rows.push(["Autopilot", `<span class="pill warn">${m.arm_refusal}</span>`]);
  $("tbl-mission").innerHTML = kv(rows);

  // The override is latched until a human clears it, so make the way out
  // impossible to miss: highlight RESUME for exactly as long as it is the
  // thing standing between the operator and a flyable aircraft.
  const resumeBtn = $("btn-resume");
  if (resumeBtn) {
    resumeBtn.classList.toggle("amber", !!m.pilot_override);
    resumeBtn.textContent = m.pilot_override
      ? "RESUME · TAKE BACK CONTROL"
      : "RESUME";
  }

  if (m.pilot_override !== lastOverride) {
    if (lastOverride !== null) {
      showToast(m.pilot_override
        ? "PILOT OVERRIDE · transmitter has control · press RESUME to fly again"
        : "AUTO · ground station has control",
        m.pilot_override ? "warn" : "good");
    }
    lastOverride = m.pilot_override;
  }
  $("tbl-health").innerHTML = kv([
    ["CPU", fmt(h.cpu, 0) + " %"], ["RAM", fmt(h.ram, 0) + " %"],
    ["Temp", fmt(h.temp, 0) + " °C"], ["LIDAR FPS", fmt(h.lidar_fps, 1)],
    ["Radar FPS", fmt(h.radar_fps, 1)], ["MAVLink FPS", fmt(h.mavlink_fps, 1)],
  ]);
  $("mode-actual").textContent = t.flight_mode || "--";
  $("mode-req").textContent = m.phase || "--";
  document.querySelectorAll("#mode-grid button").forEach((b) =>
    b.classList.toggle("on", b.dataset.mode === t.flight_mode));

  // announce flight-mode changes on-screen (skip the first render so it
  // doesn't fire just because the page loaded)
  const mode = t.flight_mode || null;
  if (mode && mode !== lastMode) {
    if (lastMode !== null) showToast("MODE · " + mode, "good");
    lastMode = mode;
  }
}

/* ---------- header ---------- */
function updateHeader(d) {
  const t = d.telemetry || {}, link = d.link || {};
  setPill($("pill-link"), link.connected ? "LINK OK" : "LINK LOST", link.connected ? "good" : "bad");
  setPill($("pill-source"), d.source || "SIM", d.sim ? "warn" : "good");
  setPill($("pill-mode"), t.flight_mode || "--", "");
  setPill($("pill-armed"), t.armed ? "ARMED" : "DISARMED", t.armed ? "good" : "bad");
  window.setPropTelemetry && window.setPropTelemetry(t);
  $("chip-sat").textContent = t.satellites || 0;
  $("chip-batt").textContent = fmt(t.battery_v, 1);
  $("chip-pkt").textContent = t.packets || 0;
  $("chip-cpu").textContent = fmt(d.health ? d.health.cpu : 0, 0);
  $("chip-clock").textContent = new Date().toLocaleTimeString();
  $("av-on").classList.toggle("on", av_enabled(d));
  $("av-off").classList.toggle("on", !av_enabled(d));
  $("src-sim").classList.toggle("on", d.sim);
  $("src-real").classList.toggle("on", !d.sim);
}
function av_enabled(d) { return d.avoidance ? d.avoidance.enabled : true; }

/* ---------- sparklines ---------- */
const sparkDefs = [["CPU %", "cpu"], ["RAM %", "ram"], ["Batt V", "battery_v"], ["LIDAR Hz", "lidar_fps"], ["MAVLink Hz", "mavlink_fps"], ["Net ms", "net"]];
const sparkData = {};
function buildSparks() {
  const wrap = $("health-spark");
  sparkDefs.forEach(([label, key]) => {
    sparkData[key] = [];
    const row = document.createElement("div"); row.className = "spark";
    row.innerHTML = `<span class="lbl">${label}</span><canvas width="120" height="18"></canvas><span class="val" id="sv-${key}">--</span>`;
    wrap.appendChild(row);
  });
}
function pushSpark(key, val, max) {
  const arr = sparkData[key]; if (!arr) return;
  arr.push(val); if (arr.length > 60) arr.shift();
  $("sv-" + key).textContent = fmt(val, key === "battery_v" ? 1 : 0);
  const cv = document.querySelectorAll("#health-spark .spark")[sparkDefs.findIndex((s) => s[1] === key)].querySelector("canvas");
  const ctx = cv.getContext("2d"); ctx.clearRect(0, 0, cv.width, cv.height);
  ctx.strokeStyle = "#22d3ee"; ctx.beginPath();
  const m = max || Math.max(1, ...arr);
  arr.forEach((v, i) => { const x = i / 59 * cv.width, y = cv.height - (v / m) * (cv.height - 2) - 1; i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
  ctx.stroke();
}
function updateSparks(d) {
  const h = d.health || {}, t = d.telemetry || {};
  pushSpark("cpu", h.cpu || 0, 100); pushSpark("ram", h.ram || 0, 100);
  pushSpark("battery_v", t.battery_v || 0, 17); pushSpark("lidar_fps", h.lidar_fps || 0);
  pushSpark("mavlink_fps", h.mavlink_fps || 0); pushSpark("net", netLatency, 200);
}

/* ---------- console ---------- */
function updateConsole(d) {
  const box = $("console");
  (d.console || []).forEach((e) => {
    if (e.id <= lastConsoleId) return;
    lastConsoleId = e.id;
    const div = document.createElement("div");
    const ts = new Date(e.t * 1000).toLocaleTimeString();
    div.innerHTML = `<span class="t">${ts}</span> <span class="s">${e.src}</span> <span class="${e.level}">${e.msg}</span>`;
    box.appendChild(div);

    // pop aux-switch / mode events from the flight controller on-screen too
    // (E-Stop, AutoTune attempts, mode-change rejects) so transmitter actions
    // are easy to track even when they aren't a flight-mode change. Skip the
    // first pass so the console backlog doesn't flood the screen on load.
    if (seededConsole && e.src === "mavlink.link") {
      const m = (e.msg || "").replace(/^FC:\s*/, "");
      if (/^RC\d+:/.test(m) || /Mode change to .+ (failed|denied)/i.test(m)) {
        const cls = e.level === "error" ? "bad" : (e.level === "warning" ? "warn" : "good");
        showToast(m, cls);
      }
    }
  });
  while (box.children.length > 200) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
  seededConsole = true;
}

/* ---------- cameras (live MJPEG + detection overlay) ---------- */
function initCams() {
  [0, 1].forEach((i) => {
    const img = $("camimg" + i);
    img.src = "/api/camera/" + i + "/stream";
    img.onerror = () => { setTimeout(() => { img.src = "/api/camera/" + i + "/stream?t=" + Date.now(); }, 2000); };
  });
}
function updateCam(i, cam) {
  const lbl = $("camlbl" + i), ov = $("camov" + i);
  const prefix = i === 0 ? "USB" : "P1";
  if (cam && cam.connected) {
    lbl.textContent = `${prefix} ${cam.res} ${cam.fps}fps`;
  } else {
    lbl.textContent = `${prefix} offline`;
  }
  ov.innerHTML = "";
  (cam && cam.detections || []).forEach((b) => {
    const box = document.createElement("div");
    box.className = "cam-box";
    box.style.left = (b.x * 100) + "%"; box.style.top = (b.y * 100) + "%";
    box.style.width = (b.w * 100) + "%"; box.style.height = (b.h * 100) + "%";
    const span = document.createElement("span");
    span.textContent = `${b.label} LOCK ${fmt(b.conf, 2)}`;
    box.appendChild(span); ov.appendChild(box);
  });
}

/* ---------- NL box ---------- */
function nlLog(text, ok) {
  const log = $("nl-log"), div = document.createElement("div");
  div.style.color = ok === undefined ? "var(--dim)" : (ok ? "var(--green)" : "var(--red)");
  div.textContent = text; log.prepend(div);
  while (log.children.length > 20) log.removeChild(log.lastChild);
}
function sendNL() {
  const el = $("nl-input"), text = el.value.trim(); if (!text) return;
  nlLog("> " + text); send("nl", { text }); el.value = "";
  nlHistoryPush(text);
}
function nlHistoryPush(text) {
  if (nlHistory[nlHistory.length - 1] !== text) {
    nlHistory.push(text);
    if (nlHistory.length > NL_HISTORY_MAX) nlHistory.shift();
    try { localStorage.setItem(NL_HISTORY_KEY, JSON.stringify(nlHistory)); } catch {}
  }
  nlHistoryIdx = nlHistory.length;
  nlDraft = "";
}
function nlHistoryNav(dir) {
  const el = $("nl-input");
  if (nlHistory.length === 0) return;
  if (nlHistoryIdx === nlHistory.length) nlDraft = el.value;  // stash in-progress text
  const next = nlHistoryIdx + dir;
  if (next < 0 || next > nlHistory.length) return;             // clamp at both ends
  nlHistoryIdx = next;
  el.value = nlHistoryIdx === nlHistory.length ? nlDraft : nlHistory[nlHistoryIdx];
  requestAnimationFrame(() => el.setSelectionRange(el.value.length, el.value.length));
}

/* ---------- save scan ---------- */
function saveScan() {
  const scan = latest.scan || null;
  const payload = { ts: Date.now() / 1000, source: latest.source, scan, obstacles: latest.obstacles || [] };
  const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
  const a = document.createElement("a"); a.href = URL.createObjectURL(blob);
  a.download = "scan_" + Date.now() + ".json"; a.click();
  nlLog("✓ scan saved", true);
}

/* ---------- render ---------- */
function render(d) {
  updateHeader(d); drawRadar(d); updateMap(d); updatePanels(d); updateDelivery(d);
  updateSparks(d); updateConsole(d);
  updateCam(0, (d.cameras || [])[0]); updateCam(1, (d.cameras || [])[1]);
}

/* ---------- logs list ---------- */
function loadLogList() {
  fetch("/api/logs").then((r) => r.json()).then((d) => {
    const sel = $("log-select"); sel.innerHTML = '<option value="">-- select log --</option>';
    (d.logs || []).forEach((f) => { const o = document.createElement("option"); o.value = f; o.textContent = f; sel.appendChild(o); });
  });
}

/* ---------- wiring ---------- */
function wire() {
  document.querySelectorAll("[data-cmd]").forEach((b) => {
    const cmd = b.dataset.cmd;
    b.addEventListener("click", () => {
      if (cmd === "arm") {
        confirmDanger("ARM MOTORS?",
          "The propellers will spin up immediately. Ensure the area is clear.",
          () => send("arm"));
        return;
      }
      send(cmd);
    });
  });
  $("nl-send").onclick = sendNL;
  $("nl-input").addEventListener("keydown", (e) => {
    if (e.key === "Enter") { e.preventDefault(); sendNL(); }
    else if (e.key === "ArrowUp") { e.preventDefault(); nlHistoryNav(-1); }
    else if (e.key === "ArrowDown") { e.preventDefault(); nlHistoryNav(1); }
  });
  // "AUTO MODE · READY FOR NEXT MISSION": one click = clear a latched
  // transmitter takeover (resume) + a stuck delivery-in-progress flag
  // (delivery_reset, itself gated to a disarmed aircraft server-side, so
  // this can never interrupt a real flight). Replaces "restart the Pi" as
  // the way back to flyable after an override, an abort, or a finish.
  $("btn-auto-mode").onclick = () => {
    send("resume");
    send("delivery_reset");
    showToast("AUTO MODE · clearing takeover + delivery state", "good");
  };
  $("src-sim").onclick = () => send("set_source", { mode: "sim" });
  $("src-real").onclick = () => confirmDanger("SWITCH TO REAL MODE?",
    "Commands will control the PHYSICAL drone (Pixhawk + LIDAR). Simulation safety is disabled.",
    () => send("set_source", { mode: "real" }));
  // Payload servo on Pixhawk AUX1 (= output channel 9). MG995R, ~8.3us/deg.
  // Two positions only: Lock=1100us holds the payload, Release=1410us drops it
  // (~310us swing ~= 37deg). Use the tuning slider below to find the exact
  // positions on the real mechanism, then update these two constants.
  const SRV_CH = 9;
  const SRV_LOCK = 1100, SRV_RELEASE = 1410;
  $("srv-lock").onclick = () => send("set_servo", { channel: SRV_CH, pwm: SRV_LOCK });
  $("srv-release").onclick = () => confirmDanger("RELEASE PAYLOAD?",
    "The AUX1 servo (MG995R) will swing open and drop whatever is attached.",
    () => send("set_servo", { channel: SRV_CH, pwm: SRV_RELEASE }));
  // TEMP tuning control: emergency-braking trigger distance. The number field and
  // the slider stay in sync and both push set_stop_distance live (throttled for
  // the slider). Once tuned, the exact value goes into avoidance_stop_m in config.
  const brakeNum = $("brake-dist-num"), brakeSlider = $("brake-dist");
  if (brakeNum && brakeSlider) {
    let brakeLast = 0;
    const brakeSend = (v, throttle) => {
      v = Math.max(0.3, Math.min(8, Number(v) || 0));
      brakeNum.value = v; brakeSlider.value = v;
      const now = performance.now();
      if (!throttle || now - brakeLast > 80) {
        send("set_stop_distance", { distance_m: v });
        brakeLast = now;
      }
    };
    brakeSlider.oninput = () => brakeSend(brakeSlider.value, true);
    brakeSlider.onchange = () => brakeSend(brakeSlider.value, false);
    brakeNum.onchange = () => brakeSend(brakeNum.value, false);
  }
  $("btn-savescan").onclick = saveScan;
  $("btn-replay").onclick = () => { const f = $("log-select").value; if (f) send("replay", { file: f }); };
  $("log-select").onchange = (e) => { if (e.target.value) send("replay", { file: e.target.value }); };
}

window.addEventListener("DOMContentLoaded", () => {
  initTheme(); buildModeGrid(); initMap(); buildSparks(); setupRadarControls();
  wire(); initDelivery(); initCams(); loadLogList(); connect();
});
