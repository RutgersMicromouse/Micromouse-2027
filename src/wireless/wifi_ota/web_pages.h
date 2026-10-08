#pragma once

// The two web pages the robot serves over its own Wi-Fi. Included only by wifi_ota.cpp.
//   UPDATE_INDEX_HTML - the firmware upload page at /update
//   APP_PAGE_HTML     - the phone app at http://192.168.4.1

// Web update page HTML
static const char UPDATE_INDEX_HTML[] PROGMEM =
    "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Antigrav-Mouse OTA</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:-apple-system,sans-serif;background:#121214;color:#eee;text-align:center;padding:40px 20px;}"
    ".card{background:#1e1e24;max-width:460px;margin:0 auto;padding:30px;border-radius:12px;box-shadow:0 8px 24px rgba(0,0,0,0.5);}"
    "h1{color:#4af;margin-bottom:8px;}p{color:#aaa;font-size:14px;}"
    "input[type='file']{display:block;margin:24px auto;color:#ccc;}"
    "input[type='submit']{background:#4af;color:#000;font-weight:700;border:none;padding:12px 28px;border-radius:6px;cursor:pointer;font-size:16px;}"
    "input[type='submit']:hover{background:#38d;}"
    "#progress{display:none;margin-top:20px;font-size:14px;color:#fa4;}"
    "</style></head><body>"
    "<div class='card'>"
    "<h1>🐭 Antigrav-Mouse</h1>"
    "<p>Over-The-Air Wireless Firmware Update</p>"
    "<form method='POST' action='/update' enctype='multipart/form-data' onsubmit='document.getElementById(\"progress\").style.display=\"block\";'>"
    "<input type='file' name='update' accept='.bin' required>"
    "<input type='submit' value='Upload & Flash Firmware'>"
    "</form>"
    "<div id='progress'>⚡ Flashing firmware to ESP32-S3... Robot will reboot in ~8 seconds.</div>"
    "</div></body></html>";


// The phone app: one page served by the robot itself at http://192.168.4.1
// (join the robot's Wi-Fi first). It asks /data four times a second and sends button presses to /cmd.
static const char APP_PAGE_HTML[] PROGMEM = R"PAGE(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-title" content="Antigrav">
<meta name="theme-color" content="#0b0d10">
<title>Antigrav-Mouse</title>
<style>
*{box-sizing:border-box}
body{margin:0;padding:16px;padding-top:max(16px,env(safe-area-inset-top));background:#0b0d10;color:#f5f7fa;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif}
h1{font-size:24px;margin:0}
.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:#6d7480;margin-right:6px}
.live .dot{background:#52d273;box-shadow:0 0 12px #52d273}
.dim{color:#8f9aaa;font-size:13px}
.card{background:#171c24;border:1px solid #2c3440;border-radius:16px;padding:14px;margin-bottom:12px}
#state{font-size:17px;font-weight:700;margin:2px 0 0}
.row{display:grid;grid-template-columns:1fr 1fr;gap:10px}
button{appearance:none;border:0;border-radius:14px;padding:16px 10px;font-size:17px;font-weight:700;color:#0b0d10;background:#f5f7fa;width:100%}
button:active{opacity:.6}
#start{background:#52d273;font-size:22px;padding:22px 10px}
#stop{background:#ff5d5d;color:#fff;font-size:22px;padding:22px 10px}
button.plain{background:#232a34;color:#eef2f7;border:1px solid #303844;font-size:15px;padding:13px 8px}
.bars{display:grid;grid-template-columns:repeat(6,1fr);gap:6px}
.bar{display:flex;flex-direction:column;align-items:center;gap:4px}
.bar b{font:600 13px ui-monospace,Consolas,monospace}
.bar span{color:#8f9aaa;font-size:11px}
.track{width:100%;height:110px;background:#07090c;border:1px solid #232a34;border-radius:8px;display:flex;align-items:flex-end;overflow:hidden}
.fill{width:100%;height:0;background:linear-gradient(#7cf29a,#2f9e52);transition:height .15s linear}
.vals{display:grid;grid-template-columns:1fr 1fr;gap:6px 14px;font-size:14px}
.vals b{font-family:ui-monospace,Consolas,monospace}
#log{margin:0;height:220px;overflow:auto;background:#07090c;border:1px solid #232a34;border-radius:10px;padding:10px;font:12px/1.4 ui-monospace,Consolas,monospace;color:#cfe3d4;white-space:pre-wrap;word-break:break-word}
form{display:flex;gap:8px;margin-top:10px}
input{flex:1;min-width:0;background:#07090c;border:1px solid #303844;border-radius:12px;padding:12px;color:#f5f7fa;font:14px ui-monospace,Consolas,monospace}
form button{width:auto;padding:12px 18px}
a{color:#7fb6ff}
button.busy{background:#ff5d5d;color:#fff;border-color:#ff5d5d}
.tune{display:grid;grid-template-columns:1fr 130px;gap:8px 10px;align-items:center;font-size:14px;margin-bottom:10px}
.tune input.changed{border-color:#ffc94d}
#maze{display:block;width:100%;max-width:420px;margin:0 auto;background:#07090c;border-radius:10px}
.key{display:flex;flex-wrap:wrap;gap:4px 14px;margin-top:8px;font-size:12px;color:#8f9aaa}
.key i{display:inline-block;width:14px;height:4px;border-radius:2px;margin-right:5px;vertical-align:middle}
.cams{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.cams img{width:100%;border-radius:8px;background:#07090c;display:block}
@media(max-width:520px){.cams{grid-template-columns:1fr}}
</style></head><body>
<div class="top"><h1>Antigrav-Mouse</h1><div id="link" class="dim"><span class="dot"></span><span id="linkText">connecting</span></div></div>

<div class="card"><div class="dim">The robot is</div><div id="state">...</div><div class="dim" id="mode"></div>
<div style="margin-top:8px">It thinks it is in cell <b id="cell">--</b></div><div class="dim">Cells explored so far: <b id="visited">--</b> &nbsp;(the start cell is (0, 0); first number counts right, second counts forward)</div>
<div style="margin-top:8px">45&deg; sensors say the next cell has: <b id="look">--</b></div>
<div class="dim">At the last cell edge it <b id="why">--</b></div>
<div class="dim">Times it had to brake because the next move came late: <b id="late">--</b></div></div>

<div class="card">
  <div class="dim" style="margin-bottom:8px">The maze as the robot believes it. The start cell is bottom-left, north is up.</div>
  <canvas id="maze" width="840" height="840"></canvas>
  <div class="key">
    <span><i style="background:#f5f7fa"></i>wall it believes in</span>
    <span><i style="background:#52d273"></i>sensor sees a wall (bar = where it thinks the wall is)</span>
    <span><i style="background:#5b6675"></i>sensor sees nothing</span>
    <span><i style="background:#ffc94d"></i>goal</span>
  </div>
  <div class="dim" style="margin-top:6px">The robot is drawn in the middle of the cell it believes it is in, turned to its measured heading. It keeps track of cells, not millimetres, so it does not slide between cells here. The robot and its sensors are drawn to scale from the PCB drawing; beam lengths are rough.</div>
</div>

<div class="card">
  <div class="top" style="margin-bottom:8px"><span class="dim">Cameras</span><span class="dim" id="camNote"></span></div>
  <div class="cams" id="cams"></div>
  <div class="dim" style="margin-top:8px">The cameras are plugged into a computer, not the robot. On that computer, join the robot's Wi-Fi and run <b>tools/camera_feeds/camera_server.py</b>, then put the address it prints here:</div>
  <form id="camForm"><input id="camAddr" autocomplete="off" autocapitalize="off" spellcheck="false" placeholder="http://192.168.4.2:8090"><button type="submit">Show</button></form>
</div>

<div class="row" style="margin-bottom:12px">
  <button id="start">START</button>
  <button id="stop">STOP</button>
</div>
<div class="card">
  <div class="top" style="margin-bottom:6px"><span>Speed run at <b id="speedPctText">35</b>% of full speed</span><span class="dim">10 = slowest, 100 = full</span></div>
  <input type="range" id="speedPct" min="10" max="100" step="5" value="35" style="width:100%;padding:0;margin:6px 0 12px">
  <div class="row">
    <button class="plain" id="speedrun">Speed run at this speed</button>
    <button class="plain" data-cmd="return">Return to start</button>
  </div>
</div>
<div class="row" style="margin-bottom:12px">
  <button class="plain" data-cmd="calib">Calibrate sensors (facing the back wall), then turn round</button>
  <button class="plain" data-cmd="clear">Forget the maze</button>
  <button class="plain" data-cmd="cell">Drive one cell (search, step by step)</button>
  <button class="plain" data-cmd="left45">Turn left 45&deg;</button>
  <button class="plain" data-cmd="right45">Turn right 45&deg;</button>
  <button class="plain" data-cmd="left90">Turn left 90&deg; (on the spot)</button>
  <button class="plain" data-cmd="right90">Turn right 90&deg; (on the spot)</button>
  <button class="plain" data-cmd="curveleft">Curve left 90&deg; (rolling)</button>
  <button class="plain" data-cmd="curveright">Curve right 90&deg; (rolling)</button>
  <button class="plain" data-cmd="health">Health check</button>
  <button class="plain" data-cmd="resetall">Reset sensors</button>
  <button class="plain" data-cmd="ir">Raw IR readings</button>
</div>

<div class="card">
  <div class="top" style="margin-bottom:8px"><span class="dim">IR wall sensors</span><span class="dim">Walls: <b id="walls">- - -</b></span></div>
  <div class="bars">
    <div class="bar"><div class="track"><div class="fill" id="f0"></div></div><b id="v0">--</b><span>L 90</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f1"></div></div><b id="v1">--</b><span>L 45</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f2"></div></div><b id="v2">--</b><span>Front L</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f3"></div></div><b id="v3">--</b><span>Front R</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f4"></div></div><b id="v4">--</b><span>R 45</span></div>
    <div class="bar"><div class="track"><div class="fill" id="f5"></div></div><b id="v5">--</b><span>R 90</span></div>
  </div>
</div>

<div class="card"><div class="vals">
  <div>Heading <b id="heading">--</b>&deg;</div><div>Battery <b id="vbat">--</b> V</div>
  <div>Left wheel <b id="encL">--</b></div><div>Right wheel <b id="encR">--</b></div>
  <div>Motor driver <b id="motor">--</b></div><div>IMU <b id="imu">--</b></div>
  <div>Motor supply <b id="supply">--</b> V</div><div>Loop <b id="loop">--</b> &micro;s</div>
</div></div>

<div class="card">
  <div class="dim" style="margin-bottom:8px">Tuning. Change a number, press Apply, try it. A changed box is outlined in yellow until applied. Not allowed during a run.</div>
  <div class="tune" id="tune"></div>
  <div class="row">
    <button class="plain" id="tuneApply">Apply</button>
    <button class="plain" id="tuneSave">Save on the robot</button>
    <button class="plain" data-cmd="tune">List values</button>
    <button class="plain" id="tuneReset">Back to code values</button>
  </div>
</div>

<div class="card">
  <div class="top" style="margin-bottom:8px"><span class="dim">Robot output (same as the serial monitor)</span><button class="plain" id="clear" style="width:auto;padding:6px 12px;font-size:13px">Clear</button></div>
  <pre id="log"></pre>
  <form id="form"><input id="cmd" autocomplete="off" autocapitalize="off" spellcheck="false" placeholder="command, e.g. status"><button type="submit">Send</button></form>
</div>
<p class="dim" style="text-align:center"><a href="/update">Upload new firmware</a></p>

<script>
const $ = id => document.getElementById(id);
let next = 0, misses = 0;
let done = 0;        // Commands the robot has taken so far (from its last answer)
let waiting = null;  // The button whose command is still being carried out

// A button given here turns red until the robot has taken its command and has stopped moving
function send(command, button) {
  $("log").textContent += "> " + command + "\n";
  $("log").scrollTop = $("log").scrollHeight;
  fetch("/cmd?c=" + encodeURIComponent(command)).catch(() => {});
  if (button) {
    if (waiting) waiting.button.classList.remove("busy");
    button.classList.add("busy");
    waiting = { button: button, done: done, since: Date.now() };
  }
}

$("start").onclick = () => send("start");
$("stop").onclick = () => send("stop");
document.querySelectorAll("button[data-cmd]").forEach(b => b.onclick = () => send(b.dataset.cmd, b));
// The speed slider: remembered on this phone, sent with the Speed run button ("speedrun 45")
try { $("speedPct").value = localStorage.getItem("speedPct") || 35; } catch (e) {}
const showSpeed = () => { $("speedPctText").textContent = $("speedPct").value; };
$("speedPct").oninput = () => { showSpeed(); try { localStorage.setItem("speedPct", $("speedPct").value); } catch (e) {} };
showSpeed();
$("speedrun").onclick = () => send("speedrun " + $("speedPct").value, $("speedrun"));
$("clear").onclick = () => { $("log").textContent = ""; };
// The Tuning card. Same names, in the same order, as kTune in control.cpp: keep the two in step.
const TUNE = [["v_kp", "Speed loop P"], ["v_ki", "Speed loop I"], ["v_kd", "Speed loop D"],
  ["h_kp", "Heading loop P"], ["h_ki", "Heading loop I"], ["h_kd", "Heading loop D"],
  ["k_sync", "Wheel sync"], ["enc_a", "Wheel-speed smoothing (1 = none)"], ["imu_a", "Heading smoothing (1 = none)"],
  ["dist_k", "Distance scale (raise if it stops short)"],
  ["d_kp", "Distance loop P"], ["d_ki", "Distance loop I"], ["d_kd", "Distance loop D"],
  ["w_kp", "Wall centring P"], ["w_ki", "Wall centring I"], ["w_kd", "Wall centring D"],
  ["ff_ks", "Feedforward: friction"], ["ff_kv", "Feedforward: per mm/s"], ["ff_ka", "Feedforward: per mm/s\u00b2"],
  ["turn_ff", "Turn feedforward scale (lower if turns overshoot)"],
  ["h_max", "Heading loop: most effort"], ["h_imax", "Heading loop: most from I"],
  ["w_max", "Wall centring: most steering (deg)"], ["w_gyro", "Wall centring: turn-rate damping"]];
TUNE.forEach(([name, label]) => {
  $("tune").insertAdjacentHTML("beforeend", "<div>" + label + " <span class=dim>" + name + "</span></div>" +
    "<input id=t_" + name + " inputmode=decimal autocomplete=off>");
  $("t_" + name).oninput = e => e.target.classList.add("changed");
});
const pause = ms => new Promise(done => setTimeout(done, ms));
// The robot takes one command at a time, so changed values are sent one after another
async function applyTuning() {
  for (const [name] of TUNE) {
    const box = $("t_" + name);
    if (!box.classList.contains("changed")) continue;
    send("tune " + name + " " + box.value.trim());
    box.classList.remove("changed");
    await pause(300);
  }
}
$("tuneApply").onclick = applyTuning;
// Saving replaces the set kept on the robot, which is used at every power-on, so it asks twice:
// once showing exactly what would be saved, then once more.
$("tuneSave").onclick = async () => {
  const values = TUNE.map(([name, label]) => label + ": " + $("t_" + name).value.trim()).join("\n");
  if (!confirm("Save these values on the robot?\n\n" + values)) return;
  if (!confirm("Are you sure? This replaces the tuning saved on the robot and is used every time it is switched on.")) return;
  await applyTuning();
  send("tune save");
};
$("tuneReset").onclick = () => {
  if (!confirm("Go back to the values in the code and wipe the set saved on the robot?")) return;
  TUNE.forEach(([name]) => $("t_" + name).classList.remove("changed"));
  send("tune reset");
};

$("form").onsubmit = e => {
  e.preventDefault();
  const c = $("cmd").value.trim();
  if (c) send(c);
  $("cmd").value = "";
};

// ---------------------------------------------------------------- The maze picture
// The robot's shape, in mm ahead of the wheel axle and mm to the left of the centre line. Read
// off the 1:1 PCB drawing (PCB_Micromouse_Fall_26_3, 2026-10-08) and checked against a photo:
// the board is 68.6 mm wide, its front arc reaches 33 mm ahead of the axle and its back edge is
// 37 mm behind. The axle is taken as the middle of the two motor cut-outs.
// Sensors, in the order L90, L45, FL, FR, R45, R90: where the tip of each emitter / receiver
// pair is, and the way it points (degrees, left positive).
const SENSOR_AHEAD = [16, 31, 38, 38, 31, 16];
const SENSOR_LEFT = [36, 24, 9, -9, -24, -36];
const SENSOR_AIM = [90, 45, 0, 0, -45, -90];
const BOARD = [[-37, 34], [21, 34], [28, 24], [32, 12], [33, 0], [32, -12], [28, -24], [21, -34], [-37, -34]];
const WHEEL_LEFT_MM = 40, WHEEL_LENGTH_MM = 40, WHEEL_WIDTH_MM = 8; // Wheels sit outside the board
const CELL_MM = 180;

function drawMaze(s) {
  const n = s.n;
  if (!n || !s.map || s.map.length < n * n) return;
  const canvas = $("maze"), g = canvas.getContext("2d");
  const pad = 20, size = canvas.width - 2 * pad, cell = size / n, mm = cell / CELL_MM;
  // Maze millimetres (x right, y forward) to canvas pixels (y down)
  const px = x => pad + x * mm, py = y => pad + size - y * mm;
  g.clearRect(0, 0, canvas.width, canvas.height);

  for (let y = 0; y < n; y++) for (let x = 0; x < n; x++) {
    const seen = +s.seen[y * n + x];
    g.fillStyle = (seen & 1) ? "#1b2530" : "#0f1319"; // Visited cells are lighter
    g.fillRect(px(x * CELL_MM) + 1, py((y + 1) * CELL_MM) + 1, cell - 2, cell - 2);
    if (seen & 2) {
      g.strokeStyle = "#ffc94d"; g.lineWidth = 3;
      g.strokeRect(px(x * CELL_MM) + cell * .3, py((y + 1) * CELL_MM) + cell * .3, cell * .4, cell * .4);
    }
  }
  g.lineCap = "round";
  const line = (x1, y1, x2, y2) => { g.beginPath(); g.moveTo(px(x1), py(y1)); g.lineTo(px(x2), py(y2)); g.stroke(); };
  for (let y = 0; y < n; y++) for (let x = 0; x < n; x++) {
    const w = parseInt(s.map[y * n + x], 16), x0 = x * CELL_MM, y0 = y * CELL_MM, x1 = x0 + CELL_MM, y1 = y0 + CELL_MM;
    const wall = (on, a, b, c, d) => {
      g.strokeStyle = on ? "#f5f7fa" : "#232a34"; g.lineWidth = on ? Math.max(3, 12 * mm) : 1;
      line(a, b, c, d);
    };
    wall(w & 1, x0, y1, x1, y1); wall(w & 2, x1, y0, x1, y1); wall(w & 4, x0, y0, x1, y0); wall(w & 8, x0, y0, x0, y1);
  }

  // The robot: middle of its cell, turned to the heading it measures (0 = the way it started)
  const cx = (s.x + .5) * CELL_MM, cy = (s.y + .5) * CELL_MM;
  const th = (90 + (s.heading || 0)) * Math.PI / 180, fx = Math.cos(th), fy = Math.sin(th);
  const at = (ahead, left) => [cx + ahead * fx - left * fy, cy + ahead * fy + left * fx];
  const shape = points => {
    g.beginPath();
    points.forEach(([a, l], i) => {
      const p = at(a, l);
      i ? g.lineTo(px(p[0]), py(p[1])) : g.moveTo(px(p[0]), py(p[1]));
    });
    g.closePath(); g.fill(); g.stroke();
  };
  g.lineWidth = 2;
  g.fillStyle = "#3d7dff"; g.strokeStyle = "#cfe0ff";
  shape(BOARD);
  g.fillStyle = "#11151b"; g.strokeStyle = "#8f9aaa";
  for (const side of [1, -1]) {
    const inner = side * (WHEEL_LEFT_MM - WHEEL_WIDTH_MM / 2), outer = side * (WHEEL_LEFT_MM + WHEEL_WIDTH_MM / 2);
    shape([[WHEEL_LENGTH_MM / 2, inner], [WHEEL_LENGTH_MM / 2, outer], [-WHEEL_LENGTH_MM / 2, outer], [-WHEEL_LENGTH_MM / 2, inner]]);
  }

  // Each sensor: a beam from where it sits, the way it points. Above its wall level the beam is
  // green and ends in a bar where the robot reckons the wall is (weaker reading = further away).
  if (s.ir) for (let i = 0; i < 6; i++) {
    const p = at(SENSOR_AHEAD[i], SENSOR_LEFT[i]);
    const a = th + SENSOR_AIM[i] * Math.PI / 180, ux = Math.cos(a), uy = Math.sin(a);
    const level = s.lvl ? s.lvl[i] : 0, sees = level > 0 && s.ir[i] >= level;
    const reach = sees ? Math.max(15, Math.min(170, 90 * Math.sqrt(level / Math.max(1, s.ir[i])))) : 60;
    g.strokeStyle = sees ? "#52d273" : "#5b6675"; g.lineWidth = sees ? 4 : 2;
    g.setLineDash(sees ? [] : [6, 6]);
    line(p[0], p[1], p[0] + ux * reach, p[1] + uy * reach);
    g.setLineDash([]);
    if (sees) { g.lineWidth = 6; line(p[0] + ux * reach + uy * 14, p[1] + uy * reach - ux * 14, p[0] + ux * reach - uy * 14, p[1] + uy * reach + ux * 14); }
    g.fillStyle = sees ? "#52d273" : "#8f9aaa";
    g.beginPath(); g.arc(px(p[0]), py(p[1]), 5, 0, 7); g.fill();
  }
}

// ---------------------------------------------------------------- Cameras
// Pictures come straight from the camera computer (tools/camera_feeds/camera_server.py), not
// through the robot. The address is remembered on this phone or computer.
async function showCameras(address) {
  address = address.trim().replace(/\/+$/, "");
  if (address && !/^https?:\/\//.test(address)) address = "http://" + address;
  $("cams").innerHTML = "";
  if (!address) { $("camNote").textContent = "not set up"; return; }
  $("camAddr").value = address;
  try { localStorage.setItem("camAddr", address); } catch (e) {}
  $("camNote").textContent = "looking for " + address;
  try {
    const list = await (await fetch(address + "/cameras.json", { cache: "no-store" })).json();
    $("camNote").textContent = list.length + " camera" + (list.length === 1 ? "" : "s");
    list.forEach(cam => {
      const img = document.createElement("img");
      img.alt = cam.name; img.src = address + cam.stream;
      $("cams").appendChild(img);
    });
  } catch (e) {
    $("camNote").textContent = "no answer from " + address;
  }
}
$("camForm").onsubmit = e => { e.preventDefault(); showCameras($("camAddr").value); };
let savedCams = "";
try { savedCams = localStorage.getItem("camAddr") || ""; } catch (e) {}
showCameras(savedCams);

function show(d) {
  const s = d.status || {};
  drawMaze(s);
  if (d.done !== undefined) done = d.done;
  if (waiting) {
    const taken = done > waiting.done;
    // Finished, or the robot never took the command (it was busy, or was restarted)
    if ((taken && !s.busy) || (!taken && Date.now() - waiting.since > 8000)) {
      waiting.button.classList.remove("busy");
      waiting = null;
    }
  }
  $("state").textContent = s.state || "...";
  $("mode").textContent = s.mode ? "Mode: " + s.mode + (s.tier ? ", speed tier " + s.tier : "") : "";
  if (s.ir) for (let i = 0; i < 6; i++) {
    $("v" + i).textContent = s.ir[i];
    $("f" + i).style.height = Math.min(100, s.ir[i] / 4095 * 100) + "%";
  }
  if (s.walls) $("walls").textContent =
    (s.walls[0] === "L" ? "LEFT " : "- ") + (s.walls[1] === "F" ? "FRONT " : "- ") + (s.walls[2] === "R" ? "RIGHT" : "-");
  for (const k of ["heading", "vbat", "encL", "encR", "motor", "imu", "supply", "loop", "cell", "visited", "look", "why", "late"])
    if (s[k] !== undefined) $(k).textContent = s[k];
  // Tuning boxes follow the robot, except one that is being edited
  if (s.tune) TUNE.forEach(([name], i) => {
    const box = $("t_" + name);
    if (!box.classList.contains("changed") && document.activeElement !== box) box.value = s.tune[i];
  });
  if (d.log) {
    const log = $("log");
    const atBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 40;
    log.textContent = (log.textContent + d.log).slice(-30000);
    if (atBottom) log.scrollTop = log.scrollHeight;
  }
  next = d.next;
}

async function poll() {
  try {
    const r = await fetch("/data?since=" + next, { cache: "no-store" });
    show(await r.json());
    misses = 0;
  } catch (e) {
    misses++;
  }
  $("link").className = misses < 4 ? "dim live" : "dim";
  $("linkText").textContent = misses < 4 ? "live" : "no answer from the robot";
  setTimeout(poll, 250);
}
poll();
</script>
</body></html>)PAGE";
