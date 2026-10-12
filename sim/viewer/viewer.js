// =============================================================
// SIMULATION VIEWER: REPLAY
//
// Plays back a trace from sim/tests/test_firmware_drive.py
// (--trace file). Every number shown comes from that file: the
// simulated robot's real state and what the firmware believed
// and commanded at the same moment. Nothing is animated that
// was not simulated.
//
// Keep the field names in step with traceMaze / traceFrame /
// traceEnd in the harness, and the name lists below in step
// with MotionAction, NavState (src/types.h) and
// SafetySupervisor::State (src/control/safety/safety.h).
// =============================================================

const ACTIONS = ["idle", "forward (cells)", "forward (mm)", "half cell", "diagonal", "turn left 90", "turn right 90",
  "turn left 45", "turn right 45", "curve left 90", "curve right 90", "curve left 45", "curve right 45",
  "turn round 180", "align on front wall", "square on front wall", "emergency stop"];
const NAV = ["idle", "calibrating", "searching to the centre", "exploring back to the start", "search done",
  "speed run", "speed run finished", "error"];
const SAFE = ["NORMAL", "CAUTION", "BRAKING", "FAULT", "RECOVERY"];
const COMPASS = ["north", "east", "south", "west"];
const CELL = 180; // mm

// What the chart can show. Each line: a value per frame, its colour, and the span it is drawn over.
const GROUPS = {
  "Motion: clearance, speed, heading error": [
    { name: "clearance to the nearest wall or post (mm)", color: "#7fb6ff", lo: 0, hi: 130, get: (f) => f.clr },
    { name: "speed (mm/s)", color: "#3ddc84", lo: -50, hi: 600, get: (f) => f.spd },
    { name: "heading error (deg)", color: "#ffb454", lo: -30, hi: 30, get: (f) => f.he },
  ],
  "Wheels and motors": [
    { name: "left wheel (mm/s)", color: "#3ddc84", lo: -600, hi: 600, get: (f) => f.wl },
    { name: "right wheel (mm/s)", color: "#ffb454", lo: -600, hi: 600, get: (f) => f.wr },
    { name: "left motor command", color: "#1f7a3f", lo: -800, hi: 800, get: (f) => f.ml },
    { name: "right motor command", color: "#b7791f", lo: -800, hi: 800, get: (f) => f.mr },
  ],
  "IR: side sensors": [
    { name: "L90", color: "#3ddc84", lo: 0, hi: 3000, get: (f) => f.ir[0] },
    { name: "R90", color: "#ffb454", lo: 0, hi: 3000, get: (f) => f.ir[5] },
    { name: "L45", color: "#1f7a3f", lo: 0, hi: 3000, get: (f) => f.ir[1] },
    { name: "R45", color: "#b7791f", lo: 0, hi: 3000, get: (f) => f.ir[4] },
  ],
  "IR: front sensors, and turn rate": [
    { name: "FL", color: "#3ddc84", lo: 0, hi: 4095, get: (f) => f.ir[2] },
    { name: "FR", color: "#ffb454", lo: 0, hi: 4095, get: (f) => f.ir[3] },
    { name: "turn rate (deg/s)", color: "#7fb6ff", lo: -400, hi: 400, get: (f) => f.gz },
  ],
};

const el = (id) => document.getElementById(id);
let maze = null;      // The header line of the trace
let runs = [];        // [{ name, frames, maps, end, events }]
let run = null;       // The run on screen
let at = 0;           // Index of the frame on screen
let playing = false;
let lastTick = 0;

// ---------------------------------------------------------- loading
function load(text) {
  maze = null; runs = [];
  let frames = [], maps = [];
  const close = (name, end) => { runs.push({ name, frames, maps, end, events: findEvents(frames, end) }); frames = []; maps = []; };
  for (const line of text.split("\n")) {
    if (!line.trim()) continue;
    let rec;
    try { rec = JSON.parse(line); } catch (e) { continue; } // (a line cut off by a killed run)
    if (rec.maze !== undefined) maze = rec;
    else if (rec.end !== undefined) close(rec.end, rec);
    else if (rec.map !== undefined) maps.push({ i: frames.length - 1, map: rec.map, open: rec.open });
    else frames.push(rec);
  }
  if (frames.length) close("unfinished run", { ok_clr: 46, min_clr: NaN, abort: "the trace stops here" });
  runs = runs.filter((r) => r.frames.length);
  el("run").innerHTML = runs.map((r, i) => `<option value="${i}">${r.name}</option>`).join("");
  el("loaded").textContent = maze ? `${maze.maze}: ${runs.length} run(s)` : "that file has no maze line";
  if (maze && runs.length) choose(0);
}

// Collisions (centre closer to a wall than the robot's radius allows), near misses, safety states, aborts
function findEvents(frames, end) {
  const events = [];
  let hitting = false, near = false, safe = 0;
  frames.forEach((f, i) => {
    const hit = f.clr < end.ok_clr, close = f.clr < end.ok_clr + 10;
    if (hit && !hitting) events.push({ i, bad: true, text: `COLLISION: ${f.clr.toFixed(0)} mm from a wall or post` });
    else if (close && !near && !hit) events.push({ i, bad: false, text: `near miss: ${f.clr.toFixed(0)} mm` });
    hitting = hit; near = close;
    if (f.safe !== safe) { events.push({ i, bad: f.safe >= 2, text: `safety supervisor: ${SAFE[f.safe]}` }); safe = f.safe; }
  });
  if (end.abort) events.push({ i: frames.length - 1, bad: true, text: "aborted: " + end.abort });
  return events;
}

function choose(index) {
  run = runs[index];
  at = 0;
  el("seek").max = Math.max(0, run.frames.length - 1);
  const e = run.end;
  const collided = run.events.some((ev) => ev.text.startsWith("COLLISION"));
  el("verdict").textContent = (e.abort ? "ABORTED: " + e.abort : collided ? "COLLISION" : "No collision") +
    (isNaN(e.min_clr) ? "" : `. Closest approach ${e.min_clr} mm (a collision is under ${e.ok_clr} mm).`);
  el("verdict").className = e.abort || collided ? "bad" : "good";
  el("events").innerHTML = "";
  run.events.forEach((ev) => {
    const li = document.createElement("li");
    li.textContent = run.frames[ev.i].t.toFixed(2) + " s  " + ev.text;
    if (ev.bad) li.className = "bad";
    li.onclick = () => { at = Math.max(0, ev.i - 25); playing = false; show(); }; // 25 frames = half a second before
    el("events").append(li);
  });
  if (!run.events.length) el("events").innerHTML = '<li class="dim">none</li>';
  show();
}

// ---------------------------------------------------------- drawing
function fit(canvas) {
  canvas.width = canvas.clientWidth * devicePixelRatio;
  canvas.height = canvas.clientHeight * devicePixelRatio;
  return canvas.getContext("2d");
}

function drawMaze() {
  const canvas = el("maze"), ctx = fit(canvas);
  const n = maze.n, scale = canvas.width / (n * CELL + 40);
  // World mm (x east, y north, start cell centre at 0,0) -> canvas pixels
  const X = (mm) => (mm + CELL / 2 + 20) * scale, Y = (mm) => canvas.height - (mm + CELL / 2 + 20) * scale;
  const f = run.frames[at];
  // The four edges of a cell as [x0, y0, x1, y1], in the order north, east, south, west
  const edges = (cx, cy) => [[cx - 90, cy + 90, cx + 90, cy + 90], [cx + 90, cy - 90, cx + 90, cy + 90],
                             [cx - 90, cy - 90, cx + 90, cy - 90], [cx - 90, cy - 90, cx - 90, cy + 90]];

  // The cell the firmware believes it is in
  ctx.strokeStyle = "#2b6cb0"; ctx.lineWidth = 2 * devicePixelRatio;
  ctx.strokeRect(X(f.cx * CELL - 90), Y(f.cy * CELL + 90), CELL * scale, CELL * scale);

  // True walls and posts
  ctx.fillStyle = "#c9d3df";
  const w = maze.wall_half;
  const rect = (x0, y0, x1, y1) => ctx.fillRect(X(x0), Y(y1), (x1 - x0) * scale, (y1 - y0) * scale);
  for (let y = 0; y < n; y++) for (let x = 0; x < n; x++) {
    const bits = parseInt(maze.walls[y * n + x], 16), cx = x * CELL, cy = y * CELL;
    if (bits & 1) rect(cx - 90, cy + 90 - w, cx + 90, cy + 90 + w);
    if (bits & 2) rect(cx + 90 - w, cy - 90, cx + 90 + w, cy + 90);
    if (bits & 4) rect(cx - 90, cy - 90 - w, cx + 90, cy - 90 + w);
    if (bits & 8) rect(cx - 90 - w, cy - 90, cx - 90 + w, cy + 90);
  }
  ctx.fillStyle = "#8f9aaa";
  for (let y = 0; y <= n; y++) for (let x = 0; x <= n; x++) rect(x * CELL - 90 - w, y * CELL - 90 - w, x * CELL - 90 + w, y * CELL - 90 + w);

  // The firmware's own map at this moment, drawn just inside each cell: a wall it believes in
  // blue, an edge it knows nothing about dotted. A blue line where no grey wall is, or none
  // where one is, is the map being wrong.
  const snapshot = el("showMap").checked && [...run.maps].reverse().find((m) => m.i <= at);
  if (snapshot) {
    const inset = 14;
    for (let y = 0; y < n; y++) for (let x = 0; x < n; x++) {
      const walls = parseInt(snapshot.map[y * n + x], 16), open = parseInt(snapshot.open[y * n + x], 16);
      edges(x * CELL, y * CELL).forEach(([x0, y0, x1, y1], d) => {
        if (!(walls & (1 << d)) && (open & (1 << d))) return; // A confirmed opening: nothing drawn
        const sx = Math.sign(x * CELL - (x0 + x1) / 2), sy = Math.sign(y * CELL - (y0 + y1) / 2);
        ctx.strokeStyle = walls & (1 << d) ? "#4f9dff" : "#5a6676";
        ctx.setLineDash(walls & (1 << d) ? [] : [3 * devicePixelRatio, 5 * devicePixelRatio]);
        ctx.lineWidth = 1.5 * devicePixelRatio;
        ctx.beginPath();
        ctx.moveTo(X(x0 + sx * inset + (y0 === y1 ? inset : 0)), Y(y0 + sy * inset + (x0 === x1 ? inset : 0)));
        ctx.lineTo(X(x1 + sx * inset - (y0 === y1 ? inset : 0)), Y(y1 + sy * inset - (x0 === x1 ? inset : 0)));
        ctx.stroke();
      });
    }
    ctx.setLineDash([]);
  }

  // The path actually followed so far; red where it was a collision
  ctx.lineWidth = 2 * devicePixelRatio;
  for (let i = 1; i <= at; i++) {
    const a = run.frames[i - 1], b = run.frames[i];
    ctx.strokeStyle = b.clr < run.end.ok_clr ? "#ff5d5d" : "#3ddc84";
    ctx.beginPath(); ctx.moveTo(X(a.x), Y(a.y)); ctx.lineTo(X(b.x), Y(b.y)); ctx.stroke();
  }

  // The six IR beams, from where each sensor sits to what its beam really hits.
  // Heading is degrees anticlockwise from north: ahead = (-sin, cos), left = (-cos, -sin).
  const h = f.h * Math.PI / 180, s = Math.sin(h), c = Math.cos(h);
  const sees = [f.seen[0] === "L", null, f.seen[1] === "F", f.seen[1] === "F", null, f.seen[2] === "R"];
  for (let i = 0; i < 6; i++) {
    const ox = f.x + maze.sensor_ahead[i] * -s + maze.sensor_left[i] * -c;
    const oy = f.y + maze.sensor_ahead[i] * c + maze.sensor_left[i] * -s;
    const aim = h + maze.sensor_aim[i] * Math.PI / 180, len = Math.min(f.rng[i], 400);
    ctx.strokeStyle = sees[i] === true ? "#7cf29a" : sees[i] === false ? "#5a6676" : "#ffb454";
    ctx.lineWidth = 1 * devicePixelRatio;
    ctx.beginPath(); ctx.moveTo(X(ox), Y(oy)); ctx.lineTo(X(ox - Math.sin(aim) * len), Y(oy + Math.cos(aim) * len)); ctx.stroke();
  }

  // The robot: the circle the simulation tests for collisions, and a line for its heading
  ctx.strokeStyle = f.clr < run.end.ok_clr ? "#ff5d5d" : "#e8edf4"; ctx.lineWidth = 2 * devicePixelRatio;
  ctx.beginPath(); ctx.arc(X(f.x), Y(f.y), maze.radius * scale, 0, 2 * Math.PI); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(X(f.x), Y(f.y)); ctx.lineTo(X(f.x - s * maze.radius), Y(f.y + c * maze.radius)); ctx.stroke();
}

function drawChart() {
  const canvas = el("chart"), ctx = fit(canvas), frames = run.frames, w = canvas.width, h = canvas.height;
  const lines = GROUPS[el("group").value];
  el("legend").innerHTML = "Whole run, click to jump: " + lines.map((l) => `<b style="color:${l.color}">${l.name}</b>`).join(" &nbsp; ") +
    (lines[0].name.startsWith("clearance") ? ' &nbsp; <b style="color:#ff5d5d">collision limit</b>' : "");
  const step = Math.max(1, Math.floor(frames.length / w));
  for (const line of lines) {
    ctx.strokeStyle = line.color; ctx.lineWidth = devicePixelRatio; ctx.beginPath();
    for (let i = 0; i < frames.length; i += step) {
      const x = (i / (frames.length - 1 || 1)) * w;
      const y = h - ((Math.min(line.hi, Math.max(line.lo, line.get(frames[i]))) - line.lo) / (line.hi - line.lo)) * h;
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    }
    ctx.stroke();
  }
  if (lines[0].name.startsWith("clearance")) { // The collision limit, on the clearance scale
    const limit = h - (run.end.ok_clr / 130) * h;
    ctx.strokeStyle = "#ff5d5d"; ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.moveTo(0, limit); ctx.lineTo(w, limit); ctx.stroke(); ctx.setLineDash([]);
  }
  const x = (at / (frames.length - 1 || 1)) * w;
  ctx.strokeStyle = "#e8edf4"; ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
}

function show() {
  if (!run || !run.frames.length) return;
  const f = run.frames[at];
  el("seek").value = at;
  el("clock").textContent = `${f.t.toFixed(2)} s of ${run.frames[run.frames.length - 1].t.toFixed(1)} s`;
  el("pos").textContent = `${f.x.toFixed(0)}, ${f.y.toFixed(0)} mm`;
  el("heading").textContent = `${f.h.toFixed(1)} deg`;
  el("gz").textContent = `${f.gz} deg/s`;
  el("wheels").textContent = `${f.wl}, ${f.wr} mm/s`;
  el("clr").textContent = `${f.clr.toFixed(0)} mm`;
  el("rng").textContent = f.rng.map((v) => (v >= 999 ? "far" : v)).join(" ") + " mm";
  el("nav").textContent = NAV[f.nav] || f.nav;
  el("act").textContent = ACTIONS[f.act] || f.act;
  el("cell").textContent = `(${f.cx}, ${f.cy}) ${COMPASS[f.dir] || ""}`;
  // The firmware tracks cells, not millimetres, and names the cell a move is heading for
  const cells = Math.hypot(f.x - f.cx * CELL, f.y - f.cy * CELL) / CELL;
  el("poserr").textContent = `${cells.toFixed(1)} cells`;
  el("poserr").className = cells > 1.6 ? "bad" : "";
  el("spd").textContent = `${f.spd} mm/s`;
  el("he").textContent = `${f.he.toFixed(1)} deg`;
  el("ir").textContent = f.ir.join(" ");
  el("seen").textContent = (f.seen[0] === "L" ? "left " : "") + (f.seen[1] === "F" ? "front " : "") + (f.seen[2] === "R" ? "right" : "") || "none";
  el("motors").textContent = `${f.ml}, ${f.mr}`;
  el("safe").textContent = SAFE[f.safe] || f.safe;
  el("play").textContent = playing ? "Pause" : "Play";
  drawMaze();
  drawChart();
}

// ---------------------------------------------------------- controls
function tick(now) {
  if (playing && run) {
    // Frames are 20 ms of simulated time apart
    const frames = ((now - lastTick) / 20) * Number(el("rate").value);
    if (frames >= 1) {
      at = Math.min(run.frames.length - 1, at + Math.floor(frames));
      lastTick = now;
      if (at === run.frames.length - 1) playing = false;
      show();
    }
  } else {
    lastTick = now;
  }
  requestAnimationFrame(tick);
}

function save(name, href) {
  const link = document.createElement("a");
  link.download = name; link.href = href; link.click();
}

el("group").innerHTML = Object.keys(GROUPS).map((g) => `<option>${g}</option>`).join("");
el("play").onclick = () => { if (run && at === run.frames.length - 1) at = 0; playing = !playing; show(); };
el("reset").onclick = () => { at = 0; playing = false; show(); };
el("seek").oninput = (e) => { at = Number(e.target.value); playing = false; show(); };
el("run").onchange = (e) => { playing = false; choose(Number(e.target.value)); };
el("group").onchange = show;
el("showMap").onchange = show;
el("chart").onclick = (e) => { if (run) { at = Math.round((e.offsetX / e.target.clientWidth) * (run.frames.length - 1)); playing = false; show(); } };
el("file").onchange = (e) => e.target.files[0] && e.target.files[0].text().then(load);
el("saveMaze").onclick = () => run && save("maze.png", el("maze").toDataURL("image/png"));
el("saveChart").onclick = () => run && save("chart.png", el("chart").toDataURL("image/png"));
el("saveFrame").onclick = () => run && save("moment.json", "data:application/json," + encodeURIComponent(JSON.stringify({ maze: maze.maze, run: run.name, frame: run.frames[at] }, null, 1)));
window.onresize = show;

// When the page is served from sim/ (run_viewer.bat does that), the last trace loads by itself
window.reloadTrace = () => fetch("trace.jsonl", { cache: "no-store" }).then((r) => (r.ok ? r.text() : Promise.reject())).then(load).catch(() => {});
window.reloadTrace();
requestAnimationFrame(tick);
