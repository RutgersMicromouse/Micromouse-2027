// =============================================================
// TUNING STUDIO
//
// The part of the dashboard that shows what the control and
// safety code is doing: the safety supervisor's state and its
// stopping-distance sum, rolling charts, the results of the
// robot's own tests, and the ladder of trials (maze size by
// kind of move).
//
// It only reads the lines the robot prints. app.js hands every
// whole line to Studio.line(); a true answer means "this was a
// data line, keep it out of the output panel".
//
// Keep in step with the firmware: the "[TEL]" and "[SAF]" lines
// in src/main.cpp (telemetryTask), "[SAFE]" (operatorTask) and
// the "[TURNTEST] done" line in src/ui/turn_test/turn_test.cpp.
// =============================================================

const Studio = (() => {
  const el = (id) => document.getElementById(id);

  // SafetySupervisor::Reason, in the order of the enum in src/control/safety/safety.h
  const REASONS = [
    "nothing to object to",
    "front wall inside the stopping distance",
    "the motion controller stopped the move (stall, encoder, heading or tilt)",
    "heading sensor in fault: steering on the wheels alone",
    "the two front sensors disagree",
    "motor supply low",
    "motor driver not answering",
  ];

  // ---------------------------------------------------------- rolling charts
  const KEEP = 150; // 30 s at the robot's 5 lines a second
  const series = { heading: [], speed: [], left: [], right: [], l90: [], r90: [], need: [], room: [] };
  const push = (name, value) => {
    series[name].push(value);
    if (series[name].length > KEEP) series[name].shift();
  };

  // One chart: a few named lines over the same span, scaled to what they hold
  function draw(canvasId, lines, unit) {
    const canvas = el(canvasId);
    const ctx = canvas.getContext("2d");
    const w = (canvas.width = canvas.clientWidth * devicePixelRatio);
    const h = (canvas.height = canvas.clientHeight * devicePixelRatio);
    ctx.clearRect(0, 0, w, h);
    const all = lines.flatMap((l) => series[l.name]).filter((v) => v !== null);
    if (!all.length) return;
    let lo = Math.min(...all), hi = Math.max(...all);
    if (hi - lo < 1) { hi += 0.5; lo -= 0.5; }
    const pad = 14 * devicePixelRatio;
    const y = (v) => h - pad - ((v - lo) / (hi - lo)) * (h - 2 * pad);
    ctx.font = 11 * devicePixelRatio + "px system-ui";
    ctx.fillStyle = "#8f9aaa";
    ctx.fillText(hi.toFixed(0) + " " + unit, 4, 12 * devicePixelRatio);
    ctx.fillText(lo.toFixed(0) + " " + unit, 4, h - 3);
    ctx.lineWidth = 1.5 * devicePixelRatio;
    for (const line of lines) {
      const data = series[line.name];
      ctx.strokeStyle = line.color;
      ctx.beginPath();
      let pen = false;
      data.forEach((v, i) => {
        if (v === null) { pen = false; return; }
        const x = (i / (KEEP - 1)) * w;
        if (pen) ctx.lineTo(x, y(v)); else ctx.moveTo(x, y(v));
        pen = true;
      });
      ctx.stroke();
    }
  }

  function redraw() {
    draw("chartHeading", [{ name: "heading", color: "#7fb6ff" }], "deg");
    draw("chartSpeed", [
      { name: "speed", color: "#e8edf4" },
      { name: "left", color: "#3ddc84" },
      { name: "right", color: "#ffb454" },
    ], "mm/s");
    draw("chartSides", [{ name: "l90", color: "#3ddc84" }, { name: "r90", color: "#ffb454" }], "");
    draw("chartStop", [{ name: "need", color: "#ff5d5d" }, { name: "room", color: "#7fb6ff" }], "mm");
  }

  // ---------------------------------------------------------- the robot's lines
  function telemetry(line) {
    const heading = line.match(/Hdg:\s*(-?[\d.]+)/);
    const speed = line.match(/Spd:\s*(-?[\d.]+)/);
    const ir = line.match(/L90=\s*(\d+).*R90=\s*(\d+)/);
    if (heading) push("heading", Number(heading[1]));
    if (speed) push("speed", Number(speed[1]));
    if (ir) { push("l90", Number(ir[1])); push("r90", Number(ir[2])); }
  }

  function safety(line) {
    const m = line.match(
      /\[SAF\] (\w+) \| reason (\d+) \| v (-?[\d.]+) \| need (-?[\d.]+) \| room (-?[\d.]+) \| scale ([\d.]+) \| wl (-?[\d.]+) \| wr (-?[\d.]+) \| gz (-?[\d.]+) \| tilt (-?[\d.]+) \| supply ([\d.]+)/
    );
    if (!m) return;
    const [state, reason, , need, room, scale, wl, wr, gz, tilt, supply] = m.slice(1).map((v, i) => (i ? Number(v) : v));
    const badge = el("safeState");
    badge.textContent = state;
    badge.className = "badge " + state.toLowerCase();
    el("safeReason").textContent = REASONS[reason] || "";
    el("safeNeed").textContent = need.toFixed(0);
    el("safeRoom").textContent = room < 0 ? "not known" : room.toFixed(0) + " mm";
    el("safeScale").textContent = Math.round(scale * 100) + " %";
    el("safeTurnRate").textContent = gz.toFixed(0);
    el("safeTilt").textContent = tilt.toFixed(0);
    el("safeSupply").textContent = supply.toFixed(1);
    // The two bars share one scale: the red one must stay shorter than the blue one
    const full = Math.max(120, need, room);
    el("barNeed").style.width = (need / full) * 100 + "%";
    el("barRoom").style.width = room < 0 ? "0" : (room / full) * 100 + "%";
    push("need", need);
    push("room", room < 0 ? null : room);
    push("left", wl);
    push("right", wr);
  }

  // "[TURNTEST] done | search speed | LEFT mean -0.3 sd 0.3 worst -0.8 (n=5) | RIGHT mean +0.2 ..."
  function turnTest(line) {
    const side = (name) => line.match(new RegExp(name + " mean ([-+][\\d.]+) sd ([\\d.]+) worst ([-+][\\d.]+)"));
    const left = side("LEFT"), right = side("RIGHT");
    if (!left || !right) return;
    const settings = (line.match(/h_kp=.*/) || [""])[0];
    const row = el("turnRows").insertRow(0);
    const worst = Math.max(Math.abs(left[3]), Math.abs(right[3]));
    row.className = worst <= 2 ? "good" : worst <= 5 ? "fair" : "bad";
    [new Date().toLocaleTimeString(), left[1], left[2], left[3], right[1], right[2], right[3],
     (Number(left[1]) - Number(right[1])).toFixed(1), settings]
      .forEach((text) => (row.insertCell().textContent = text));
  }

  // Every line that is the outcome of something, newest first
  const RESULT_TAGS = /^\[(SAFE|SAFETY|SAFETY ALERT|LADDER|LIMITS|BRAKE|TURNTEST|STRAIGHTTEST|STRAIGHT|MOVECHECK|MOTORCHECK|CURVETEST|CALIB|RESULT|NAV)\]/;
  function result(line) {
    const item = document.createElement("li");
    item.textContent = new Date().toLocaleTimeString() + "  " + line.trim();
    if (/^\[(SAFE|SAFETY)/.test(line)) item.className = "alert";
    el("results").prepend(item);
    while (el("results").children.length > 200) el("results").lastChild.remove();
  }

  // Where the robot's checklist stands, from what it prints ("ladder" asks it to say)
  function checklist(text) {
    const open = text.match(/\[LADDER\] Step (\d) of \d open: ([^.]*)/), finished = text.match(/\[LADDER\] Step (\d) \(.*\) FINISHED at (\d+)%/);
    const all = text.includes("[LADDER] Every step is finished"), reset = text.includes("[LADDER] Started over");
    const at = all ? 7 : open ? Number(open[1]) : finished ? Number(finished[1]) + 1 : reset ? 1 : 0;
    if (at) {
      el("ladderNow").textContent = all ? "every step finished" : "step " + at + " is open" + (open ? " (" + open[2] + ")" : "");
      document.querySelectorAll("[data-step]").forEach((li) => {
        const step = Number(li.dataset.step);
        li.className = step < at ? "done" : step === at ? "open" : "locked";
      });
    }
    const maze = text.match(/(?:Maze size is now|MAZE:) (\d+)x\d+/);
    if (maze) el("mazeNow").textContent = maze[1] + " x " + maze[1];
  }

  function line(text) {
    const trimmed = text.replace(/^\s+/, "");
    checklist(trimmed);
    if (trimmed.startsWith("[TEL]")) { telemetry(trimmed); return false; } // app.js shows the bars from it
    if (trimmed.startsWith("[SAF]")) { safety(trimmed); redraw(); return true; }
    if (trimmed.startsWith("[TURNTEST] done")) turnTest(trimmed);
    if (RESULT_TAGS.test(trimmed)) result(trimmed);
    return false;
  }

  // ---------------------------------------------------------- the ladder of trials
  // Each maze size has to pass each kind of move before the next size is worth trying. A square
  // is clicked through: not tried -> passed -> failed. Kept in this browser only.
  const SIZES = ["3 x 3", "5 x 5", "8 x 8", "16 x 16"];
  const MOVES = ["Straights", "Turns on the spot", "Smooth curves", "Diagonals"];
  const MARKS = ["", "pass", "fail"];
  let ladder = {};
  try { ladder = JSON.parse(localStorage.getItem("ladder") || "{}"); } catch (e) {}

  function drawLadder() {
    const table = el("ladder");
    table.innerHTML = "<tr><th></th>" + MOVES.map((m) => "<th>" + m + "</th>").join("") + "</tr>";
    SIZES.forEach((size) => {
      const row = table.insertRow();
      row.insertCell().textContent = size;
      MOVES.forEach((move) => {
        const key = size + "|" + move;
        const cell = row.insertCell();
        const mark = ladder[key] || 0;
        cell.className = "step " + MARKS[mark];
        cell.textContent = ["not tried", "passed", "failed"][mark];
        cell.onclick = () => {
          ladder[key] = (mark + 1) % 3;
          try { localStorage.setItem("ladder", JSON.stringify(ladder)); } catch (e) {}
          drawLadder();
        };
      });
    });
  }

  // ---------------------------------------------------------- replaying a recording
  // A file written by tools/run_recorder (each line "HH:MM:SS.mmm  text") is fed through exactly
  // as if the robot were printing it, 200 lines at a time so the page stays alive.
  function replay(file, feed) {
    file.text().then((text) => {
      const lines = text.split(/\r?\n/).map((l) => l.replace(/^\d\d:\d\d:\d\d\.\d+\s\s/, "")).filter((l) => !l.startsWith("# "));
      let at = 0;
      const step = () => {
        for (let n = 0; n < 200 && at < lines.length; n++, at++) feed(lines[at] + "\n");
        el("replayNote").textContent = at < lines.length ? "replaying " + at + " of " + lines.length + " lines" : "replayed " + lines.length + " lines";
        if (at < lines.length) setTimeout(step, 30);
      };
      step();
    });
  }

  function start(feed, send) {
    drawLadder();
    el("replayFile").addEventListener("change", (event) => event.target.files[0] && replay(event.target.files[0], feed));
    document.querySelectorAll("[data-studio-cmd]").forEach((button) =>
      button.addEventListener("click", () => send(button.dataset.studioCmd).catch((error) => console.warn(error))));
    window.addEventListener("resize", redraw);
  }

  return { line, start };
})();
