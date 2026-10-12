// =============================================================
// SIMULATION VIEWER: LAB BENCH
//
// The third tab: change the simulated robot and its
// surroundings in the page and run it. Needs the lab's own
// server (sim/lab/server.py, started by run_viewer.bat), which
// runs the real firmware on the simulated robot and answers
// with what was measured. The form is built from the list of
// parameters the simulation itself reports, so it cannot drift
// from sim/lab/params.h.
// =============================================================

(() => {
  const el = (id) => document.getElementById(id);
  let lab = null; // What /api/params answered: parameters with bounds, scenarios, the robot's tuning

  // Shown beside each group of settings. The units are in the names (kg, n = newtons, mm, deg, s).
  const GROUPS = [
    ["Model, and the speed run's moves (0 turns on the spot, 1 curves, 2 diagonals)", ["dynamics", "moves"]],
    ["Robot mechanics (force model only)", ["mass_kg", "inertia_kg_m2", "drive_weight_share", "friction", "rolling_n", "yaw_drag_nms",
      "com_height_mm", "com_behind_axle_mm", "stall_force_n", "wheel_mass_kg", "tyre_stiffness", "driver_lag_s", "deadband", "motor_left", "motor_right"]],
    ["Supply", ["supply_volts", "sag_volts"]],
    ["Suction fan (force model only)", ["fan_on", "fan_mass_kg", "fan_down_n", "fan_seal", "fan_power_w"]],
    ["Encoders", ["enc_miss_left", "enc_miss_right"]],
    ["IR light and noise", ["ir_ambient", "ir_noise", "ir_ambient_noise", "ir_falloff", "ir_angle_power"]],
    ["IR strength of each pair (1 = nominal)", ["ir_gain_l90", "ir_gain_l45", "ir_gain_fl", "ir_gain_fr", "ir_gain_r45", "ir_gain_r90"]],
    ["IR chance of a pair giving nothing", ["ir_drop_l90", "ir_drop_l45", "ir_drop_fl", "ir_drop_fr", "ir_drop_r45", "ir_drop_r90"]],
    ["IMU", ["imu_noise_deg", "imu_drift_deg_s", "imu_drop", "imu_flip"]],
    ["Judging and experiments", ["robot_radius_mm", "exp_speed"]],
  ];
  const LIGHT = { // Presets for the lighting; every number stays editable below
    dim: { ir_ambient: 10, ir_ambient_noise: 0 }, normal: { ir_ambient: 60, ir_ambient_noise: 0 },
    bright: { ir_ambient: 1500, ir_ambient_noise: 0.02 },
  };

  function build() {
    lab.params.push({ name: "moves", value: 0, lo: 0, hi: 2 }); // The firmware's move set, not a physics number
    const byName = Object.fromEntries(lab.params.map((p) => [p.name, p]));
    el("benchForm").innerHTML = GROUPS.map(([title, names]) =>
      `<h2>${title}</h2><div class="fields">` + names.filter((n) => byName[n]).map((n) =>
        `<label>${n}<input type="number" step="any" data-param="${n}" data-default="${byName[n].value}" value="${byName[n].value}" ` +
        `min="${byName[n].lo}" max="${byName[n].hi}" title="${byName[n].lo} to ${byName[n].hi}"></label>`).join("") + "</div>").join("");
    el("benchPreset").innerHTML = '<option value="">Start from a scenario...</option>' +
      Object.entries(lab.scenarios).map(([name, s]) => `<option value="${name}" title="${s.what}">${name}</option>`).join("");
    el("benchLight").innerHTML = '<option value="">Lighting preset...</option>' + Object.keys(LIGHT).map((k) => `<option>${k}</option>`).join("");
    el("benchForm").oninput = mark;
  }

  const inputs = () => [...document.querySelectorAll("[data-param]")];
  // A setting changed from its default is outlined, and only those are sent
  function mark() { inputs().forEach((i) => i.classList.toggle("changed", Number(i.value) !== Number(i.dataset.default))); }
  function apply(values) { inputs().forEach((i) => { if (values[i.dataset.param] !== undefined) i.value = values[i.dataset.param]; }); mark(); }

  el("benchPreset").onchange = (e) => {
    if (!e.target.value) return;
    inputs().forEach((i) => (i.value = i.dataset.default));
    apply(Object.fromEntries((lab.scenarios[e.target.value].sets || []).map((s) => s.split("="))));
    el("benchNote").textContent = lab.scenarios[e.target.value].what;
  };
  el("benchLight").onchange = (e) => e.target.value && apply(LIGHT[e.target.value]);
  el("benchDefaults").onclick = () => { inputs().forEach((i) => (i.value = i.dataset.default)); mark(); el("benchNote").textContent = ""; };

  function run(experiment) {
    const bad = inputs().find((i) => !i.checkValidity() || i.value === "");
    if (bad) { el("benchOut").textContent = `${bad.dataset.param} must be ${bad.min} to ${bad.max}`; return; }
    const sets = inputs().filter((i) => Number(i.value) !== Number(i.dataset.default)).map((i) => `${i.dataset.param}=${Number(i.value)}`);
    const request = { sets, size: Number(el("benchSize").value), seed: Number(el("benchSeed").value) || 1,
                      robot_tuning: el("benchTuning").checked, experiment: experiment || null };
    el("benchOut").textContent = "Running the firmware on the simulated robot... (a 16 x 16 maze takes a few minutes)";
    el("benchOut").className = "dim";
    document.querySelectorAll("#bench button").forEach((b) => (b.disabled = true));
    fetch("/api/run", { method: "POST", body: JSON.stringify(request) }).then((r) => r.json()).then((result) => {
      const lines = [];
      if (result.error) lines.push("Not run: " + result.error);
      else if (experiment) { lines.push(result.output.trim(), "", "Open the Replay tab to watch it."); window.reloadTrace && window.reloadTrace(); }
      else {
        lines.push(result.failed ? "FAILED the lab's limits" : "PASSED every limit", "");
        result.runs.forEach((r) => {
          lines.push(`${r.run}: ${r.failures.length ? "FAIL" : "pass"}   ${r.time_s} s, closest ${r.min_clearance_mm} mm, ` +
            `turns worst ${r.left_turn_worst_deg} / ${r.right_turn_worst_deg} deg, off centre ${r.lateral_max_mm} mm, slipping ${r.slip_s} s`);
          r.failures.forEach((f) => lines.push("      " + f));
        });
        if (!result.runs.length) lines.push("Nothing ran: " + result.output.trim());
        lines.push("", "Settings: " + (sets.join(" ") || "all defaults (the original model)") + (request.robot_tuning ? "  + the robot's saved tuning" : "  + the code's tuning"),
          `Maze: random ${request.size} x ${request.size}, seed ${request.seed}. Open the Replay tab to watch it.`);
        window.reloadTrace && window.reloadTrace();
      }
      el("benchOut").textContent = lines.join("\n");
      el("benchOut").className = result.error || result.failed ? "bad" : experiment ? "" : "good";
    }).catch(() => { el("benchOut").textContent = "The lab's server did not answer."; })
      .finally(() => document.querySelectorAll("#bench button").forEach((b) => (b.disabled = false)));
  }
  // ---------------------------------------------------------- the robot's checklist, on the simulated robot
  // The same six steps as the checklist on the real robot (src/ui/ladder), each run here with
  // the settings in the form above, so a change of mass, friction or a sensor fault shows at
  // once in which step it bites. A step's row says what was measured and whether it passed.
  const CHECKLIST = [
    { name: "1. Turns on the spot", what: "ten turns at the tuned speed, each within 2 degrees of 90 (the robot then also tries faster)", ask: { experiment: "turns" },
      judge: (r) => { const t = r.rows[0], worst = Math.max(Math.abs(t.left_worst_deg), Math.abs(t.right_worst_deg));
                      return [worst <= 2, `worst turn ${worst.toFixed(1)} deg off`]; } },
    { name: "2. Straights", what: "a closed 5-cell corridor whose side walls change, end to end and back, faster each time", ask: { experiment: "limits_hard" },
      judge: (r) => { const s = r.rows[0]; return [s.best_percent > 0, `fastest clean level ${s.best_percent} % (${s.ended})`]; } },
    { name: "3. Curves, 5 x 5", what: "the layout the checklist says to build: a search, then speed runs with smooth curves, inside every limit", ask: { size: 5, moves: 1, maze: "curves_5x5" } },
    { name: "4. Diagonals, 5 x 5", what: "the same with diagonals and V turns", ask: { size: 5, moves: 2, maze: "diagonals_5x5" } },
    { name: "5. Everything, 8 x 8", what: "a random half-size maze, all moves", ask: { size: 8, moves: 2 } },
    { name: "6. Everything, 16 x 16", what: "the full maze, all moves (takes a few minutes)", ask: { size: 16, moves: 2 } },
  ];
  const mazeVerdict = (r) => {
    const failing = r.runs.filter((run) => run.failures.length);
    return [!r.failed, r.failed ? (failing.length ? `${failing[0].run}: ${failing[0].failures[0]}` : "nothing ran")
                                : `closest approach ${Math.min(...r.runs.map((run) => run.min_clearance_mm))} mm`];
  };

  function step(index) {
    const item = CHECKLIST[index], cell = el("simStep" + index);
    const sets = inputs().filter((i) => Number(i.value) !== Number(i.dataset.default) && i.dataset.param !== "moves")
      .map((i) => `${i.dataset.param}=${Number(i.value)}`);
    if (item.ask.moves !== undefined) sets.push("moves=" + item.ask.moves);
    const request = { sets, size: item.ask.size || 5, seed: Number(el("benchSeed").value) || 1,
                      robot_tuning: el("benchTuning").checked, experiment: item.ask.experiment || null, maze: item.ask.maze || null };
    cell.textContent = "running..."; cell.className = "dim";
    document.querySelectorAll("#bench button").forEach((b) => (b.disabled = true));
    return fetch("/api/run", { method: "POST", body: JSON.stringify(request) }).then((r) => r.json()).then((result) => {
      const [passed, detail] = result.error ? [false, result.error] : (item.judge || mazeVerdict)(result);
      cell.textContent = (passed ? "PASSED: " : "NOT PASSED: ") + detail;
      cell.className = passed ? "good" : "bad";
      window.reloadTrace && window.reloadTrace();
      return passed;
    }).catch(() => { cell.textContent = "the lab's server did not answer"; cell.className = "bad"; return false; })
      .finally(() => document.querySelectorAll("#bench button").forEach((b) => (b.disabled = false)));
  }
  el("simChecklist").innerHTML = CHECKLIST.map((item, i) =>
    `<tr><td><button data-sim-step="${i}">Run</button></td><td><b>${item.name}</b><div class="dim">${item.what}</div></td><td id="simStep${i}" class="dim">not run</td></tr>`).join("");
  document.querySelectorAll("[data-sim-step]").forEach((b) => (b.onclick = () => step(Number(b.dataset.simStep))));
  // In order, stopping at the first step that does not pass, as the robot's own checklist does
  el("simAll").onclick = async () => { for (let i = 0; i < CHECKLIST.length; i++) if (!(await step(i))) break; };

  el("benchRun").onclick = () => run(null);
  document.querySelectorAll("[data-experiment]").forEach((b) => (b.onclick = () => run(b.dataset.experiment)));

  fetch("/api/params").then((r) => (r.ok ? r.json() : Promise.reject())).then((answer) => { lab = answer; build(); })
    .catch(() => { el("benchOut").textContent = "The lab bench needs the lab's server: start sim\\viewer\\run_viewer.bat (it runs sim/lab/server.py)."; });
})();
