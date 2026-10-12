// =============================================================
// SIMULATION VIEWER: RESULTS AND PROGRESS
//
// The second tab. It shows what sim/lab/lab.py has actually
// run (sim/lab/results/history.jsonl, one line per scenario
// run, and the result file behind each) and the hand-kept
// record of the development stages (sim/lab/progress.json).
// Needs the page served from sim/, as run_viewer.bat does.
// =============================================================

(() => {
  const el = (id) => document.getElementById(id);
  const cell = (row, text, className) => { const td = row.insertCell(); td.textContent = text; if (className) td.className = className; return td; };

  // The three tabs: each button shows its own section and hides the other two
  const TABS = { tabReplay: "replay", tabBench: "bench", tabResults: "results" };
  function showTab(tab) {
    Object.entries(TABS).forEach(([button, section]) => { el(section).hidden = button !== tab; el(button).className = button === tab ? "on" : ""; });
    if (tab === "tabResults") loadResults();
    if (tab === "tabReplay") window.onresize();
  }
  Object.keys(TABS).forEach((button) => (el(button).onclick = () => showTab(button)));

  function loadResults() {
    fetch("../lab/results/history.jsonl", { cache: "no-store" }).then((r) => (r.ok ? r.text() : Promise.reject())).then((text) => {
      const runs = text.split("\n").filter((l) => l.trim()).map((l) => JSON.parse(l)).reverse();
      const table = el("history");
      table.innerHTML = "<tr><th>When</th><th>Scenario</th><th>Mazes</th><th>Trials failed</th><th>First failure</th><th>Closest (mm)</th><th>Firmware</th></tr>";
      runs.forEach((h) => {
        const row = table.insertRow();
        row.className = "row";
        cell(row, h.when.replace("_", " "));
        cell(row, h.scenario);
        cell(row, h.maze);
        cell(row, `${h.failed} of ${h.trials}` + (h.regression ? "  REGRESSION" : ""), h.regression ? "bad" : h.failed ? "warn" : "good");
        cell(row, h.first_failure || "", "dim");
        cell(row, h.worst_clearance_mm == null ? "" : h.worst_clearance_mm);
        cell(row, h.commit);
        row.onclick = () => detail(h);
      });
      const failed = runs.filter((h) => h.failed).length;
      el("historyNote").textContent = `${runs.length} scenario run(s) recorded, ${failed} with a failing trial, ${runs.filter((h) => h.regression).length} regression(s). ` +
        "A failing trial is one where any run broke a limit, whether or not it reached the goal.";
    }).catch(() => { el("historyNote").textContent = "No results yet (or the page is not served from sim/). Run: python3 sim/lab/lab.py regression"; });

    fetch("../lab/progress.json", { cache: "no-store" }).then((r) => r.json()).then((progress) => {
      const table = el("stages");
      table.innerHTML = "<tr><th>Stage</th><th>Status</th><th>Checked by</th></tr>";
      progress.stages.forEach((s) => {
        const row = table.insertRow();
        cell(row, s.stage);
        cell(row, s.status, { done: "good", partly: "warn", blocked: "bad", "not started": "dim" }[s.status]);
        cell(row, s.evidence, "dim");
      });
      el("limits").innerHTML = "";
      progress.limits.forEach((text) => { const li = document.createElement("li"); li.textContent = text; el("limits").append(li); });
    }).catch(() => {});
  }

  // One scenario run in full: its settings, its limits, and every trial with the reasons it failed
  function detail(h) {
    el("detailTitle").textContent = `${h.scenario}, ${h.when.replace("_", " ")}`;
    fetch("../lab/results/" + h.file, { cache: "no-store" }).then((r) => r.json()).then((result) => {
      const lines = [result.what, "", "Settings on top of the defaults: " + (result.sets.join(" ") || "none (the original model)"),
        "Firmware commit: " + result.commit, "Mazes: " + result.maze, ""];
      if (result.summary.failure_rate_95) {
        lines.push(`Failure rate ${(result.summary.failure_rate * 100).toFixed(0)} %; with this many trials the true rate is plausibly ` +
          `${(result.summary.failure_rate_95[0] * 100).toFixed(0)} to ${(result.summary.failure_rate_95[1] * 100).toFixed(0)} % (95 %).`, "");
      }
      result.trials.forEach((t) => {
        lines.push(`Seed ${t.seed}: ${t.failures.length ? "FAILED" : "passed"}`);
        t.failures.forEach((f) => lines.push("    " + f));
      });
      lines.push("", "Over every run of every trial (mean, spread, worst):");
      Object.entries(result.summary).forEach(([name, m]) => {
        if (m && m.mean !== undefined) lines.push(`    ${name.padEnd(28)} ${m.mean.toPrecision(4).padStart(9)}  ${m.sd.toPrecision(3).padStart(8)}  ${Number(m.worst).toPrecision(4).padStart(9)}`);
      });
      lines.push("", "Traces of failed trials are kept in sim/lab/results/traces/ (load one in the Replay tab).");
      el("detail").textContent = lines.join("\n");
    });
  }
})();
