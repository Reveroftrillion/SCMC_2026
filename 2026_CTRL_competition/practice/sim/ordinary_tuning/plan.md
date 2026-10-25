# Ordinary driving tuning — 2026-10-06

User request: release each signal immediately once fully stopped at its line;
improve ordinary driving START–CP10 and CP13–END using offline and MORAI trials;
repair the six known offline planning test failures. No commits requested.

## Design and acceptance

- Keep the existing linked worktree `roundabout-gaps` and existing unrelated files.
- Measure baseline with `signal_director --hold-s 0 --hold-max 0 --release-m 0 --near-m 0.5`.
  The existing stop detector requires |v| < 0.05 m/s within 0.5 m of the rear-axle stop target.
  Straight intersections get green; the left-turn intersection gets its legal left arrow.
- Diagnose and repair tests independently; distinguish obsolete fixtures from production defects.
- Fit longitudinal response from logged commands/status, prototype simple feedback/feedforward
  candidates and test lag/gain uncertainty offline. Do not fit to NPC timing or add route-specific boosts.
- Root exclusively owns controller edits and simulator execution. Freeze code during each run.
- Safety: zero measured ordinary-zone speed >60 km/h; no collision/contact; no increase in
  cross-track/heading excursions, stop overshoot, or tick gaps. Preserve highway tuning.
- Compare section times and signal/pedestrian waiting separately. A faster full lap alone is insufficient.
- Test 1–2 runs per candidate; retain only improvements supported by both analysis and MORAI evidence.

## Execution checklist

- [x] Inspect baseline controller and scenario tools; verify linked worktree.
- [x] Replace the running random signal director with stop-triggered release.
- [x] Record unchanged-controller baseline with the new signal condition.
- [x] Reproduce and fix the six planning test failures, then full planning suite.
- [x] Identify longitudinal model; offline candidate/uncertainty comparison.
- [x] Add regression tests before implementing the chosen minimal controller change.
- [x] Run control suite, then MORAI candidate comparisons and safety analysis.
- [x] Review final diff, full relevant suites, submission separation check; summarize evidence/limits.

## Progress

- Baseline HEAD 74c33ff. Existing pass_records.jsonl changed by the previous authorized lap;
  existing hw_lat_sweep/ and lat_sweep/ are untouched.
- Replaced old director PID 2016882. New director session 73119, baseline runner session 7445.
- Planning repair delegated to fix_six_tests; offline identification to offline_speed.
- Authorization: user explicitly requested tuning and test fixes, so execute the scoped work
  without further design/implementation approval. No commit/push of code.

- Baseline 145202: PASS174.0s; ordinary pre105.746/post28.700s; 60.186km/h at939m under old boost.
- Candidate A145602: PASS162.2s; pre104.180/post28.987s; max59.605km/h. Its 10.55s highway gain is traffic-dependent and is NOT attributed to ordinary tuning.
- Installed packages/config were copies, contrary to handoff. Candidate A used new Python via PYTHONPATH but OLD boost setting; rebuilt control/planning/bringup with --symlink-install after A finished. Config diffs before build were only intended boost change.
- Review found predictor reset + hold-last-throttle overspeed across missing feedback/position jumps. New regression failed5 cases; ordinary-only coasting and preserved trend fixes them.
- Planning541 passed, production planning unchanged. Six failures were obsolete fixtures/expectations.
- Submission check has three pre-existing comment/docstring references to practice in lidar/obstacle_node.py, planning/race_hud_node.py, planning/highway_box_tracks.py; outside this task, left unchanged.

Final evidence and adoption limits: RESULTS.md. Latest151922 PASS169.6, ordinary135.797vs134.446 baseline; free-cruise improvement observed but overall ordinary lap gain NOT established. Latest control110passed; unchanged planning541passed. Failed151136 tunnelstop retained in evidence. No codecommit/push.
