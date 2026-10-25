# BEV Occupancy Implementation Plan

> Use executing-plans and TDD. Independent pure grid and pure motion modules can be delegated; root owns integration and MORAI.

**Goal:** First static mission uses occupancy shape and predicted motion without semantic labels.
**Architecture:** Reuse RingLane visibility concepts and EdgeTracks Kalman estimation in a two-dimensional grid. Keep occupancy, motion, and candidate validation independently testable.
**Spec:** docs/superpowers/specs/2026-10-06-bev-avoidance-design.md
**Constraints:** Preserve prior edits, no commit/push, no practice references in production, short checks then MORAI; tunnel behavior stays on previous implementation.

- [x] Add pure convex-footprint BEV rasterization with occupied/free/unknown and shadow tests.
- [x] Add occupancy-region motion tracking using EdgeTracks, including stationary jitter and lateral crossing tests.
- [x] Integrate complete scans and occupancy collision checks into first mission and visible grid markers.
- [x] Verify timing, do direct MORAI first-mission run, record failures as well as progress.

Validation completed, mission acceptance NOT achieved: MORAI 174505 stopped at s311.4; region split/merge uncertainty causes repeated braking. Tunnel remains unchanged. Latest focused tests:29 passed; target gaps >0.5s:0 in s270–400. See practice/HANDOFF.md for limitations.
