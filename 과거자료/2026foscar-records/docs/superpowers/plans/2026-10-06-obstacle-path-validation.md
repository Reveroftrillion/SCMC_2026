# Multi-obstacle Path Validation Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans for integration and superpowers:test-driven-development for each change.

**Goal:** Preserve observed obstacle pieces and publish only candidate paths checked against every known static obstacle and mapped pavement.

**Architecture:** Reuse the tunnel swept-body geometry and candidate chain for static avoidance. Keep collision feasibility separate from ranking; preserve a safe committed path and brake if no path is feasible. Perception changes require evidence of filtering or extent loss.

**Tech Stack:** Python, NumPy, ROS 2 Humble, pytest, MORAI.

**Spec:** User-approved design in conversation: observed occupied space, candidate curves, whole-body entry/hold/return validation, feasible candidates ranked by progress/steering/path continuity. Tunnel has only static vehicles; people, children and deer are PEDESTRIAN.

## Global Constraints

- Preserve existing uncommitted work; no commit/push.
- No new arbitrary speed caps, route coordinates, or waiting timers.
- Production code must not consume practice recordings or simulator truth.
- Keep signal-line lateral hold and isolated ROS test domains.
- No camera changes. Simulator execution owned by root only.

## Review Focus

- Perforated and fragmented objects must not create a body-sized false opening.
- Safe endpoint offsets must not hide a colliding entry curve.
- Road-boundary rejection must happen before candidate selection/publication.
- New observations invalidate a committed curve even if object count is unchanged.
- Returning to route must not cut through remaining or occluded obstacles.

## Tasks

- [x] Reproduce lidar omission/extent failures; repair evidence-backed filters with regression tests (fragment_detection).
- [x] Add planner regressions for candidate gaps, body sweep endpoints and feasible full chains; repair pure geometry (sweep_candidates).
- [x] Integrate shared full-path checks into first static mission, retaining signal-line hold and safe commitment (root).
- [x] Run relevant suites and planner timing checks; fix regressions.
- [x] Run difficult scenario in MORAI, inspect logs/contacts, and record actual limitations.

## Validation outcome

Implementation and requested simulator checks performed. Safe difficult-scenario completion is NOT achieved: static mission stops at s309.7; tunnel collision ID13 at s1877.76. Follow-up work remains for candidate feasibility and dynamic braking along the selected path. Full regression suite: 776 passed, 118.14s. See practice/sim/obstacle_paths/results.json.
