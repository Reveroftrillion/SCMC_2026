"""Explicit fixed-course prior for the supplied stationary SCMC box.

Geometry source: supplied 2026_molit_comp_sample_scene.json, object UNIQUEID 2,
position (-60.610, -142.178), yaw -83.219 degrees, scale (2, 3, 2).
This is a planning prior, never a fabricated sensor observation. Actual BEV
occupancy and the signal policy retain braking authority.
"""
import hashlib
import math

from .ctrl_state import Target
from .ctrl_mapped_avoidance import paved_moves
from .ctrl_tunnel_slalom import first_hit


ROUTE_POINTS_SHA256 = 'c464e34356f76eda8d9901cdbdb7b88f324fab0bd0caab6bb69cf7526077e0ab'
ROUTE_POINTS = 4406  # Route removes duplicate vertices; follower may append finish runout.
# Right is negative Frenet d. Enter inside the intersection pavement and
# return before its right-hand boundary narrows; full body checked at runtime.
MOVES = ((-3.5, 316., 12.), (0., 331., 12.))


class StaticCourse:
    def __init__(self):
        self.route = None
        self.matches = False
        self.checked = None
        self.paved = False
        self.active = False
        self.reason = ''

    def target(self, ctx, avoid):
        if ctx.mission_name != 'STATIC_OBSTACLE' or ctx.route is None:
            return None
        if self.route is not ctx.route:
            self.route = ctx.route
            self.matches = (len(ctx.route.points) >= ROUTE_POINTS and
                            hashlib.sha256(ctx.route.points[:ROUTE_POINTS].tobytes()).hexdigest() == ROUTE_POINTS_SHA256)
            self.active = False
            self.checked = None
        if not self.matches or abs(ctx.s_offset) > .05:
            return None
        # Keep the calibrated waiting pose straight before the current red line.
        # The next line behind this box cannot delay the fixed manoeuvre.
        if not self.active and math.isfinite(ctx.lateral_hold_s) and ctx.s < ctx.lateral_hold_s < 326.35:
            return None
        if not self.active and ctx.s > MOVES[0][1]+1.:
            self.reason = 'fixed course entry already passed'
            return Target(state=avoid.name, v_max=0.)
        self.active = True
        half = avoid.path_planner(ctx).half
        key = (id(ctx.road_map), avoid.front_m, avoid.rear_overhang_m, half)
        if key != self.checked:
            self.checked = key
            self.paved = paved_moves(ctx, 0., MOVES, 290., 370.,
                                     avoid.front_m, avoid.rear_overhang_m, half)
        # Publish one stable complete chain; do not restart its origin every tick.
        avoid.slalom = None
        avoid.slalom_origin = avoid.start_offset = 0.
        avoid.prev_d = MOVES[0][0]
        avoid.entry_start_s, avoid.entry_length_m = MOVES[0][1:]
        avoid.transition_end_s = sum(MOVES[0][1:])
        avoid.return_start_s, avoid.return_length_m = MOVES[1][1:]
        avoid.lateral_started = True
        cap = math.inf
        previous = 0.
        for offset, start, length in MOVES:
            if ctx.s < start+length:
                fit = avoid.swerve_speed(length, abs(offset-previous))
                cap = min(cap, avoid.approach_speed(ctx, fit, start-ctx.s))
            previous = offset
        hit = first_hit(avoid.boxes(), MOVES, 0., ctx.s, max(ctx.s, 360.),
                        avoid.front_m, avoid.rear_overhang_m, half)
        self.reason = ('outside mapped pavement' if not self.paved else
                       'observed box conflicts with fixed course' if hit is not None else 'fixed SCMC box course')
        if not self.paved or hit is not None:
            cap = 0.
        return Target(state=avoid.name, lat_offset=MOVES[0][0], v_max=cap,
                      entry_s=MOVES[0][1], entry_length_m=MOVES[0][2],
                      return_s=MOVES[1][1], return_length_m=MOVES[1][2], chain=MOVES)
