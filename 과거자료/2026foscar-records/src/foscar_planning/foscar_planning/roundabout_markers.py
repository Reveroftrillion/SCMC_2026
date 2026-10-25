"""RViz markers for the rebuilt roundabout entry (/behavior/roundabout_markers).

  ring boxes   FREE green (light green: free by inference), OCCUPIED red, UNKNOWN grey (shadow) / dark grey (past the sight)
  objects      each run as one object, tail to nose across the lane: white seen, grey carried on
  runs         speed range text over each occupied stretch (edge Kalman estimate, km/h)
  gaps         lime (seen free) / yellow (partly hidden) arcs outside the ring: length, opening(+)/closing(-) rate
  bodies       the planner's bodies as arcs: cyan (seen), magenta (the unseen stream upstream)
  target lead  the car followed, outlined: magenta in this scan, grey carried on unseen, none once lost
  wait station a yellow bar across our path
  decision     big text over our car: GO / APPROACH x km/h / WAIT launch in t s / ENTER, and what blocked
"""
import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .roundabout_boxes import FREE, OCCUPIED


def roundabout_markers(planner, plan_info, ego_xy, ego_yaw, route, s_offset, stamp, boxes_only=False):
    arr = MarkerArray()
    clear = Marker(); clear.action = Marker.DELETEALL; arr.markers.append(clear)
    if planner is None:
        return arr
    lane = planner.lane

    def marker(kind, ns, color, alpha=.9):
        m = Marker(); m.header.frame_id = 'map'; m.header.stamp = stamp; m.ns = ns; m.id = len(arr.markers)
        m.type = kind; m.action = Marker.ADD; m.pose.orientation.w = 1.
        m.color.r, m.color.g, m.color.b = color; m.color.a = alpha
        arr.markers.append(m)
        return m

    def arc(u0, u1, z, n=6):
        return [Point(x=float(p[0]), y=float(p[1]), z=z) for p in (lane.xy_of(u0+(u1-u0)*k/n) for k in range(n+1))]

    def text(xy, value, color, size=.9, z=2.):
        m = marker(Marker.TEXT_VIEW_FACING, 'roundabout_text', color, 1.)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(xy[0]), float(xy[1]), z
        m.scale.z = size; m.text = value

    colors = {FREE: (.2, .8, .3), OCCUPIED: (1., .15, .15)}
    for u0, u1, state, key in lane.boxes:
        c = colors.get(state, (.35, .35, .35) if key in ('RANGE', 'ISLAND') else (.6, .6, .6))
        if boxes_only and key == 'inferred':
            c = (.6, 1., .6)                         # hidden, but no car can be there: free by inference
        if boxes_only:
            # like the highway boxes: each box fills the lane, a thin gap between boxes
            m = marker(Marker.TRIANGLE_LIST, 'roundabout_boxes', c, .45 if state == FREE else .75)
            m.scale.x = m.scale.y = m.scale.z = 1.
            cc, R, hw = lane.circle.center, lane.circle.radius, lane.lane_half

            def at(u, r):
                p = lane.xy_of(u); f = (R+r)/R
                return Point(x=float(cc[0]+(p[0]-cc[0])*f), y=float(cc[1]+(p[1]-cc[1])*f), z=.05)
            a, b = u0+.04, u1-.04
            q = [at(a, -hw), at(b, -hw), at(b, hw), at(a, hw)]
            # both windings: one of them faces up whichever way the ring turns (a face seen from behind is drawn black)
            m.points = [q[0], q[1], q[2], q[0], q[2], q[3], q[0], q[2], q[1], q[0], q[3], q[2]]
            continue
        m = marker(Marker.LINE_STRIP, 'roundabout_boxes', c, .55 if state == FREE else .9)
        m.scale.x = .5 if state != FREE else .25
        m.points = arc(u0+.05, u1-.05, .1, 2)
    if boxes_only:
        # the gap we chose (tuck-in practice): orange band from the rear car's front to the lead's tail, the lead outlined
        view = getattr(planner, 'tuck_view', None)
        if view and not view.get('lead') and view.get('state') in ('GO', 'FOLLOW'):
            view = dict(view, lead=None)
        if view:
            cc, R = lane.circle.center, lane.circle.radius

            def ring_pts(u0, u1, r_off, z, n=10):
                out = []
                for k in range(n+1):
                    p = lane.xy_of(u0+(u1-u0)*k/n); f = (R+r_off)/R
                    out.append(Point(x=float(cc[0]+(p[0]-cc[0])*f), y=float(cc[1]+(p[1]-cc[1])*f), z=z))
                return out
            lead, rear = view.get('lead'), view.get('rear')
            # The outline is the car the logic acts on, as it perceives it now (user 2026-10-08: a magenta outline stayed on
            # an empty ring): magenta when that car is in this scan, grey while it is carried on unseen at its own measured
            # speed, none once no run or marked car stands there (lost).
            seen = carried = False
            if lead is not None:
                over = lambda lo, hi: min(hi, lead[1])-max(lo, lead[0]) > 0.
                seen = any(r.coasted == 0. and over(r.lo, r.hi) for r in lane.runs)
                mk = getattr(planner, 'marks', {}).get('lead')
                run = mk.get('run') if mk else None
                carried = not seen and (run is not None and over(run.lo, run.hi)
                                        or any(r.coasted > 0. and over(r.lo, r.hi) for r in lane.runs))
                if not (seen or carried):
                    lead = None
            if lead is not None:
                lo = rear[1] if rear is not None else max(lane.edges[0], lead[0]-15.)
                if lead[0] > lo:
                    m = marker(Marker.LINE_STRIP, 'roundabout_target_gap', (1., .55, 0.), 1.)
                    m.scale.x = .6
                    m.points = ring_pts(lo, lead[0], lane.lane_half+.8, .4)
                m = marker(Marker.LINE_STRIP, 'roundabout_target_lead', (1., 0., 1.) if seen else (.55, .55, .55), 1.)
                m.scale.x = .3
                hw = lane.lane_half
                m.points = ring_pts(lead[0], lead[1], -hw, .5, 4)+ring_pts(lead[1], lead[0], hw, .5, 4)+ring_pts(lead[0], lead[0], -hw, .5, 1)[:1]
            # launch state as a light over our car (no text): red no launch fits, orange waiting, green go / clear,
            # light green a provisional go (the ring we need not yet in sight), blue following; the bar under it shrinks with the time to the launch (1 m per s)
            state = view.get('state'); tau = view.get('tau')
            if state == 'WAIT':
                col = (1., .1, .1) if tau is None else (1., .55, 0.)
            else:
                col = {'CLEAR': (.1, 1., .2), 'GO': (.1, 1., .2), 'FOLLOW': (.2, .5, 1.),
                       'TENTATIVE': (.7, .95, .4)}.get(state, (.6, .6, .6))
            if ego_xy is not None:
                m = marker(Marker.SPHERE, 'roundabout_launch_light', col, 1.)
                m.scale.x = m.scale.y = m.scale.z = 1.2
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(ego_xy[0]), float(ego_xy[1]), 3.
                if state == 'WAIT' and tau is not None and tau > 0.:
                    m = marker(Marker.LINE_STRIP, 'roundabout_launch_bar', (1., .55, 0.), 1.)
                    m.scale.x = .3
                    m.points = [Point(x=float(ego_xy[0])-tau/2, y=float(ego_xy[1]), z=2.),
                                Point(x=float(ego_xy[0])+tau/2, y=float(ego_xy[1]), z=2.)]
        # runs the scan lost but the edge filter carries on (coasted): occupied too
        # each object as we hold it: from its tail to its nose, the lane's width -- white when seen this scan, grey
        # when carried on behind a shadow or the island
        cc, R, hw = lane.circle.center, lane.circle.radius, lane.lane_half

        def edge(u0, u1, r_off, n=6):
            out = []
            for k in range(n+1):
                p = lane.xy_of(u0+(u1-u0)*k/n); f = (R+r_off)/R
                out.append(Point(x=float(cc[0]+(p[0]-cc[0])*f), y=float(cc[1]+(p[1]-cc[1])*f), z=.3))
            return out
        for r in lane.runs:
            m = marker(Marker.LINE_STRIP, 'roundabout_objects', (.55, .55, .55) if r.coasted else (1., 1., 1.), 1.)
            m.scale.x = .2
            m.points = edge(r.lo, r.hi, -hw)+edge(r.hi, r.lo, hw)+edge(r.lo, r.lo, -hw, 1)[:1]
        return arr
    for r in lane.runs:
        est = r.speed
        label = 'v ?' if est is None else f'{est[0]*3.6:.0f}+-{planner.p.kf_sigmas*est[1]*3.6:.0f} km/h'
        text(lane.xy_of((r.lo+r.hi)/2), label, (1., .6, .6), .7, 1.6)
    # gaps: lime when every box in it is seen free, yellow when part of it is hidden; length and how fast it closes
    for gp in lane.gaps():
        if gp.length < .5:
            continue
        m = marker(Marker.LINE_STRIP, 'roundabout_gaps', (.6, 1., .1) if gp.sure else (1., .85, .1), .9)
        m.scale.x = .35
        R = lane.circle.radius
        m.points = [Point(x=float(lane.circle.center[0]+(p.x-lane.circle.center[0])*(R+2.6)/R),
                          y=float(lane.circle.center[1]+(p.y-lane.circle.center[1])*(R+2.6)/R), z=.3)
                    for p in arc(gp.lo, gp.hi, .3, 10)]
        closing = ''
        if gp.rear_speed and gp.front_speed:
            closing = f' {(gp.front_speed[0]-gp.rear_speed[0])*3.6:+.0f} km/h'
        mid = lane.xy_of((gp.lo+gp.hi)/2); cc = lane.circle.center
        text((cc[0]+(mid[0]-cc[0])*(R+4.)/R, cc[1]+(mid[1]-cc[1])*(R+4.)/R),
             f'gap {gp.length:.1f} m{closing}' + ('' if gp.sure else ' ?'), (.6, 1., .1) if gp.sure else (1., .85, .1), .7, .8)
    try:
        bodies = planner.bodies(0., 0.)
    except Exception:
        bodies = []
    for b in bodies:
        lo = max(b.lo, lane.edges[0])
        if b.hi <= lo:
            continue
        m = marker(Marker.LINE_STRIP, 'roundabout_bodies', (1., .2, 1.) if b.name == 'unseen' else (.1, .9, 1.), .8)
        m.scale.x = .15
        m.points = arc(lo, b.hi, .5, 8)
    # wait station
    s = planner.stop_s+s_offset
    x, y = route.at(s); h = route.heading_at(s)
    m = marker(Marker.LINE_STRIP, 'roundabout_wait', (1., .9, .1), 1.)
    m.scale.x = .3
    m.points = [Point(x=x-1.6*math.sin(h), y=y+1.6*math.cos(h), z=.2), Point(x=x+1.6*math.sin(h), y=y-1.6*math.cos(h), z=.2)]
    # decision
    info = plan_info or {}
    d = info.get('decision', '')
    if d == 'APPROACH':
        label = f"APPROACH {info.get('approach_mps', 0)*3.6:.0f} km/h"
    elif d == 'WAIT':
        w = info.get('wait_s')
        label = 'WAIT' + (f' launch in {w:.1f} s' if isinstance(w, (int, float)) and math.isfinite(w) else '')
    elif d == 'ENTER':
        label = f"ENTER keep gap {info.get('keep_gap_mps', 0)*3.6:.0f} km/h"
    else:
        label = d
    color = {'GO': (.2, 1., .3), 'ENTER': (.2, 1., .3), 'APPROACH': (1., .9, .2)}.get(d, (1., .4, .3))
    if ego_xy is not None:
        text(ego_xy, label, color, 1.3, 3.)
        blocked = info.get('blocked') or []
        if blocked:
            b = blocked[0]
            text(ego_xy, f'blocked by {b[3]} at +{b[2]} s', (1., .7, .5), .7, 4.3)
    return arr
