"""Simulator-specific following response; an assumption, not observed braking.

Input trace is ego's rear occupancy boundary after entering the circulating
lane. No reaction is assumed before that first overlap plus reaction_s.
"""


def predict_rear_response(position,speed,front_extent,trace,reaction_s=.4,decel=2.5):
    if not trace:return {}
    free_speed=max(0.,speed);v=free_speed;s=position;previous=0.;out={}
    reaction_at=trace[0][0]+reaction_s
    for i,(dt,ego_rear) in enumerate(trace):
        elapsed=max(0.,dt-previous)
        if previous>=reaction_at and i:
            delayed=[p for p in trace[:i+1] if p[0]<=previous-reaction_s+1e-6]
            seen=delayed[-1] if delayed else trace[0]
            if len(delayed)>1:
                ego_speed=max(0.,(delayed[-1][1]-delayed[-2][1])/max(.01,delayed[-1][0]-delayed[-2][0]))
            else:ego_speed=0.
            seen_rear=out.get(seen[0],dict(s=position,speed=free_speed))
            gap=seen[1]-(seen_rear['s']+front_extent)
            desired=2.+.3*seen_rear['speed']
            wanted=max(0.,min(free_speed,ego_speed+.6*(gap-desired)))
            new_v=max(0.,min(v+elapsed,max(v-decel*elapsed,wanted)))
            s+=(v+new_v)*.5*elapsed;v=new_v
        else:s+=v*elapsed
        out[dt]=dict(s=s,speed=v)
        previous=dt
    return out


def handoff_reserve(position,speed,front_extent,trace,reaction_s=.4,decel=2.5,uncertainty_mps=.35):
    """Minimum clearance after stable insertion under explicit NPC following.

    trace starts at the handoff (relative t=0), not initial entry. Thus rear
    response is not used to justify clearance during the insertion itself.
    front_extent already includes uncertainty at handoff; grow it thereafter.
    """
    if len(trace)<2 or abs(trace[0][0])>1e-6 or trace[-1][0]<2.-1e-6:
        return -float('inf')
    response=predict_rear_response(position,speed,front_extent,trace,reaction_s,decel)
    return min(ego_rear-response[t]['s']-front_extent-uncertainty_mps*t for t,ego_rear in trace)
