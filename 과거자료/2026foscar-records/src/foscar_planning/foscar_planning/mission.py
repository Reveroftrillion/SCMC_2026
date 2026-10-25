"""Route checkpoint ownership, independent of lower FSM completion (no ROS)."""
import math
from dataclasses import dataclass, replace


@dataclass(frozen=True)
class MissionPolicy:
    name: str
    allowed_states: frozenset
    speed_mps: float
    pedestrian: bool = False
    roundabout: bool = False
    signal: bool = False          # camera traffic signal needed (YOLO runs only here)
    after_pedestrian_yield_mps: float = None
    start_s: float = -math.inf
    end_s: float = math.inf
    limit_start_s: float = None   # checkpoint start when the mission takes over earlier (approach_s)
    pedestrian_seen_mps: float = None   # cap while a person is seen ahead (tunnel, user 2026-10-10)

    @classmethod
    def parse(cls, row, zone=False):
        required = {'name', 'allowed_states', 'speed_kmh'} | ({'start_s', 'end_s'} if zone else set())
        allowed = required | {'pedestrian', 'roundabout', 'signal', 'after_pedestrian_yield_kmh', 'limit_start_s',
                              'pedestrian_seen_kmh'}
        if not isinstance(row, dict) or not required <= row.keys() or row.keys()-allowed:
            raise ValueError('mission: missing or unknown policy fields')
        if not isinstance(row['name'], str) or not row['name']:
            raise ValueError('mission: name must be nonempty')
        states = row['allowed_states']
        if not isinstance(states, list) or any(not isinstance(s,str) or not s for s in states):
            raise ValueError('mission: allowed_states must be state names')
        speed = row['speed_kmh']
        if speed is not None and (isinstance(speed,bool) or not isinstance(speed,(float,int)) or not math.isfinite(speed) or speed<0):
            raise ValueError('mission: speed_kmh must be nonnegative or null (unlimited)')
        after = None
        if 'after_pedestrian_yield_kmh' in row:
            value=row['after_pedestrian_yield_kmh']
            if not row.get('pedestrian',False):
                raise ValueError('mission: after-yield speed requires pedestrian capability')
            if value is not None and (isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or value<0):
                raise ValueError('mission: invalid after-yield speed')
            after=math.inf if value is None else value/3.6
        seen = row.get('pedestrian_seen_kmh')
        if seen is not None and (not row.get('pedestrian',False) or isinstance(seen,bool)
                                 or not isinstance(seen,(float,int)) or not math.isfinite(seen) or seen<0):
            raise ValueError('mission: invalid pedestrian-seen speed (needs pedestrian capability)')
        a,b = (row['start_s'],row['end_s']) if zone else (-math.inf,math.inf)
        if zone and (any(isinstance(x,bool) or not isinstance(x,(float,int)) or not math.isfinite(x) for x in (a,b)) or a>=b):
            raise ValueError('mission: invalid checkpoint interval')
        for flag in ('pedestrian','roundabout','signal'):
            if flag in row and not isinstance(row[flag],bool):
                raise ValueError('mission: capabilities must be booleans')
        return cls(row['name'],frozenset(states),math.inf if speed is None else speed/3.6,
                   row.get('pedestrian',False),row.get('roundabout',False),row.get('signal',False),after,a,b,
                   row.get('limit_start_s'),None if seen is None else seen/3.6)


class MissionSupervisor:
    def __init__(self, config):
        if not isinstance(config,dict) or set(config)!={'default','zones'} or not isinstance(config['zones'],list):
            raise ValueError('mission: expected default and zones')
        self.default=MissionPolicy.parse(config['default'])
        self.zones=sorted((MissionPolicy.parse(row,True) for row in config['zones']),key=lambda p:p.start_s)
        names=[self.default.name]+[p.name for p in self.zones]
        if len(names)!=len(set(names)) or 'LOCALIZATION_INVALID' in names:
            raise ValueError('mission: duplicate or reserved name')
        if any(a.end_s>b.start_s for a,b in zip(self.zones,self.zones[1:])):
            raise ValueError('mission: overlapping intervals')
        self.invalid=MissionPolicy('LOCALIZATION_INVALID',frozenset(),0.)
        self.reset()

    def reset(self):
        self.pedestrian_yield_seen=False
        self.pedestrian_yield_completed=False
        self.policy=None
        self.interval=None
        self.progress=None
        self.previous=None
        self.reset_observations=True
        self.reason='initial localization';self.reset_detail=''

    def update(self, ctx):
        """True means reset every lower state, even one allowed in both policies."""
        old=self.policy
        old_interval=self.interval
        self.reset_observations=False
        if not all(math.isfinite(x) for x in (ctx.s,ctx.now,ctx.v,ctx.x,ctx.y,ctx.yaw)):
            self.pedestrian_yield_seen=self.pedestrian_yield_completed=False
            self.policy=self.invalid
            self.interval=None
            self.previous=self.progress=None
            self.reset_observations=True
            self.reason='invalid localization'
            return old!=self.policy
        if self.previous is not None:
            s,t,v=self.previous
            dt=ctx.now-t
            # Ignore sub-meter projection jitter; genuine relocation starts a new run.
            if dt<0 or ctx.s<self.progress-5. or ctx.s-s>max(10.,max(abs(v),abs(ctx.v))*max(0.,dt)+10.):
                progress,self.progress=self.progress,None
                self.reset_observations=True
                self.reason='localization reset'
                self.reset_detail=f's {s:.1f}->{ctx.s:.1f}, progress {progress:.1f}, dt {dt:.3f}, v {v:.1f}/{ctx.v:.1f}'
        if self.progress is None:
            self.progress=ctx.s
        else:
            self.progress=max(self.progress,ctx.s)
        self.previous=(ctx.s,ctx.now,ctx.v)
        self.policy=next((p for p in self.zones if p.start_s<=self.progress<p.end_s),self.default)
        # Uncovered intervals have the same policy but distinct lifecycle ownership.
        self.interval=(self.policy.name, sum(p.end_s<=self.progress for p in self.zones))
        changed=old_interval!=self.interval or self.reset_observations
        if changed:
            self.pedestrian_yield_seen=self.pedestrian_yield_completed=False
        if changed and not self.reset_observations:
            self.reason='checkpoint handoff' if old else 'initial localization'
        return changed

    @property
    def speed_mps(self):
        # taken over before its checkpoint (approach_s): the cap starts at the checkpoint (limit_start_s)
        if self.policy.limit_start_s is not None and (self.progress is None or self.progress < self.policy.limit_start_s):
            return math.inf
        if self.pedestrian_yield_completed and self.policy.after_pedestrian_yield_mps is not None:
            return self.policy.after_pedestrian_yield_mps
        if getattr(self,'pedestrian_ahead',False):
            return min(self.policy.speed_mps,self.policy.pedestrian_seen_mps)
        return self.policy.speed_mps

    def observe_pedestrian_yield(self, *, active, pedestrian, verified_clear):
        """Unlock the preparation cap after an actual yield and observed passage.

        The capability remains enabled: a new person can still demand a stop.
        Missing observations alone do not complete the speed-policy handshake.
        """
        if not self.policy.pedestrian:
            return False
        if active and pedestrian:
            self.pedestrian_yield_seen=True
        completed=(self.pedestrian_yield_seen and not active and verified_clear)
        changed=completed and not self.pedestrian_yield_completed
        if completed:self.pedestrian_yield_completed=True
        return changed

    VEHICLE_MIN_SIZE_M = 2.0   # obstacle_node vehicle_min_size_m: smaller bodies were never sized as a vehicle

    PEDESTRIAN_SEEN_M = 40.   # the lidar's range ahead

    def apply_context(self, ctx):
        # A person seen ahead (on the path or beside it) slows the mission down to pedestrian_seen_kmh until passed
        self.pedestrian_ahead=self.policy.pedestrian_seen_mps is not None and any(
            e.type=='PEDESTRIAN' and 0.<e.s-ctx.s<=self.PEDESTRIAN_SEEN_M
            for e in list(ctx.events)+list(getattr(ctx,'observations',[]) or []))
        ctx.mission_name=self.policy.name
        ctx.mission_speed_mps=self.speed_mps
        ctx.mission_pedestrian=self.policy.pedestrian
        ctx.mission_end_s=self.policy.end_s
        # No NPC vehicles appear in the pedestrian mission. A VEHICLE label on a body now smaller than any vehicle
        # (the lidar's vehicle_min_size, 2.0 m) is a person whose label stuck from one sidewalk frame merged with
        # something: it crossed 4 m ahead of us and we drove on at 20 km/h (2026-09-30 11:10 lap). Treat it as the
        # pedestrian it is. A car-sized VEHICLE label there (a static barricade) is still not a pedestrian.
        if self.policy.pedestrian and self.policy.name != 'TUNNEL':
            ctx.events=[replace(e,type='PEDESTRIAN') if e.type=='VEHICLE' and max(e.size_x,e.size_y)<self.VEHICLE_MIN_SIZE_M
                        else e for e in ctx.events]

    def limit(self, target):
        return replace(target,v_max=min(target.v_max,self.speed_mps))
