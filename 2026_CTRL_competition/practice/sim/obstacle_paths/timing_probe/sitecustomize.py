"""Temporary externally enabled callback probe; not part of vehicle packages."""
import sys
if sys.argv[0].endswith('/behavior_node'):
    import cProfile,time,functools
    from foscar_planning.behavior_node import BehaviorNode
    def instrument(name):
        original=getattr(BehaviorNode,name);done=False
        @functools.wraps(original)
        def wrapped(self,*args,**kwargs):
            nonlocal done
            s=self.s_global or 0.;profile=None
            if 1860<s<1870 and not done:
                done=True;profile=cProfile.Profile();profile.enable()
            begin=time.perf_counter()
            try:return original(self,*args,**kwargs)
            finally:
                elapsed=time.perf_counter()-begin
                if profile:
                    profile.disable();profile.dump_stats('/tmp/tunnel_'+name+'.prof')
                if elapsed>.08:print('CALLBACK_TIME',name,round(s,2),round(elapsed,4),flush=True)
        setattr(BehaviorNode,name,wrapped)
    instrument('tick');instrument('on_footprints')
