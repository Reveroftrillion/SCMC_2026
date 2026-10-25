#!/usr/bin/env python3
"""Export derived lane chains and checkpoint geometry without controlling MORAI."""
import argparse
import json
from pathlib import Path
from foscar_planning.highway_route import HighwayRoute
from foscar_planning.highway_config import HighwayConfig


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--link-file',required=True);ap.add_argument('--route-file',required=True)
    ap.add_argument('--checkpoint-file',required=True);ap.add_argument('--config',required=True)
    ap.add_argument('--output',required=True);ap.add_argument('--plot')
    a=ap.parse_args();c=HighwayConfig.load(a.config)
    h=HighwayRoute.from_files(a.link_file,a.route_file,a.checkpoint_file,**c.route)
    result=dict(start_link=h.start_link,chains=h.chains,rejoin_s=h.rejoin_s,end_s=h.end_s,
                checkpoints={k:dict(xy=v['xy'].tolist(),s=v['s'],radius=v['radius']) for k,v in h.checkpoints.items()},
                deadlines={lane:h.change_deadline(lane,c.vehicle.width_m+2*c.policy.margin_m) for lane in h.lanes})
    Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax=plt.subplots(figsize=(7,11))
        for lane,t in h.tables.items():ax.plot(t[:,1],t[:,2],label=f'lane {lane}')
        mask=(h.original.s>=h.rejoin_s)&(h.original.s<=h.end_s)
        ax.plot(h.original.points[mask,0],h.original.points[mask,1],label='original route after CP12',color='black',linestyle='--')
        for n in (h.start_checkpoint,h.hold_checkpoint,h.rejoin_checkpoint,h.end_checkpoint):
            v=h.checkpoints[n];ax.scatter(*v['xy'],color='black');ax.annotate(f'CP{n}',v['xy'])
        ax.set_xlabel('map x [m]');ax.set_ylabel('map y [m]');ax.legend();ax.grid();fig.tight_layout();fig.savefig(a.plot)
    print('Derived lanes:',h.lanes,'rejoin s:',round(h.rejoin_s,2),'highway end s:',round(h.end_s,2))

if __name__=='__main__':main()
