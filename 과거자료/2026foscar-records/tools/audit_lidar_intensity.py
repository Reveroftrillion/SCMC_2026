#!/usr/bin/env python3
"""Read CLOSED bags: height/intensity evidence in route Frenet cells.

Diagnostic only. Ground hits do not certify an empty vehicle volume. Route
projection is limited to the selected station window to avoid other laps.
No control publisher, semantic thresholds, or tracking deletion is used.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from scipy.spatial import cKDTree
from sensor_msgs_py import point_cloud2 as pc2
from analyze_pedestrian import messages
from foscar_lidar.obstacle_node import ObstacleNode, stamp_to_sec
from foscar_lidar.ground import estimate_ground_plane, relative_height


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag');parser.add_argument('output')
    parser.add_argument('--s-min',type=float,default=860.)
    parser.add_argument('--s-max',type=float,default=960.)
    args=parser.parse_args()
    rclpy.init()
    node=ObstacleNode(parameter_overrides=[Parameter('use_tf',value=False),Parameter('publish_markers',value=False)])
    cells=defaultdict(lambda:dict(ground=Counter(),above=Counter(),ground_frames=set(),above_frames=set()))
    counts=Counter();tree=None
    try:
        for _,name,msg in messages(args.bag,{'/sensors/lidar/points','/localization/odometry','/map/route'}):
            if name=='/localization/odometry':node.on_odom(msg);continue
            if name=='/map/route':
                node.on_route(msg)
                indices=np.flatnonzero((node.route_s[:-1]>=args.s_min)&(node.route_s[:-1]<=args.s_max))
                if len(indices):tree=cKDTree(node.route_xy[indices])
                continue
            counts['clouds']+=1
            if tree is None or 'intensity' not in {f.name for f in msg.fields}:
                counts['missing_route_or_intensity']+=1;continue
            t=stamp_to_sec(msg.header.stamp)
            if node.ego_pose_at(t) is None:counts['missing_pose']+=1;continue
            raw=pc2.read_points_numpy(msg,field_names=('x','y','z','intensity'),skip_nans=True)
            pts=node.lidar_to_base(raw[:,:3].copy(),msg.header.frame_id,node.get_clock().now())
            plane=estimate_ground_plane(pts)
            if plane is None:counts['missing_ground_plane']+=1;continue
            height=relative_height(pts,plane)
            # Use only the supported road-plane region and range, before ground removal.
            valid=(pts[:,0]>3)&(pts[:,0]<28)&(abs(pts[:,1])<5)&np.isfinite(raw[:,3])
            pts=pts[valid];height=height[valid];intensity=raw[valid,3]
            xy=node.to_map(pts,t)
            _,near=tree.query(xy,k=min(4,len(indices)))
            near=np.asarray(near).reshape(len(xy),-1)
            seg=indices[near];a=node.route_xy[seg];v=node.route_xy[seg+1]-a
            lens=np.linalg.norm(v,axis=2);delta=xy[:,None,:]-a
            u=np.clip(np.sum(delta*v,axis=2)/np.maximum(lens*lens,1e-9),0,1)
            residual=delta-u[:,:,None]*v
            best=np.argmin(np.sum(residual*residual,axis=2),axis=1);row=np.arange(len(xy))
            station=node.route_s[seg[row,best]]+u[row,best]*lens[row,best]
            d=(v[row,best,0]*residual[row,best,1]-v[row,best,1]*residual[row,best,0])/np.maximum(lens[row,best],1e-9)
            for s,lat,z,value in zip(station,d,height,intensity):
                if abs(lat)>2.:continue
                kind='ground' if abs(z)<=.12 else 'above' if .4<=z<=1.8 else None
                if kind is None:continue
                cell=cells[(int(np.floor(s)),int(np.floor(lat/.5)))]
                cell[kind][float(value)]+=1;cell[kind+'_frames'].add(counts['clouds'])
                counts[kind+'_points']+=1
    finally:node.destroy_node();rclpy.shutdown()
    rows=[]
    for (s,d),cell in sorted(cells.items()):
        rows.append(dict(s_m=s,d_m=d*.5,**{k:dict(points=sum(cell[k].values()),frames=len(cell[k+'_frames']),
            intensity_histogram=dict(sorted(cell[k].items()))) for k in ('ground','above')}))
    result=dict(source=str(Path(args.bag).resolve()),scope='diagnostic; height proxies, not vehicle/road truth or free-space permission',
        cell_size_m=[1.,.5],counts=dict(counts),cells=rows)
    Path(args.output).write_text(json.dumps(result,indent=2))
    print(json.dumps(dict(counts=counts,cells=len(rows),mixed_height_cells=sum(bool(r['ground']['points'] and r['above']['points']) for r in rows)),indent=2))


if __name__=='__main__':main()
