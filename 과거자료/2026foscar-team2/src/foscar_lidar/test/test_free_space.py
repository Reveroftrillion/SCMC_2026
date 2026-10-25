import numpy as np
from foscar_lidar.free_space import ray_box_evidence


def test_returns_behind_volume_are_positive_free_space_evidence():
    result = ray_box_evidence([0, 0, 1], [[20, 0, 1], [20, .3, 1.2]], [9, -1, .3], [11, 1, 1.8])
    assert result['through'] == 2
    assert result['occupied'] == 0


def test_occlusion_missing_returns_and_ground_are_not_clearance():
    for points in ([], [[5, 0, 1]], [[20, 5, 1]], [[20, 0, -3]], [[np.nan, 0, 1]]):
        result = ray_box_evidence([0, 0, 1], points, [9, -1, .3], [11, 1, 1.8])
        assert result['through'] == 0


def test_obstacle_returns_and_other_free_rays_are_reported_together():
    result = ray_box_evidence([0, 0, 1], [[10, 0, 1], [20, .3, 1.2]], [9, -1, .3], [11, 1, 1.8])
    assert result['occupied'] == 1
    assert result['through'] == 1


def test_return_on_far_surface_is_not_a_ray_through_the_volume():
    result = ray_box_evidence([0, 0, 1], [[11, 0, 1]], [9, -1, .3], [11, 1, 1.8])
    assert result['through'] == 0
    assert result['occupied'] == 1


def test_body_clearance_requires_spatial_coverage_and_no_occupied_returns():
    from foscar_lidar.free_space import body_clearance
    points=np.array([[20.,y,z] for y in np.linspace(-3,3,61) for z in np.linspace(-.5,2.5,31)])
    assert body_clearance([0,0,1],points,[10,0],2.,2.,0.)
    assert not body_clearance([0,0,1],np.vstack([points,[10.,0.,1.]]),[10,0],2.,2.,0.)
    assert not body_clearance([0,0,1],[[20.,0.,1.]], [10,0],2.,2.,0.)
    assert not body_clearance([0,0,1],points*.2,[10,0],2.,2.,0.)


def test_clearance_requires_two_fresh_sweeps_and_rejects_stale_query():
    from foscar_lidar.clearance import ClearanceVerifier
    verifier=ClearanceVerifier()
    points=np.array([[20.,y,z] for y in np.linspace(-4,4,81) for z in np.linspace(-1,3,41)])
    q=dict(frame='map',stamp=1.,objects=[dict(id='ghost',last_seen=.5,x=10.,y=0.,yaw=0.,length=2.,width=2.,vx=0.,vy=0.)])
    verifier.accept(q)
    assert not verifier.check(1.05,points,[0,0,1],(0,0,0))
    assert verifier.check(1.15,points,[0,0,1],(0,0,0))==[dict(id='ghost',last_seen=.5)]
    assert not verifier.check(1.5,points,[0,0,1],(0,0,0))


def test_lane_patch_requires_all_cells_and_rejects_occupied_or_missing_returns():
    from foscar_lidar.free_space import lane_patch_clearance
    points=np.array([[20.,y,z] for y in np.linspace(-4,4,81) for z in np.linspace(-1,2,31)])
    assert lane_patch_clearance([0,0,2.],points,[10,0],4.,3.5,0.)
    for bad in ([],points*.2,np.vstack([points,[10.,0.,.8]]),points[points[:,1]>0]):
        assert not lane_patch_clearance([0,0,2.],bad,[10,0],4.,3.5,0.)


def test_ring_clearance_uses_low_body_rays_but_requires_repeated_unoccupied_sweeps():
    from foscar_lidar.clearance import ClearanceVerifier
    points=np.array([[20.,y,-.5] for y in np.linspace(-4,4,161)])
    q=dict(frame='map',stamp=1.,objects=[dict(id='old',purpose='ring_vehicle',last_seen=.5,
        x=10.,y=0.,yaw=0.,length=2.,width=2.,vx=0.,vy=0.)])
    verifier=ClearanceVerifier();verifier.accept(q)
    assert not verifier.check(1.05,points,[0,0,2.],(0,0,0))
    assert verifier.check(1.15,points,[0,0,2.],(0,0,0))==[dict(id='old',last_seen=.5)]
    assert not verifier.check(1.25,np.vstack([points,[10.,0.,1.8]]),[0,0,2.],(0,0,0))
    q['objects'][0].pop('purpose');verifier=ClearanceVerifier();verifier.accept(q)
    assert not verifier.check(1.05,points,[0,0,2.],(0,0,0))
    assert not verifier.check(1.15,points,[0,0,2.],(0,0,0))


def test_side_endpoint_proof_is_not_object_deletion_and_requires_two_clear_sweeps():
    from foscar_lidar.clearance import ClearanceVerifier
    points=np.array([[20.,y,z] for y in np.linspace(-4,4,81) for z in np.linspace(-1,2,31)])
    q=dict(frame='map',stamp=1.,objects=[dict(id='boundary:car:rear',track_id='car',side='rear',
        purpose='ring_boundary',last_seen=1.,x=10.,y=0.,yaw=0.,length=.8,width=2.3,
        vx=0.,vy=0.,edge_s=10.,speed=0.)])
    verifier=ClearanceVerifier();verifier.accept(q)
    assert not verifier.check(1.05,points,[0,0,2.],(0,0,0))
    assert not verifier.boundaries
    assert not verifier.check(1.15,points,[0,0,2.],(0,0,0))
    assert verifier.boundaries[0]['id']=='car'
    assert verifier.boundaries[0]['side']=='rear'
    assert not verifier.check(1.25,np.vstack([points,[10.,0.,.8]]),[0,0,2.],(0,0,0))
    assert not verifier.boundaries
    for t in (1.27,1.29):
        verifier.check(t,points*.2,[0,0,2.],(0,0,0))
        assert not verifier.boundaries


def test_boundary_patch_uses_current_outline_not_old_centroid_extrapolation():
    from foscar_lidar.clearance import ClearanceVerifier
    q=dict(track_id='car',side='rear',edge_s=0.,circle_center=[0.,0.],circle_radius=20.,circle_direction=1,
           x=99.,y=99.,stamp=1.,width=2.3)
    arc=np.array([2.,4.,6.]);xy=np.c_[21*np.cos(arc/20),21*np.sin(arc/20)]
    objects=[dict(id='car',xy=xy.tolist(),center=xy[1].tolist())]
    now=ClearanceVerifier.current_endpoint(q,objects,1.2)
    assert abs(now['edge_s']-2.)<1e-8
    assert now['stamp']==1.2 and abs(now['x'])<22.
    assert ClearanceVerifier.current_endpoint(q,[],1.2) is None
