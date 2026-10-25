"""Conservative local road-plane estimate for ground removal during body tilt.

Only a broad, dominant, near-ground surface is accepted. Missing support falls
back to the original fixed-height filter. The generic window ends at 28 m;
tunnel material support can validate a continuation within the existing ROI.
"""

import numpy as np


def region(points):
    return ((points[:, 0] > 3.) & (points[:, 0] < 28.)
            & (np.abs(points[:, 1]) < 5.))


def estimate_ground_plane(points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    points = points[region(points) & (points[:, 2] > -.8) & (points[:, 2] < 1.2)]
    if len(points) < 150:
        return None
    random_generator = np.random.default_rng(0)
    if len(points) > 1500:
        points = points[random_generator.choice(len(points), 1500, replace=False)]
    design = np.c_[points[:, :2], np.ones(len(points))]
    # 가장 많은 inlier를 지지하는 평면을 선택한 뒤 그 점들로 최소제곱 재추정한다.
    best_inliers = None
    best_inlier_count = 0
    for _ in range(80):
        sample_indices = random_generator.choice(len(points), 3, replace=False)
        try:
            plane = np.linalg.solve(design[sample_indices], points[sample_indices, 2])
        except np.linalg.LinAlgError:
            continue
        if np.linalg.norm(plane[:2]) > .18 or abs(plane[2]) > .25:
            continue
        inliers = np.abs(design @ plane - points[:, 2]) < .12
        if int(inliers.sum()) > best_inlier_count:
            best_inliers, best_inlier_count = inliers, int(inliers.sum())
    if best_inliers is None or best_inlier_count < max(120, .6 * len(points)):
        return None
    support = points[best_inliers]
    if np.ptp(support[:, 0]) < 8. or np.ptp(support[:, 1]) < 3.:
        return None
    plane = np.linalg.lstsq(design[best_inliers], points[best_inliers, 2], rcond=None)[0]
    if np.linalg.norm(plane[:2]) > .18 or abs(plane[2]) > .25:
        return None
    return plane


def estimate_material_ground_plane(points, ground_mask):
    """Tunnel fallback using returns already classified as asphalt by material.

    A different sensor height reference can put the road above the generic
    estimator's intercept guard. Centre only the existing asphalt returns;
    retain its slope, support count, dominance and spatial-extent checks.
    The input height window stays the same as the generic estimator. Without
    enough broad material support, keep its original fixed-height fallback.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    mask = np.asarray(ground_mask, dtype=bool)
    support = points[mask & np.isfinite(points).all(axis=1) & region(points)
                     & (points[:, 2] > -.8) & (points[:, 2] < 1.2)].copy()
    if not len(support):
        return None
    reference_z = np.median(support[:, 2])
    centered = support.copy()
    centered[:, 2] -= reference_z
    plane = estimate_ground_plane(centered)
    if plane is None:
        # A sloped road's median height is not its height at the origin. After
        # braking it can leave a valid plane outside the centered intercept guard.
        # Retry the height reference only; the same robust estimator must still
        # validate slope, dominance, residuals and broad spatial support.
        design = np.c_[support[:, :2], np.ones(len(support))]
        reference_z = np.linalg.lstsq(design, support[:, 2], rcond=None)[0][2]
        centered[:, 2] = support[:, 2] - reference_z
        plane = estimate_ground_plane(centered)
    if plane is not None:
        plane[2] += reference_z
    return plane


def material_height_region(points, plane, ground_mask, max_x, max_y=5., additional_mask=None):
    """Validate tunnel ground correction inside the observed asphalt hull.

    The generic window remains the fallback. Same-scan material support must
    satisfy the existing estimator's count, residual, dominance and spatial
    extent requirements. Additional support must be broad too. This follows
    a curved road beyond the fixed rectangle without extrapolating the plane
    outside observed material or enlarging the obstacle ROI.
    """
    points = np.asarray(points)
    supported = region(points)
    if plane is None:
        return supported
    roi = (np.isfinite(points).all(axis=1) & (points[:, 0] > 3.) & (points[:, 0] < max_x)
           & (np.abs(points[:, 1]) < max_y))
    candidates = points[np.asarray(ground_mask, dtype=bool) & roi
                        & (points[:, 2] > -.8) & (points[:, 2] < 1.2)]
    if len(candidates) < 150:
        return supported
    agrees = np.abs(candidates[:, 2] - candidates[:, :2] @ plane[:2] - plane[2]) < .12
    evidence = candidates[agrees]
    if (len(evidence) < max(120, .6 * len(candidates))
            or np.ptp(evidence[:, 0]) < 8. or np.ptp(evidence[:, 1]) < 3.):
        return supported
    extra_support = evidence[~region(evidence)]
    if len(extra_support) < 3 or np.ptp(extra_support[:, 1]) < 3.:
        return supported
    # Keep asphalt as the plane/support anchor. Low lane-paint observations may
    # support its edge only after the original broad-asphalt checks above.
    # The same residual and height window reject elevated reflective objects.
    if additional_mask is not None:
        extra_material = points[np.asarray(additional_mask, dtype=bool) & roi
                                & (points[:, 2] > -.8) & (points[:, 2] < 1.2)]
        agrees = np.abs(extra_material[:, 2] - extra_material[:, :2] @ plane[:2] - plane[2]) < .12
        evidence = np.vstack((evidence, extra_material[agrees]))
    from scipy.spatial import ConvexHull, QhullError
    try:
        hull = ConvexHull(evidence[:, :2])
    except QhullError:
        return supported
    extra = np.flatnonzero(roi & ~supported)
    supported[extra] = np.all(points[extra, :2] @ hull.equations[:, :2].T
                              + hull.equations[:, 2] <= 1e-9, axis=1)
    return supported


def relative_height(points, plane, supported=None):
    points = np.asarray(points)
    height = points[:, 2].copy()
    if plane is not None:
        if supported is None:
            supported = region(points)
        height[supported] -= points[supported, :2] @ plane[:2] + plane[2]
    return height
