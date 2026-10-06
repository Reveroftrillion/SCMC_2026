#!/usr/bin/env python3
"""Image-only stop-line candidates and bounded temporal confirmation; no ROS/GPS."""
from collections import deque
from dataclasses import dataclass, field
import math

import cv2
import numpy as np


@dataclass
class Detection:
    state: str = 'SEARCHING'
    candidate: bool = False
    detected: bool = False
    confidence: float = 0.0  # Geometric white-band coverage, not a trained probability.
    image_y_ratio: float = float('nan')
    endpoints: list = field(default_factory=list)  # Normalized original-image coordinates.
    distance_m: float = float('nan')
    distance_valid: bool = False


def quadrilateral(value):
    points = np.asarray(value, dtype=np.float32)
    if (points.shape != (4, 2) or not np.isfinite(points).all() or
            np.any(points < 0) or np.any(points > 1) or
            not cv2.isContourConvex(points) or abs(cv2.contourArea(points)) < .01):
        raise ValueError('ROI must be four convex normalized points, in boundary order')
    return points


class StopLineDetector:
    def __init__(self, roi=((.1, .92), (.35, .45), (.65, .45), (.9, .92)),
                 bev_size=320, white_v_min=165, white_s_max=80,
                 min_coverage=.6, min_band_height=.01, max_band_height=.09, crosswalk_min_bands=3,
                 crosswalk_max_gap=.14, arrow_stem_min_length=.10, calibration_image_points=(),
                 calibration_ground_points=()):
        self.roi = quadrilateral(roi)
        self.bev_size = int(bev_size)
        self.white_v_min, self.white_s_max = int(white_v_min), int(white_s_max)
        self.min_coverage = float(min_coverage)
        self.min_band_height, self.max_band_height = float(min_band_height), float(max_band_height)
        self.crosswalk_min_bands, self.crosswalk_max_gap = int(crosswalk_min_bands), float(crosswalk_max_gap)
        self.arrow_stem_min_length = float(arrow_stem_min_length)
        if (not 64 <= self.bev_size <= 1024 or not 0 <= self.white_s_max <= 255 or
                not 0 <= self.white_v_min <= 255 or not .3 <= self.min_coverage <= 1 or
                not 0 < self.min_band_height < self.max_band_height < .3 or self.crosswalk_min_bands < 3 or
                not 0 < self.crosswalk_max_gap < 1 or not 0 < self.arrow_stem_min_length < .5):
            raise ValueError('Invalid stop-line detector settings')
        self.calibration_image = self.calibration_ground = None
        if len(calibration_image_points) or len(calibration_ground_points):
            self.calibration_image = quadrilateral(calibration_image_points)
            self.calibration_ground = np.asarray(calibration_ground_points, dtype=np.float32)
            if (self.calibration_ground.shape != (4, 2) or
                    not np.isfinite(self.calibration_ground).all() or
                    not cv2.isContourConvex(self.calibration_ground) or
                    abs(cv2.contourArea(self.calibration_ground)) < .1):
                raise ValueError('Ground calibration requires four convex metric points')

    def has_arrow_stem(self, bird_mask, horizontal, band):
        """Reject a transverse arrow branch attached to a long central stroke.

        Inspect paint before horizontal opening removes the stem. Restrict the check
        to the central half of this band so a stop line touching edge lane paint survives.
        """
        start, end, _support = band
        pixels = np.flatnonzero(horizontal[(start+end)//2])
        if pixels.size < 2:
            return False
        left, right = int(pixels[0]), int(pixels[-1])
        width = right-left+1
        margin = int(width*.25)
        central_left, central_right = left+margin, right-margin+1
        length = max(int(math.ceil(self.bev_size*self.arrow_stem_min_length)),
                     3*(end-start+1))
        # Bridge tiny JPEG/paint gaps; zero padding must not invent strokes at ROI edges.
        connected = cv2.morphologyEx(bird_mask, cv2.MORPH_CLOSE, np.ones((3, 1), np.uint8),
                                     borderType=cv2.BORDER_CONSTANT, borderValue=0)
        vertical = cv2.morphologyEx(connected, cv2.MORPH_OPEN, np.ones((length, 1), np.uint8),
                                    borderType=cv2.BORDER_CONSTANT, borderValue=0)
        columns = np.flatnonzero(np.any(vertical[start:end+1, central_left:central_right] != 0, axis=0))
        if not columns.size:
            return False
        runs = np.split(columns, np.flatnonzero(np.diff(columns) > 1)+1)
        return max(len(run) for run in runs) >= max(3, int(math.ceil(width*.02)))

    def detect(self, frame):
        height, width = frame.shape[:2]
        if height < 32 or width < 32:
            raise ValueError('Image is too small')
        scale = np.float32([width - 1, height - 1])
        source = self.roi * scale
        size = self.bev_size
        destination = np.float32([[0, size-1], [0, 0], [size-1, 0], [size-1, size-1]])
        transform = cv2.getPerspectiveTransform(source, destination)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, self.white_v_min), (180, self.white_s_max, 255))
        bird_mask = cv2.warpPerspective(mask, transform, (size, size), flags=cv2.INTER_NEAREST)
        # Long transverse paint survives; narrow longitudinal lane markings do not.
        horizontal = cv2.morphologyEx(bird_mask, cv2.MORPH_OPEN,
                                     np.ones((1, max(3, int(size*.10))), np.uint8))
        horizontal = cv2.morphologyEx(horizontal, cv2.MORPH_CLOSE,
                                     np.ones((1, max(3, int(size*.02))), np.uint8))
        coverage = np.count_nonzero(horizontal, axis=1) / float(size)
        rows = np.flatnonzero(coverage >= self.min_coverage)
        bands, arrows = [], []
        if rows.size:
            for group in np.split(rows, np.flatnonzero(np.diff(rows) > 2) + 1):
                start, end = int(group[0]), int(group[-1])
                # A one-row ROI-edge fragment of an arrow/bonnet is not a full band.
                if size*self.min_band_height <= end-start+1 <= size*self.max_band_height:
                    band = (start, end, float(np.mean(coverage[group])))
                    (arrows if self.has_arrow_stem(bird_mask, horizontal, band) else bands).append(band)

        # Conservative rejection of nearby repeated transverse bands (zebra crossing).
        repeated = set()
        for start in range(len(bands)):
            group = [start]
            for index in range(start+1, len(bands)):
                gap = bands[index][0] - bands[index-1][1]
                if gap > size*self.crosswalk_max_gap:
                    break
                group.append(index)
            if len(group) >= self.crosswalk_min_bands:
                repeated.update(group)
        choices = [band for i, band in enumerate(bands) if i not in repeated]
        result = Detection(state='CROSSWALK_PATTERN' if repeated and not choices else
                           'ARROW_PATTERN' if arrows and not choices else 'SEARCHING')
        inverse = np.linalg.inv(transform)

        def band_endpoints(band):
            start, end, _support = band
            row = (start+end)/2.
            pixels = np.flatnonzero(horizontal[int(row)])
            if not pixels.size:
                return None
            points = np.float32([[[pixels[0], row], [pixels[-1], row]]])
            return cv2.perspectiveTransform(points, inverse)[0]

        if choices:
            # Bottommost isolated band is the nearest candidate in this lane ROI.
            band = max(choices, key=lambda band: band[1])
            support = band[2]
            original = band_endpoints(band)
            if original is not None:
                result = Detection(state='CANDIDATE', candidate=True, confidence=support,
                                   image_y_ratio=float(np.mean(original[:, 1])/height),
                                   endpoints=(original/scale).tolist())
                if self.calibration_image is not None:
                    to_ground = cv2.getPerspectiveTransform(self.calibration_image*scale,
                                                           self.calibration_ground)
                    ground = cv2.perspectiveTransform(original.reshape(1, 2, 2), to_ground)[0]
                    # Ground points are [forward, left] from the front bumper's ground projection.
                    left0, left1 = ground[:, 1]
                    if np.isfinite(ground).all() and left0*left1 <= 0 and abs(left1-left0) > .01:
                        fraction = -left0/(left1-left0)
                        distance = float(ground[0, 0]+fraction*(ground[1, 0]-ground[0, 0]))
                        if 0 <= distance <= 100:
                            result.distance_m, result.distance_valid = distance, True
        annotated = frame.copy()
        cv2.polylines(annotated, [source.astype(np.int32)], True, (0, 180, 255), 2)
        for arrow in arrows:
            original = band_endpoints(arrow)
            if original is not None:
                endpoints = original.astype(int)
                cv2.line(annotated, tuple(endpoints[0]), tuple(endpoints[1]), (0, 0, 255), 3)
                cv2.putText(annotated, 'ARROW', tuple(endpoints[0]), cv2.FONT_HERSHEY_SIMPLEX,
                            .45, (0, 0, 255), 1)
        if result.endpoints:
            endpoints = (np.asarray(result.endpoints)*scale).astype(int)
            cv2.line(annotated, tuple(endpoints[0]), tuple(endpoints[1]), (0, 255, 0), 4)
        return result, annotated, horizontal


class Confirmation:
    def __init__(self, window=5, hits=3, max_y_jump=.08, timeout=.5):
        self.window, self.hits = int(window), int(hits)
        self.max_y_jump, self.timeout = float(max_y_jump), float(timeout)
        if (not 1 <= self.hits <= self.window <= 30 or
                not math.isfinite(self.max_y_jump) or not 0 < self.max_y_jump < 1 or
                not math.isfinite(self.timeout) or self.timeout <= 0):
            raise ValueError('Invalid stop-line confirmation settings')
        self.history = deque(maxlen=self.window)
        self.last_time = self.last_y = None

    def clear(self):
        self.history.clear()
        self.last_time = self.last_y = None

    def update(self, result, now):
        if (self.last_time is not None and
                (now <= self.last_time or now-self.last_time > self.timeout)):
            self.clear()
        if (result.candidate and self.last_y is not None and
                abs(result.image_y_ratio-self.last_y) > self.max_y_jump):
            self.clear()
        self.last_time = now
        self.history.append(bool(result.candidate))
        if result.candidate:
            self.last_y = result.image_y_ratio
        result.detected = bool(result.candidate and sum(self.history) >= self.hits)
        if result.detected:
            result.state = 'DETECTED'
        # Never advertise an unconfirmed/missing candidate as a usable metric measurement.
        if not result.detected:
            result.distance_valid = False
            result.distance_m = float('nan')
        return result
