#!/usr/bin/env python3
"""Actual observations, missed detections and mission boundaries without ROS/YOLO."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from traffic_camera_core import (CameraMission, Observation, Signal, SignalTracker, TRAFFIC_CLASSES)


def light(label='4red', width=.04, x=.5, y=.25, confidence=.8):
    return Observation(label, confidence, (x-width/2, y-.01, x+width/2, y+.01))


def signal(width=.05, track=1, observed=True, y=.25, x=.5):
    box = (x-width/2, y-.01, x+width/2, y+.01)
    return Signal(label='4red', track_id=track, observed=observed, held=not observed,
                  bbox=box, raw_bbox=box if observed else None)


class TrackerTests(unittest.TestCase):
    def confirmed(self, label='4greenleft'):
        tracker = SignalTracker()
        for now in (1., 1.05, 1.1): result = tracker.update([light(label)], now)
        self.assertEqual(result.label, label)
        return tracker

    def test_all_trained_classes_need_real_observations(self):
        for label in TRAFFIC_CLASSES:
            with self.subTest(label=label):
                tracker = SignalTracker()
                self.assertFalse(tracker.update([light(label)], 1.).valid)
                self.assertFalse(tracker.update([light(label)], 1.05).valid)
                self.assertEqual(tracker.update([light(label)], 1.1).label, label)

    def test_brief_miss_holds_colour_and_box_with_explicit_age(self):
        tracker = self.confirmed()
        result = tracker.update([], 1.2)
        self.assertEqual(result.label, '4greenleft')
        self.assertTrue(result.held)
        self.assertFalse(result.observed)
        self.assertEqual(result.raw_label, 'UNKNOWN')
        self.assertAlmostEqual(result.detection_age, .1)

    def test_long_miss_expires_colour_and_size_and_requires_reconfirmation(self):
        tracker = self.confirmed()
        result = tracker.update([], 1.5)
        self.assertFalse(result.valid)
        self.assertIsNone(result.bbox)
        self.assertFalse(tracker.update([light('4greenleft')], 1.55).valid)

    def test_red_yellow_and_red_combinations_cancel_go_immediately(self):
        for label in TRAFFIC_CLASSES-{'4green', '4greenleft'}:
            with self.subTest(label=label):
                tracker = self.confirmed()
                self.assertEqual(tracker.update([light(label)], 1.15).label, label)
                for now in (1.2, 1.25):
                    self.assertNotEqual(tracker.update([light('4greenleft')], now).label, '4greenleft')
                self.assertEqual(tracker.update([light('4greenleft')], 1.3).label, '4greenleft')

    def test_camera_failure_invalidates_held_go_and_box(self):
        tracker = self.confirmed()
        result = tracker.fault(1.15)
        self.assertFalse(result.valid)
        self.assertIsNone(result.bbox)
        self.assertEqual(result.state, 'STALE_IMAGE')
        self.assertFalse(tracker.update([light('4greenleft')], 1.2).valid)

    def test_higher_confidence_adjacent_light_does_not_replace_tracked_light(self):
        tracker = self.confirmed('4red')
        previous_id = tracker.track_id
        result = tracker.update([light('4red', confidence=.6), light('4greenleft', x=.8, confidence=.99)], 1.15)
        self.assertEqual(result.track_id, previous_id)
        self.assertEqual(result.raw_label, '4red')

    def test_unlocked_new_target_gets_new_id_and_no_previous_colour(self):
        tracker = self.confirmed()
        old_id = tracker.track_id
        result = tracker.update([light('4red', x=.8)], 2.2)
        self.assertGreater(result.track_id, old_id)
        self.assertFalse(result.valid)

    def test_active_mission_never_switches_to_another_light_on_loss(self):
        tracker = self.confirmed()
        old_id = tracker.track_id
        result = tracker.update([light('4red', x=.8)], 3., locked=True)
        self.assertEqual(result.track_id, old_id)
        self.assertFalse(result.observed)
        self.assertFalse(result.valid)

    def test_box_size_is_smoothed_without_predicting_growth_on_missing_frames(self):
        tracker = self.confirmed()
        result = tracker.update([light(width=.06)], 1.15)
        self.assertAlmostEqual(result.width, .047)
        held = tracker.update([], 1.2)
        self.assertEqual(held.bbox, result.bbox)

    def test_invalid_boxes_and_time_reversal_do_not_revive_go(self):
        tracker = self.confirmed()
        result = tracker.update([Observation('4greenleft', .9, (-.1, .1, .2, .2))], 1.6)
        self.assertFalse(result.valid)
        self.assertFalse(tracker.update([light('4greenleft')], .9).valid)


class MissionTests(unittest.TestCase):
    def active(self, **options):
        mission = CameraMission(**options)
        for now in (1., 1.15, 1.3): result = mission.update(signal(), now)
        self.assertTrue(result.active)
        return mission

    def armed(self):
        mission = self.active()
        for now in (1.4, 1.5, 1.6): result = mission.update(signal(width=.08, y=.02), now)
        self.assertTrue(result.exit_armed)
        return mission

    def test_small_distant_signal_never_starts_mission(self):
        mission = CameraMission()
        for now in (1., 1.2, 1.4, 1.6):
            self.assertFalse(mission.update(signal(width=.03), now).active)

    def test_size_needs_multiple_observations_and_elapsed_time(self):
        mission = CameraMission()
        for now in (1., 1.05, 1.1): self.assertFalse(mission.update(signal(), now).active)
        self.assertTrue(mission.update(signal(), 1.3).active)

    def test_held_boxes_cannot_start_mission(self):
        mission = CameraMission()
        mission.update(signal(), 1.)
        for now in (1.1, 1.2, 1.3):
            self.assertFalse(mission.update(signal(observed=False), now).active)
        self.assertEqual(mission.update(Signal(), 1.4).phase, 'IDLE')

    def test_different_target_cannot_reuse_entry_confirmation(self):
        mission = CameraMission()
        for now in (1., 1.2): mission.update(signal(track=1), now)
        self.assertFalse(mission.update(signal(track=2), 1.3).active)

    def test_active_is_latched_when_box_size_drops(self):
        mission = self.active()
        self.assertTrue(mission.update(signal(width=.02), 1.5).active)

    def test_simple_target_loss_never_finishes_mission(self):
        mission = self.active()
        result = mission.update(Signal(track_id=1), 5.)
        self.assertTrue(result.active)
        self.assertEqual(result.phase, 'ACTIVE_LOST')

    def test_edge_growth_then_camera_fresh_target_loss_finishes_mission(self):
        mission = self.armed()
        self.assertTrue(mission.update(Signal(track_id=1), 2.5).active)
        result = mission.update(Signal(track_id=1), 2.7)
        self.assertFalse(result.active)
        self.assertEqual(result.phase, 'COOLDOWN')
        self.assertEqual(result.reason, 'CAMERA_EDGE_EXIT')

    def test_single_edge_frame_and_small_edge_light_do_not_arm_exit(self):
        for width in (.05, .08):
            mission = self.active()
            result = mission.update(signal(width=width, y=.02), 1.4)
            self.assertFalse(result.exit_armed)
            self.assertTrue(mission.update(Signal(track_id=1), 4.).active)

    def test_camera_dropout_does_not_count_as_leaving_the_intersection(self):
        mission = self.armed()
        self.assertTrue(mission.fault(1.7).active)
        self.assertFalse(mission.exit_armed)
        self.assertTrue(mission.update(Signal(track_id=1), 5.).active)

    def test_signal_returning_from_edge_cancels_exit_evidence(self):
        mission = self.armed()
        self.assertFalse(mission.update(signal(width=.08), 1.7).exit_armed)
        self.assertTrue(mission.update(Signal(track_id=1), 4.).active)

    def test_held_box_cannot_grow_size_or_arm_exit(self):
        mission = self.active()
        result = mission.update(signal(width=.20, y=.02, observed=False), 1.4)
        self.assertAlmostEqual(result.peak_width, .05)
        self.assertFalse(result.exit_armed)

    def test_external_mode_waits_for_pass_confirmation(self):
        mission = self.active(exit_mode='external')
        for now in (1.4, 1.5, 1.6): mission.update(signal(width=.08, y=.02), now)
        self.assertTrue(mission.update(Signal(track_id=1), 5.).active)
        self.assertFalse(mission.complete(5.1).active)

    def test_cooldown_does_not_retrigger_same_large_signal(self):
        mission = self.active()
        mission.complete(1.4)
        self.assertEqual(mission.update(signal(), 5.).phase, 'COOLDOWN')
        mission.update(Signal(track_id=1), 5.1)
        result = mission.update(Signal(track_id=1), 6.2)
        self.assertEqual(result.phase, 'IDLE')
        self.assertTrue(result.reset_track)
        for now in (6.3, 6.45, 6.6): result = mission.update(signal(track=2), now)
        self.assertTrue(result.active)
        self.assertEqual(result.mission_id, 2)

    def test_disabled_mission_stays_disabled(self):
        mission = CameraMission(enabled=False)
        self.assertEqual(mission.update(signal(), 1.).phase, 'DISABLED')


if __name__ == '__main__': unittest.main()
