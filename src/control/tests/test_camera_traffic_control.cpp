#include <gtest/gtest.h>
#include "control/camera_traffic_control.h"

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
namespace {
camera_traffic::Input input(double stamp = 1.0) {
    camera_traffic::Input in;
    in.mission_fresh = in.mission_active = in.vehicle_fresh = in.line_ready = true;
    in.mission_id = 1; in.track_id = in.signal_track_id = 7;
    in.mission_stamp = in.line_stamp = in.signal_stamp = stamp;
    return in;
}
void line(camera_traffic::Input& in, double y, double stamp) {
    in.line_detected = true; in.image_y = y; in.line_stamp = stamp;
}
void green(camera_traffic::Input& in, const std::string& label, double stamp) {
    in.signal_fresh = in.signal_valid = in.signal_observed = true;
    in.label = in.raw_label = label; in.signal_stamp = stamp;
}
void confirm(camera_traffic::Controller& c, camera_traffic::Input& in, const std::string& label = "4green") {
    for (int i = 0; i < 3; ++i) {
        green(in, label, 1.1 + .05*i);
        c.update(in, 1.1 + .05*i);
    }
}
}

TEST(CameraTraffic, IdleDoesNotRestrictGlobalRoute) {
    camera_traffic::Controller c;
    auto in = input(); in.mission_active = false;
    const auto d = c.update(in, 1.0);
    EXPECT_FALSE(d.active); EXPECT_FALSE(d.stop); EXPECT_TRUE(std::isinf(d.speed_limit));
}
TEST(CameraTraffic, MissionStartsWithoutGpsAndSearchIsSlow) {
    camera_traffic::Controller c;
    auto in = input();
    auto d = c.update(in, 1);
    EXPECT_TRUE(d.active); EXPECT_EQ(d.state, "SEARCH_LINE"); EXPECT_DOUBLE_EQ(d.speed_limit, 10);
}
TEST(CameraTraffic, FirstActualBoxCapsSpeedBeforeSizeAndColourConfirmation) {
    camera_traffic::Controller c;
    auto in = input(); in.mission_active = false; in.signal_valid = false;
    in.target_observed = true;
    const auto d = c.update(in, 1);
    EXPECT_FALSE(d.active); EXPECT_FALSE(d.stop);
    EXPECT_EQ(d.state, "PRE_APPROACH"); EXPECT_DOUBLE_EQ(d.speed_limit, 40);
}
TEST(CameraTraffic, BriefDetectionLossKeepsEarlyCapWithoutExtendingFromDuplicates) {
    camera_traffic::Controller c;
    auto in = input(); in.mission_active = false; in.target_observed = true;
    c.update(in, 1);
    EXPECT_EQ(c.update(in, 2.9).state, "PRE_APPROACH");
    EXPECT_EQ(c.update(in, 3.01).state, "IDLE");
    in.signal_stamp = 3.1;
    EXPECT_EQ(c.update(in, 3.1).state, "PRE_APPROACH");
}
TEST(CameraTraffic, HeldOrUnobservedBoxCannotStartEarlyCap) {
    camera_traffic::Controller c;
    auto in = input(); in.mission_active = false; in.target_observed = false;
    EXPECT_EQ(c.update(in, 1).state, "IDLE");
    in.target_observed = true; c.update(in, 1.1);
    in.target_observed = false;
    EXPECT_EQ(c.update(in, 1.5).state, "PRE_APPROACH");
    EXPECT_EQ(c.update(in, 3.2).state, "IDLE");
}
TEST(CameraTraffic, CooldownClearsEarlyCapAndDoesNotRetriggerOnOldLight) {
    camera_traffic::Controller c;
    auto in = input(); in.mission_active = false; in.target_observed = true;
    c.update(in, 1);
    in.mission_cooldown = true; in.signal_stamp = 1.1;
    EXPECT_EQ(c.update(in, 1.1).state, "IDLE");
    in.signal_stamp = 1.2;
    EXPECT_EQ(c.update(in, 1.2).state, "IDLE");
}
TEST(CameraTraffic, EarlyCapTransitionsToStricterMissionSearch) {
    camera_traffic::Controller c;
    auto in = input(); in.mission_active = false; in.target_observed = true;
    EXPECT_DOUBLE_EQ(c.update(in, 1).speed_limit, 40);
    in.mission_active = true;
    const auto d = c.update(in, 1.1);
    EXPECT_TRUE(d.active); EXPECT_EQ(d.state, "SEARCH_LINE"); EXPECT_DOUBLE_EQ(d.speed_limit, 10);
}
TEST(CameraTraffic, SearchCannotContinueIndefinitelyWithoutALine) {
    camera_traffic::Controller c;
    auto in = input(); in.velocity = 36;
    c.update(in, 1);
    EXPECT_EQ(c.update(in, 3.1).state, "NO_STOP_LINE");
    line(in, .5, 3.2);
    EXPECT_FALSE(c.update(in, 3.2).stop);
}
TEST(CameraTraffic, ImageApproachSlowsAndWaitsAtConfiguredPosition) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .56, 1);
    const double far = c.update(in, 1).speed_limit;
    line(in, .70, 1.1);
    EXPECT_LT(c.update(in, 1.1).speed_limit, far);
    line(in, .75, 1.2);
    EXPECT_EQ(c.update(in, 1.2).state, "WAIT_SIGNAL");
    EXPECT_TRUE(c.update(in, 1.2).stop);
}
TEST(CameraTraffic, RedYellowUnknownAndMixedRedStayStopped) {
    for (const auto& label : {"4red", "4yellow", "UNKNOWN", "4redleft", "4redyellow", "3red", "bad"}) {
        camera_traffic::Controller c;
        auto in = input(); line(in, .76, 1);
        green(in, label, 1);
        EXPECT_TRUE(c.update(in, 1).stop) << label;
    }
}
TEST(CameraTraffic, BothAllowedGreensReleaseAfterThreeDistinctObservations) {
    for (const auto& label : {"4green", "4greenleft"}) {
        camera_traffic::Controller c;
        auto in = input(); line(in, .76, 1);
        c.update(in, 1);
        green(in, label, 1.1); EXPECT_TRUE(c.update(in, 1.1).stop);
        green(in, label, 1.2); EXPECT_TRUE(c.update(in, 1.2).stop);
        green(in, label, 1.3);
        const auto d = c.update(in, 1.3);
        EXPECT_FALSE(d.stop); EXPECT_EQ(d.state, "CROSSING"); EXPECT_DOUBLE_EQ(d.speed_limit, 40);
    }
}
TEST(CameraTraffic, HeldAndDuplicateGreenDoNotConfirmDeparture) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, 1); c.update(in, 1);
    green(in, "4green", 1.1);
    for (int i = 0; i < 10; ++i) EXPECT_TRUE(c.update(in, 1.1 + i*.01).stop);
    in.signal_observed = false;
    for (int i = 0; i < 3; ++i) {
        in.signal_stamp += .01;
        EXPECT_TRUE(c.update(in, 1.2 + i*.01).stop);
    }
}
TEST(CameraTraffic, BlockedPathCannotCommitAndRedStillAppliesWhenPathReopens) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, 1); c.update(in, 1);
    in.motion_allowed = false;
    confirm(c, in);
    EXPECT_EQ(c.update(in, 1.25).state, "WAIT_PATH");
    in.motion_allowed = true;
    green(in, "4red", 1.3);
    EXPECT_EQ(c.update(in, 1.3).state, "WAIT_SIGNAL");
    EXPECT_TRUE(c.update(in, 1.3).stop);
}
TEST(CameraTraffic, WrongTrackAndPreMissionGreenCannotRelease) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, 1); c.update(in, 1);
    in.signal_track_id = 8; confirm(c, in);
    EXPECT_TRUE(c.update(in, 1.25).stop);
    in.signal_track_id = 7; green(in, "4green", .9);
    EXPECT_TRUE(c.update(in, 1.25).stop);
}
TEST(CameraTraffic, StaleGreenCancelsConfirmationAndCannotReleaseWait) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, 1); c.update(in, 1);
    green(in, "4green", 1.1); c.update(in, 1.1);
    green(in, "4green", 1.2); c.update(in, 1.2);
    in.signal_fresh = false;
    EXPECT_TRUE(c.update(in, 1.3).stop);
    green(in, "4green", 1.4);
    EXPECT_TRUE(c.update(in, 1.4).stop);
}
TEST(CameraTraffic, OccludedLineDoesNotReleaseWaitingVehicle) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, 1); c.update(in, 1);
    in.line_detected = false;
    EXPECT_TRUE(c.update(in, 3).stop);
    for (int i = 0; i < 3; ++i) { green(in, "4green", 3.1+i*.1); c.update(in, 3.1+i*.1); }
    EXPECT_EQ(c.update(in, 3.35).state, "CROSSING");
}
TEST(CameraTraffic, CameraOrVehicleFailureCannotAuthorizeDeparture) {
    for (int kind = 0; kind < 3; ++kind) {
        camera_traffic::Controller c;
        auto in = input(); line(in, .76, 1); c.update(in, 1);
        confirm(c, in); // Crossing is already authorized; test a fresh controller below.
        c.reset(); in.signal_observed = false; c.update(in, 1.25);
        if (kind == 0) in.line_ready = false;
        if (kind == 1) in.mission_fresh = false;
        if (kind == 2) in.vehicle_fresh = false;
        green(in, "4green", 1.3);
        EXPECT_TRUE(c.update(in, 1.3).stop);
    }
}
TEST(CameraTraffic, LostApproachLineStopsAfterBoundedHold) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .57, 1); c.update(in, 1);
    in.line_detected = false;
    EXPECT_FALSE(c.update(in, 1.3).stop);
    EXPECT_EQ(c.update(in, 1.36).state, "STOP_LINE_LOST");
    line(in, .62, 1.4); EXPECT_FALSE(c.update(in, 1.4).stop);
}
TEST(CameraTraffic, MetricDistanceOverridesImageAndReservesStoppingMargin) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .79, 1);
    in.distance_valid = true; in.distance_m = 20;
    auto d = c.update(in, 1);
    EXPECT_EQ(d.state, "APPROACH_METRIC"); EXPECT_LE(d.speed_limit, 40);
    in.distance_m = 5; in.line_stamp = 1.1;
    EXPECT_LT(c.update(in, 1.1).speed_limit, d.speed_limit);
    in.distance_m = 2; in.line_stamp = 1.2;
    EXPECT_TRUE(c.update(in, 1.2).stop);
}
TEST(CameraTraffic, MetricHoldAccountsForTravelledDistance) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .56, 1);
    in.distance_valid = true; in.distance_m = 3; in.velocity = 36;
    c.update(in, 1);
    in.line_detected = false;
    EXPECT_TRUE(c.update(in, 1.2).stop);
}
TEST(CameraTraffic, StrictMetricModeRejectsUncalibratedLine) {
    camera_traffic::Controller c; camera_traffic::Settings s;
    s.allow_image_stop = false; c.configure(s);
    auto in = input(); line(in, .76, 1);
    EXPECT_EQ(c.update(in, 1).state, "METRIC_CALIBRATION_REQUIRED");
}
TEST(CameraTraffic, OldLineCannotStopOrCommitNewMission) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, .9);
    EXPECT_EQ(c.update(in, 1).state, "SEARCH_LINE");
    confirm(c, in);
    EXPECT_EQ(c.update(in, 1.3).state, "SEARCH_LINE");
}
TEST(CameraTraffic, EarlyCameraExitCannotReleaseRedStop) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, 1); c.update(in, 1);
    in.mission_active = false;
    const auto d = c.update(in, 1.1);
    EXPECT_TRUE(d.active); EXPECT_TRUE(d.stop);
    EXPECT_EQ(d.state, "MISSION_CHANGED_BEFORE_CROSSING");
}
TEST(CameraTraffic, CommittedCrossingIgnoresLaterSignalLossAndEndsOnCameraExit) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, 1); c.update(in, 1); confirm(c, in);
    in.signal_fresh = false; in.line_ready = false; in.mission_fresh = false;
    const auto d = c.update(in, 1.3);
    EXPECT_FALSE(d.stop); EXPECT_DOUBLE_EQ(d.speed_limit, 40);
    in.mission_fresh = true; in.mission_active = false;
    EXPECT_FALSE(c.update(in, 1.4).active);
}
TEST(CameraTraffic, NextMissionHasNoPreviousLineOrGreenVotes) {
    camera_traffic::Controller c;
    auto in = input(); line(in, .76, 1); c.update(in, 1); confirm(c, in);
    in.mission_id = 2; in.track_id = in.signal_track_id = 8; in.mission_stamp = 2;
    EXPECT_EQ(c.update(in, 2).state, "SEARCH_LINE");
    line(in, .76, 2.1); green(in, "4green", 2.1);
    EXPECT_TRUE(c.update(in, 2.1).stop);
}
TEST(CameraTraffic, InvalidSettingsAndNonFiniteInputsFailClosed) {
    camera_traffic::Controller c; camera_traffic::Settings s;
    s.max_velocity = 41; EXPECT_THROW(c.configure(s), std::invalid_argument);
    s = {}; s.go_labels = {"4redleft"}; EXPECT_THROW(c.configure(s), std::invalid_argument);
    s = {}; s.image_stop_y = s.image_slow_y; EXPECT_THROW(c.configure(s), std::invalid_argument);
    auto in = input(); in.velocity = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(c.update(in, 1).stop);
}
