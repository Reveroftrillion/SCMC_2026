#include <gtest/gtest.h>
#include "control/traffic_mission.h"

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

namespace {
traffic::MissionController controller() {
    traffic::MissionController c;
    c.configure({{"TL1", {0,0}, {20,0}, {30,0}}, {"TL2", {50,0}, {70,0}, {80,0}}}, {});
    return c;
}
void green(traffic::MissionController& c, double start) {
    for (int i=0; i<3; ++i) c.observe("4greenleft", start+i*0.1);
}
}

TEST(TrafficMission, ActivationRequiresDetectCorridor) {
    auto c = controller();
    EXPECT_FALSE(c.update({-4,0}, 1).active);
    EXPECT_FALSE(c.update({0,13}, 1).active);
    auto d=c.update({-3,0}, 1);
    EXPECT_TRUE(d.active);
    EXPECT_FALSE(d.stop);
    EXPECT_DOUBLE_EQ(d.speed_limit, 20);
}

TEST(TrafficMission, UnknownAndStopSignalsStopAtTolerance) {
    for (const auto& label : {"UNKNOWN", "4red", "4yellow", "4green", "", "garbage"}) {
        auto c=controller();
        c.update({0,0}, 1);
        c.observe(label, 1.1);
        EXPECT_TRUE(c.update({16,0}, 1.2).stop) << label;
        EXPECT_TRUE(c.update({23,0}, 1.3).stop) << label;
    }
}

TEST(TrafficMission, SpeedDecreasesBeforeStopPoint) {
    auto c=controller();
    EXPECT_DOUBLE_EQ(c.update({0,0}, 1).speed_limit, 20);
    EXPECT_LT(c.update({15,0}, 1).speed_limit, 20);
    EXPECT_TRUE(c.update({16,0}, 1).stop);
}

TEST(TrafficMission, GreenNeedsThreeRecentMessagesAndRedActsImmediately) {
    auto c=controller();
    c.update({16,0}, 1);
    c.observe("4greenleft", 1.1);
    c.observe("4greenleft", 1.2);
    EXPECT_TRUE(c.update({16,0}, 1.2).stop);
    c.observe("4greenleft", 1.3);
    EXPECT_FALSE(c.update({16,0}, 1.3).stop);
    EXPECT_EQ(c.activeIndex(), 0u);
    c.observe("4red", 1.4);
    EXPECT_TRUE(c.update({16,0}, 1.4).stop);
}

TEST(TrafficMission, StaleGreenAndDetectionLossRequireReconfirmation) {
    auto c=controller();
    c.update({16,0}, 1);
    green(c, 1.1);
    EXPECT_FALSE(c.update({16,0}, 1.4).stop);
    EXPECT_TRUE(c.update({16,0}, 2.1).stop);
    c.observe("4greenleft", 2.2);
    EXPECT_TRUE(c.update({16,0}, 2.2).stop);
    green(c, 2.3);
    EXPECT_FALSE(c.update({16,0}, 2.6).stop);
    c.observe("UNKNOWN", 2.7);
    EXPECT_TRUE(c.update({16,0}, 2.7).stop);
}

TEST(TrafficMission, OnlyCompletesAfterPassAndClearsSignalForNextMission) {
    auto c=controller();
    c.update({0,0}, 1);
    green(c, 1.1);
    EXPECT_EQ(c.update({20,0}, 1.4).state, "CROSSING");
    c.observe("4red", 1.5);
    EXPECT_FALSE(c.update({25,0}, 1.5).stop);
    c.update({33,0}, 1.5);
    EXPECT_EQ(c.activeIndex(), 0u);
    c.update({34,0}, 1.5);
    EXPECT_EQ(c.activeIndex(), 1u);
    green(c, 2.0); // A green received before entering TL2 must not release TL2.
    c.update({50,0}, 2.3);
    EXPECT_TRUE(c.update({66,0}, 2.3).stop);
    green(c, 2.4);
    EXPECT_FALSE(c.update({66,0}, 2.7).stop);
}

TEST(TrafficMission, PoseLossAndLeavingCorridorStop) {
    auto c=controller();
    c.update({0,0}, 1);
    green(c, 1.1);
    EXPECT_TRUE(c.update({16,0}, 1.4, false).stop);
    EXPECT_TRUE(c.update({16,13}, 1.4).stop);
    EXPECT_TRUE(c.update({std::numeric_limits<double>::quiet_NaN(),0}, 1.4).stop);
}

TEST(TrafficMission, FinishesAllMissions) {
    auto c=controller();
    c.update({34,0}, 1);
    c.update({84,0}, 1);
    EXPECT_EQ(c.activeIndex(), 2u);
    EXPECT_EQ(c.update({85,0}, 1).state, "DONE");
}

TEST(TrafficMission, RejectsInvalidConfiguration) {
    traffic::MissionController c;
    EXPECT_THROW(c.configure({}, {}), std::invalid_argument);
    EXPECT_THROW(c.configure({{"TL1", {0,0}, {20,0}, {0,0}}}, {}), std::invalid_argument);
    auto settings=traffic::Settings{};
    settings.signal_timeout=0;
    EXPECT_THROW(c.configure({{"TL1", {0,0}, {20,0}, {30,0}}}, settings), std::invalid_argument);
}
