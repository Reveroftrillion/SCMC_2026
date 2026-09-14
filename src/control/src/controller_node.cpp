#include <ros/ros.h>

#include "control/controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");

    Controller controller;

    ros::Rate rate(50);
    while(ros::ok()){
        ros::spinOnce();

        controller.controlPublish();

        rate.sleep();
    }
}
