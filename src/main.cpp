#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "planner.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "a_star");

    HybridAStar::Planner hy;
    ros::Rate rate(1);
    while (ros::ok()) {
        hy.setMap();
        ros::spinOnce();
        rate.sleep();
    }
    ros::spin();
    return 0;
}
