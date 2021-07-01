
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <thread>
#include <chrono>
#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include "ros/ros.h"
#include "communication/lcm_handler.hpp"
#include "system/cheetah_system.hpp"
#include <memory>

#define LCM_MULTICAST_URL "udpm://239.255.76.67:7667?ttl=2"

template class cheetah_inekf_lcm::InEKF_lcm<12>;

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "cheetah_controller");
    ros::NodeHandle n;

    // Create private node handle
    ros::NodeHandle nh("~");

    // Initialize LCM
    lcm::LCM lcm;
    if (!lcm.good())
    {
        ROS_ERROR_STREAM("LCM init failed.");
        return -1;
    }
    cheetah_inekf_lcm::InEKF_lcm<12> lcm_publisher_node;
    lcm.subscribe("microstrain", &cheetah_inekf_lcm::InEKF_lcm<12>::imu_lcm_callback, &lcm_publisher_node);
    lcm.subscribe("leg_control_data", &cheetah_inekf_lcm::InEKF_lcm<12>::joint_state_lcm_callback, &lcm_publisher_node);
    lcm.subscribe("ground_truth", &cheetah_inekf_lcm::InEKF_lcm<12>::contact_lcm_callback, &lcm_publisher_node);

    // Set noise parameters
    inekf::NoiseParams params;

    //TODO: Initialize CheetahSystem
    // CheetahSystem *system = new CheetahSystem(n);
    // system->setEstimator(std::make_shared<BodyEstimator>());

    // //TODO: Listen/Respond Loop
    // ROS_INFO("Connecting to Cassie");
    // bool received_data = false;
    // while (ros::ok())
    // {
    //     if (received_data)
    //     {
    //     }
    // }

    return 0;
}