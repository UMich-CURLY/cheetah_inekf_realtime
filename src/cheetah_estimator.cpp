
// STL
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
#include <memory>
#include <iostream>

#include "ros/ros.h"
#include "utils/cheetah_data_t.hpp"
#include "communication/lcm_handler.hpp"
#include "system/cheetah_system.hpp"

// Boost
#include <boost/algorithm/string.hpp>
// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#define LCM_MULTICAST_URL "udpm://239.255.76.67:7667?ttl=2"


int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "cheetah_estimator");
    ros::NodeHandle nh;

    // Initialize LCM
    lcm::LCM lcm(LCM_MULTICAST_URL);
    if (!lcm.good())
    {
        ROS_ERROR_STREAM("LCM init failed.");
        return -1;
    }

    // Threading
    boost::mutex cdata_mtx;

    cheetah_lcm_data_t cheetah_input_data;
    cheetah_inekf_lcm::lcm_handler lcm_subscriber_node(&lcm, &nh, &cheetah_input_data, &cdata_mtx);

    // Set noise parameters
    inekf::NoiseParams params;

    //TODO: Initialize CheetahSystem
    CheetahSystem *system = new CheetahSystem(&lcm, &nh, &cdata_mtx, &cheetah_input_data);
    // system->setEstimator(std::make_shared<BodyEstimator>());

    // //TODO: Listen/Respond Loop
    bool received_data = true;
    while (lcm.handle() == 0 && ros::ok())
    {
        system->step();
        /// TODO: publish to ros

        ros::spinOnce();
    }

    delete system;
    return 0;
}