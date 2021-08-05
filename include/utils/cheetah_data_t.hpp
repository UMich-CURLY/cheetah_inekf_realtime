#pragma once
// Utility libraries
#include "utils/imu.hpp"
#include "sensor_msgs/JointState.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"

#include <memory>

#include <boost/circular_buffer.hpp>

struct cheetah_lcm_data_t {
    cheetah_lcm_data_t() {}
    cheetah_lcm_data_t(uint32_t qsz): qsz_(qsz) {
        imu_q.resize(qsz_);
        kin_q.resize(qsz_);
        contact_q.resize(qsz_);
    }

    uint32_t qsz_;
    boost::circular_buffer<cheetah_inekf_lcm::ImuMeasurement<float>* > imu_q;
    boost::circular_buffer<inekf_msgs::KinematicsArray* > kin_q;
    boost::circular_buffer<inekf_msgs::ContactArray* > contact_q;
};

struct cheetah_lcm_packet_t {
    ros::Time t;
    cheetah_inekf_lcm::ImuMeasurement<float> imu_q;
    inekf_msgs::KinematicsArray kin_q;
    inekf_msgs::ContactArray contact_q;
};