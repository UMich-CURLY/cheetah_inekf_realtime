#pragma once
// Utility libraries
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"

#include <boost/circular_buffer.hpp>

struct cheetah_lcm_data_t {
    cheetah_lcm_data_t() {}
    cheetah_lcm_data_t(uint32_t qsz): qsz_(qsz) {
        imu_q.resize(qsz_);
        kin_q.resize(qsz_);
        contact_q.resize(qsz_);
    }

    uint32_t qsz_;
    boost::circular_buffer<sensor_msgs::Imu> imu_q;
    boost::circular_buffer<inekf_msgs::KinematicsArray> kin_q;
    boost::circular_buffer<inekf_msgs::ContactArray> contact_q;
};