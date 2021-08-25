#pragma once
// Utility libraries
#include "utils/imu.hpp"
#include "utils/joint_state.hpp"
#include "utils/contacts.hpp"
#include "utils/measurement.h"
#include "sensor_msgs/JointState.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"

#include <memory>
#include <queue>

#include <boost/circular_buffer.hpp>

struct cheetah_lcm_data_t {
    cheetah_lcm_data_t() {}

    // std::queue<cheetah_inekf_lcm::ImuMeasurement<double>* > imu_q;
    std::queue<std::shared_ptr<cheetah_inekf_lcm::ImuMeasurement<double>>> imu_q;
    std::queue<std::shared_ptr<cheetah_inekf_lcm::JointStateMeasurement>> joint_state_q;
    // std::queue<cheetah_inekf_lcm::ContactsMeasurement* > contact_q;
    std::queue<std::shared_ptr<cheetah_inekf_lcm::ContactsMeasurement>> contact_q;

};

class cheetah_lcm_packet_t {
    public:
        // cheetah_lcm_packet_t(): joint_state(12) {}
        // cheetah_inekf_lcm::ImuMeasurement<double> imu;
        // cheetah_inekf_lcm::JointStateMeasurement joint_state;
        // cheetah_inekf_lcm::ContactsMeasurement contact;

        cheetah_lcm_packet_t() {};
        std::shared_ptr<cheetah_inekf_lcm::ImuMeasurement<double>> imu;
        std::shared_ptr<cheetah_inekf_lcm::JointStateMeasurement> joint_state;
        std::shared_ptr<cheetah_inekf_lcm::ContactsMeasurement> contact;
  
        // Setters
        MeasurementType getType() const { return mtype_; }
        double getTime() { return time_; }
        void setType(MeasurementType type) { mtype_ = type; }
        void setTime(double time) { time_ = time; }
    private:
        double time_;
        MeasurementType mtype_;
};