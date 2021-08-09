#pragma once
// Utility libraries
#include "utils/imu.hpp"
#include "utils/kinematics.hpp"
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

    std::queue<cheetah_inekf_lcm::ImuMeasurement<double>* > imu_q;
    std::queue<cheetah_inekf_lcm::KinematicsMeasurement<double>* > kin_q;
    std::queue<cheetah_inekf_lcm::ContactsMeasurement* > contact_q;
};

class cheetah_lcm_packet_t {
    public:
        cheetah_inekf_lcm::ImuMeasurement<double> imu;
        cheetah_inekf_lcm::KinematicsMeasurement<double> kin;
        cheetah_inekf_lcm::ContactsMeasurement contact;
  
        // Setters
        MeasurementType getType() { return mtype_; }
        double getTime() { return time_; }
        void setType(MeasurementType type) { mtype_ = type; }
        void setTime(double time) { time_ = time; }
    private:
        double time_;
        MeasurementType mtype_;
};