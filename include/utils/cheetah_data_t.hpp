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

#include <boost/circular_buffer.hpp>

struct cheetah_lcm_data_t {
    cheetah_lcm_data_t() {}
    cheetah_lcm_data_t(uint32_t qsz): qsz_(qsz) {
        imu_q.resize(qsz_);
        kin_q.resize(qsz_);
        contact_q.resize(qsz_);
    }

    uint32_t qsz_;
    boost::circular_buffer<cheetah_inekf_lcm::ImuMeasurement<double>* > imu_q;
    boost::circular_buffer<cheetah_inekf_lcm::KinematicsMeasurement<double>* > kin_q;
    boost::circular_buffer<cheetah_inekf_lcm::ContactsMeasurement* > contact_q;
};

class cheetah_lcm_packet_t {
    public:
        cheetah_inekf_lcm::ImuMeasurement<double> imu_q;
        cheetah_inekf_lcm::KinematicsMeasurement<double> kin_q;
        cheetah_inekf_lcm::ContactsMeasurement contact_q;

        // Setters
        inline MeasurementType getType() { return mtype_; }
        inline double getTime() { return time_; }
        inline void setType(MeasurementType type) { mtype_ = type; }
        inline void setTime(double time) { time_ = time; }
    private:
        double time_;
        MeasurementType mtype_;
};