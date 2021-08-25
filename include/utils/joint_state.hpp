#pragma once
#include "utils/measurement.h"
#include <stdint.h>
#include <string>

namespace cheetah_inekf_lcm {
    class JointStateMeasurement : public Measurement {
        public:
            // Construct Encoder measurement
            JointStateMeasurement(unsigned int ENCODER_DIM) {
                type_ = JOINT_STATE;
                encoder_dim_ = ENCODER_DIM;
            }
        
            // void setKinematicsArray(const inekf_msgs::KinematicsArray& kinematics) {
            //     kin_arr = kinematics;
            // }

            // const inekf_msgs::KinematicsArray& getKinematicsArray() {
            //     return kin_arr;
            // }

            Eigen::VectorXd joint_position;
            Eigen::VectorXd joint_velocity;
            Eigen::VectorXd joint_effort;
            
            // std::vector<T> joint_position;
            // std::vector<T> joint_velocity;
            // std::vector<T> joint_effort;
        private:
            unsigned int encoder_dim_;
            // inekf_msgs::KinematicsArray kin_arr;
    };
}