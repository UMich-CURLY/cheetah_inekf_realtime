#ifndef CHEETAHSYSTEM_H
#define CHEETAHSYSTEM_H

#include <Eigen/Dense>
#include "estimator/body_estimator.hpp"
#include "system/cheetah_state.hpp"
#include <iostream>
#include <memory>
#include "ros/ros.h"
// #include "PassiveTimeSync.h"

// TODO: Singleton design pattern (there should only be one CassieSystem)
class CheetahSystem {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Default Contructor
        CheetahSystem(ros::NodeHandle n);
        // Step forward one iteration of the system
        // *uses the sensor outputs to compute the motor inputs to send via LCM
        // void step(const cassie_slrt_data_t *slrt_data, cassie_linux_data_t *linux_data);
        // Set the current estimator
        void setEstimator(std::shared_ptr<BodyEstimator> estimator);

    private:
        // ROS timestamp
        ros::Time timestamp_;
        // Cassie's current state estimate
        CheetahState state_;
        // Invariant extended Kalman filter for estimating the robot's body state
        std::shared_ptr<BodyEstimator> estimator_;
};

#endif // CHEETAHSYSTEM_H