#pragma once


#include <Eigen/Dense>
#include "estimator/body_estimator.hpp"
// #include "ControllerBase.h"
#include <iostream>
#include <memory>
#include "ros/ros.h"
// #include "RosPublisher.h"
// #include "PassiveTimeSync.h"

// TODO: Singleton design pattern (there should only be one CassieSystem)
class CheetahState {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        // Default Contructor
        CheetahState(ros::NodeHandle n);

    private:

};


