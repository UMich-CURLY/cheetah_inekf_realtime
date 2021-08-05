/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Measurement.h
 *  @author Ross Hartley
 *  @brief  Header file for Measurement class
 *  @date   September 27, 2018
 **/

#ifndef MEASUREMENT_H
#define MEASUREMENT_H 
#include <Eigen/Dense>
#include <string>
#include "ros/ros.h"
#include "inekf_msgs/ContactArray.h"
#include "inekf_msgs/KinematicsArray.h"
#include "inekf_msgs/LandmarkArray.h"
#include "InEKF.h"
#include "tf/transform_listener.h"

enum MeasurementType {EMPTY, IMU, LANDMARK, KINEMATIC, CONTACT};

class Measurement {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Measurement();
        virtual ~Measurement() = default;

        struct MeasurementHeader {
            uint64_t seq;
            ros::Time stamp;
            std::string frame_id;
        };

        double getTime();
        MeasurementType getType();

        friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    protected:
        double t_;
        MeasurementType type_;
};

struct MeasurementCompare {
  bool operator()(const std::shared_ptr<Measurement> lhs, const std::shared_ptr<Measurement> rhs) const {
    return lhs->getTime() > rhs->getTime();
  }
};

#endif 
