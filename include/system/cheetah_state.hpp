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
        // Default Contructor
        CheetahState() {}

    private:

};


#ifndef CASSIESTATE_H
#define CASSIESTATE_H

extern "C" { 
    #include "cassie_slrt_data_t.h"
}
#include <Eigen/Dense>
#include <iostream>
#include "utils.h"
#include "p_VectorNav_to_LeftToeBottom.h"
#include "p_VectorNav_to_RightToeBottom.h"
#include "Jp_VectorNav_to_LeftToeBottom.h"
#include "Jp_VectorNav_to_RightToeBottom.h"

class CheetahState {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        CheetahState();
        CassieState(const Eigen::Matrix<double,20,1>& q, const Eigen::Matrix<double,20,1>& dq, bool computeContacts = true);
        CassieState(const cassie_slrt_data_t *slrt_data, bool computeContacts = true);

        void set(const Eigen::Matrix<double,20,1>& q, const Eigen::Matrix<double,20,1>& dq, bool computeContacts = true);
        void set(const cassie_slrt_data_t *slrt_data, bool computeContacts = true);
        void setBaseRotation(const Eigen::Matrix3d& R);
        void setBasePosition(const Eigen::Vector3d& p);
        void setBaseVelocity(const Eigen::Vector3d& v);
        void setMotorPositions(const Eigen::Matrix<double,10,1>& qM);
        void setMotorVelocities(const Eigen::Matrix<double,10,1>& dqM);
        void clear();

        Eigen::Matrix<double,20,1> q() const;
        Eigen::Matrix<double,20,1> dq() const;
        Eigen::Vector3d getPosition() const;
        Eigen::Quaternion<double> getQuaternion() const;
        Eigen::Matrix3d getRotation() const;
        Eigen::Vector3d getEulerAngles() const;
        Eigen::Vector3d getEulerRates() const;
        Eigen::Matrix<double,14,1> getEncoderPositions() const;
        Eigen::Matrix<double,14,1> getEncoderVelocities() const;
        Eigen::Matrix<double,10,1> getMotorPositions() const;
        Eigen::Matrix<double,10,1> getMotorVelocities() const;
        Eigen::Matrix<double,4,1> getGRF() const;
        double getLeftContact() const;
        double getRightContact() const;
        Eigen::Vector3d getAngularVelocity() const;
        Eigen::Vector3d getKinematicVelocity() const;
        Eigen::Vector3d getBodyVelocity() const;
        double x() const;
        double y() const;
        double z() const;
        double yaw() const;
        double pitch() const;
        double roll() const;
        // double leftHipAbduction() const;
        // double leftHipRotation() const;
        // double leftHipFlexion() const;
        
        /// SEARCH:
        // enum {
        //     left_front     
        //     right_front   
        // };

        double leftKnee() const;
        double leftKneeSpring() const;
        double leftAnkle() const;
        double leftToePitch() const;
        // double rightHipAbduction() const;
        // double rightHipRotation() const;
        // double rightHipFlexion() const;
        double rightKnee() const;
        double rightKneeSpring() const;
        double rightAnkle() const;
        double rightToePitch() const;
        double dx() const;
        double dy() const;
        double dz() const;
        double dyaw() const;
        double dpitch() const;
        double droll() const;
        double dleftHipAbduction() const;
        double dleftHipRotation() const;
        double dleftHipFlexion() const;
        double dleftKnee() const;
        double dleftKneeSpring() const;
        double dleftAnkle() const;
        double dleftToePitch() const;
        double drightHipAbduction() const;
        double drightHipRotation() const;
        double drightHipFlexion() const;
        double drightKnee() const;
        double drightKneeSpring() const;
        double drightAnkle() const;
        double drightToePitch() const;

        friend std::ostream& operator<<(std::ostream& os, const CassieState& obj);  

    private:
        Eigen::Matrix<double, 20,1> q_;
        Eigen::Matrix<double, 20,1> dq_;
        Eigen::Matrix<double,4,1> GRF_;
        double left_contact_;
        double right_contact_;
        void EstimateContacts();
};

#endif // CONTROLLERBASE_H