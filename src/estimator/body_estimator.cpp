#include "estimator/body_estimator.hpp"

#include "H_Body_to_FrontLeftFoot.h"
#include "H_Body_to_FrontRightFoot.h"
#include "H_Body_to_HindLeftFoot.h"
#include "H_Body_to_HindRightFoot.h"
#include "Jp_Body_to_FrontLeftFoot.h"
#include "Jp_Body_to_FrontRightFoot.h"
#include "Jp_Body_to_HindLeftFoot.h"
#include "Jp_Body_to_HindRightFoot.h"

bool BodyEstimator::enabled() {
    return enabled_;
}

// Update filter based on sensor outputs
void BodyEstimator::update(cheetah_lcm_packet_t& cheetah_data, CheetahState& state) {
    // Extract out current IMU data [w;a]
    Eigen::Matrix<double,6,1> imu;
    imu << cheetah_data.imu_q.angular_velocity.x,
           cheetah_data.imu_q.angular_velocity.y, 
           cheetah_data.imu_q.angular_velocity.z,
           cheetah_data.imu_q.linear_acceleration.x, 
           cheetah_data.imu_q.linear_acceleration.y, 
           cheetah_data.imu_q.linear_acceleration.z;
    double t = cheetah_data.t.toSec();
    
    // Update contact information
    const double CONTACT_THRESHOLD = 1;
    std::vector<std::pair<int,bool>> contacts;
    contacts.push_back(std::pair<int,bool> (0, (state.getLeftFrontContact()>=CONTACT_THRESHOLD) ? true:false)); 
    contacts.push_back(std::pair<int,bool> (1, (state.getLeftBackContact()>=CONTACT_THRESHOLD) ? true:false)); 
    contacts.push_back(std::pair<int,bool> (2, (state.getRightFrontContact()>=CONTACT_THRESHOLD) ? true:false)); 
    contacts.push_back(std::pair<int,bool> (3, (state.getRightBackContact()>=CONTACT_THRESHOLD) ? true:false)); 
    filter_.setContacts(contacts); // Set new contact states

    // Propagate state based on IMU and contact data
    double dt = t - t_prev_;
    if (dt > 0)
        filter_.Propagate(imu_prev_, dt); 

    // Correct state based on kinematics measurements (probably in cheetah inekf ros)
    Eigen::Matrix<double,12,1> encoders = state.getEncoderPositions();
    Eigen::Matrix4d FL = H_Body_to_FrontLeftFoot(encoders); 
    Eigen::Matrix4d FR = H_Body_to_FrontRightFoot(encoders);
    Eigen::Matrix4d HL = H_Body_to_HindLeftFoot(encoders); 
    Eigen::Matrix4d HR = H_Body_to_HindRightFoot(encoders);
    Eigen::Matrix<double,3,12> JpFL = Jp_Body_to_FrontLeftFoot(encoders);
    Eigen::Matrix<double,3,12> JpFR = Jp_Body_to_FrontRightFoot(encoders);
    Eigen::Matrix<double,3,12> JpHL = Jp_Body_to_HindLeftFoot(encoders);
    Eigen::Matrix<double,3,12> JpHR = Jp_Body_to_HindRightFoot(encoders);
    Eigen::Matrix<double,6,6> covFL = Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,6> covFR = Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,6> covHL = Eigen::Matrix<double,6,6>::Identity();
    Eigen::Matrix<double,6,6> covHR = Eigen::Matrix<double,6,6>::Identity();
    covFL.block<3,3>(3,3) = JpFL*encoder_cov_*JpFL.transpose() + prior_kinematics_cov_;
    covFR.block<3,3>(3,3) = JpFR*encoder_cov_*JpFR.transpose() + prior_kinematics_cov_;
    covHL.block<3,3>(3,3) = JpHL*encoder_cov_*JpHL.transpose() + prior_kinematics_cov_;
    covHR.block<3,3>(3,3) = JpHR*encoder_cov_*JpHR.transpose() + prior_kinematics_cov_;
    inekf::Kinematics leftFrontFoot(0, FL, covFL);
    inekf::Kinematics leftHindFoot(1, HL, covHL);
    inekf::Kinematics rightFrontFoot(2, FR, covFR);
    inekf::Kinematics rightHindFoot(3, HR, covHR);
    inekf::vectorKinematics kinematics;
    kinematics.push_back(leftFrontFoot);
    kinematics.push_back(leftHindFoot);
    kinematics.push_back(rightFrontFoot);
    kinematics.push_back(rightHindFoot);  
    filter_.CorrectKinematics(kinematics);

    // --- Test abosulte positon contact measurement (z) --- //
    // Eigen::Vector3d measurement; measurement << 0,0,0; // Measure 0 ground height
    // Eigen::Matrix3d covariance = 0.01*Eigen::Matrix3d::Identity();
    // Eigen::Vector3d indices; indices << 0,0,1; // Specify that we are measuring only the z component
    // if (state.getLeftContact()>=CONTACT_THRESHOLD)
    //     filter_.CorrectContactPosition(0, measurement, covariance, indices);
    // if (state.getRightContact()>=CONTACT_THRESHOLD)
    //     filter_.CorrectContactPosition(1, measurement, covariance, indices);

    // Update Cheetah's state (Change from IMU to body frame)
    ///TODO: Check if imu strapdown model is correct
    inekf::RobotState estimate = filter_.getState();
    Eigen::Vector3d i_p_ib; i_p_ib << -0.0316, 0, 0.08;
    Eigen::Vector3d w = imu.head<3>() - estimate.getGyroscopeBias(); // Angular velocity without bias
    Eigen::Matrix3d R = estimate.getRotation(); // no extra rotation needed
    Eigen::Vector3d p = estimate.getPosition() + R*i_p_ib;
    Eigen::Vector3d v = estimate.getVelocity() + R*inekf::skew(w)*i_p_ib;
    state.setBaseRotation(R);
    state.setBasePosition(p);
    state.setBaseVelocity(v);

    // Store previous imu data
    t_prev_ = t;
    imu_prev_ = imu;
}
