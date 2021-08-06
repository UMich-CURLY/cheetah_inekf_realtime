#include "estimator/body_estimator.hpp"

#include "H_Body_to_FrontLeftFoot.h"
#include "H_Body_to_FrontRightFoot.h"
#include "H_Body_to_HindLeftFoot.h"
#include "H_Body_to_HindRightFoot.h"
#include "Jp_Body_to_FrontLeftFoot.h"
#include "Jp_Body_to_FrontRightFoot.h"
#include "Jp_Body_to_HindLeftFoot.h"
#include "Jp_Body_to_HindRightFoot.h"

BodyEstimator::BodyEstimator() :
    t_prev_(0), imu_prev_(Eigen::Matrix<double,6,1>::Zero()) {

    // Create private node handle
    ros::NodeHandle nh("~");
    // Set noise parameters
    inekf::NoiseParams params;
    double std;
    if (nh.getParam("noise/gyroscope_std", std)) { 
        params.setGyroscopeNoise(std);
    }
    if (nh.getParam("noise/accelerometer_std", std)) { 
        params.setAccelerometerNoise(std);
    }
    if (nh.getParam("noise/gyroscope_bias_std", std)) { 
        params.setGyroscopeBiasNoise(std);
    }
    if (nh.getParam("noise/accelerometer_bias_std", std)) { 
        params.setAccelerometerBiasNoise(std);
    }
    if (nh.getParam("noise/contact_std", std)) { 
        params.setContactNoise(std);
    }
    filter_.setNoiseParams(params);
    std::cout << "Noise parameters are initialized to: \n";
    std::cout << filter_.getNoiseParams() << std::endl;
    // Settings
    nh.param<bool>("settings/static_bias_initialization", static_bias_initialization_, false);
}


bool BodyEstimator::biasInitialized() { return bias_initialized_; }
bool BodyEstimator::enabled() { return enabled_; }
void BodyEstimator::enableFilter() {enabled_ = true; }

void BodyEstimator::propagateIMU(cheetah_lcm_packet_t& cheetah_data, CheetahState& state) {
    // Initialize bias from initial robot condition
    if (!bias_initialized_) {
        initBias(cheetah_data);
    }

    // Extract out current IMU data [w;a]
    Eigen::Matrix<double,6,1> imu;
    imu << cheetah_data.imu.angular_velocity.x,
           cheetah_data.imu.angular_velocity.y, 
           cheetah_data.imu.angular_velocity.z,
           cheetah_data.imu.linear_acceleration.x, 
           cheetah_data.imu.linear_acceleration.y, 
           cheetah_data.imu.linear_acceleration.z;
    double t = cheetah_data.getTime();

    // Propagate state based on IMU and contact data
    double dt = t - t_prev_;
    if (dt > 0)
        filter_.Propagate(imu_prev_, dt); 

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

// Assumes state contacts have been updated
void BodyEstimator::setContacts(CheetahState& state) {
    // Update contact information
    const double CONTACT_THRESHOLD = 1;
    std::vector<std::pair<int,bool>> contacts;
    contacts.push_back(std::pair<int,bool> (0, (state.getLeftFrontContact()>=CONTACT_THRESHOLD) ? true:false)); 
    contacts.push_back(std::pair<int,bool> (1, (state.getLeftHindContact()>=CONTACT_THRESHOLD) ? true:false)); 
    contacts.push_back(std::pair<int,bool> (2, (state.getRightFrontContact()>=CONTACT_THRESHOLD) ? true:false)); 
    contacts.push_back(std::pair<int,bool> (3, (state.getRightHindContact()>=CONTACT_THRESHOLD) ? true:false)); 
    filter_.setContacts(contacts); // Set new contact states
}

// Assumes state encoders have been updated
void BodyEstimator::correctKinematics(CheetahState& state) {
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
}

void BodyEstimator::initBias(cheetah_lcm_packet_t& cheetah_data) {
    if (!static_bias_initialization_) {
        bias_initialized_ = true;
        return;
    }
    // Initialize bias based on imu orientation and static assumption
    if (bias_init_vec_.size() < 2000) {
        Eigen::Vector3d w, a;
        w << cheetah_data.imu.angular_velocity.x, 
             cheetah_data.imu.angular_velocity.y, 
             cheetah_data.imu.angular_velocity.z;
        a << cheetah_data.imu.linear_acceleration.x,
             cheetah_data.imu.linear_acceleration.y,
             cheetah_data.imu.linear_acceleration.z;
        Eigen::Quaternion<double> quat(cheetah_data.imu.orientation.w, 
                                       cheetah_data.imu.orientation.x,
                                       cheetah_data.imu.orientation.y,
                                       cheetah_data.imu.orientation.z); 
        Eigen::Matrix3d R = quat.toRotationMatrix();
        Eigen::Vector3d g; g << 0,0,-9.81;
        a = (R.transpose()*(R*a + g)).eval();
        Eigen::Matrix<double,6,1> v; 
        v << w(0),w(1),w(2),a(0),a(1),a(2);
        bias_init_vec_.push_back(v); // Store imu data with gravity removed
    } else {
        // Compute average bias of stored data
        Eigen::Matrix<double,6,1> avg = Eigen::Matrix<double,6,1>::Zero();
        for (int i=0; i<bias_init_vec_.size(); ++i) {
            avg = (avg + bias_init_vec_[i]).eval();
        }
        avg = (avg/bias_init_vec_.size()).eval();
        std::cout << "IMU bias initialized to: " << avg.transpose() << std::endl;
        bg0_ = avg.head<3>();
        ba0_ = avg.tail<3>();
        bias_initialized_ = true;
    }
}