#pragma once

//STL
#include <iostream>
#include <fstream>
#include <memory>

// Utility types
#include "utils/cheetah_data_t.hpp"
#include "sensor_msgs/Imu.h"

// ROS related
#include <ros/ros.h>
// #include "kinematics_handler.hpp"

// LCM related
#include <lcm/lcm-cpp.hpp>
#include "lcm-types/cheetah_inekf_lcm/imu_t.hpp"
#include "lcm-types/cheetah_inekf_lcm/legcontrol_t.hpp"
#include "lcm-types/cheetah_inekf_lcm/contact_t.hpp"
#include "lcm-types/cheetah_inekf_lcm/synced_proprioceptive_lcmt.hpp"

// Threading
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

namespace cheetah_inekf_lcm {

template <unsigned int ENCODER_DIM>
class lcm_handler {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    lcm_handler(lcm::LCM* lcm, ros::NodeHandle* n, cheetah_lcm_data_t* cheetah_buffer, boost::mutex* cdata_mtx) : 
        lcm_(lcm), nh_(n), cheetah_buffer_(cheetah_buffer), cdata_mtx_(cdata_mtx) {
        assert(lcm_);  // confirm a nullptr wasn't passed in
        ROS_INFO("Cheetah_Lcm ready to initialize...."); 

        /// SYNCED:
        lcm_->subscribe("synced_proprioceptive_data", &cheetah_inekf_lcm::lcm_handler<12>::synced_msgs_lcm_callback, this);
	    
	    seq_imu_data_ = 0;
        seq_joint_state_ = 0;
        seq_contact_ = 0;

        //Set private variables
        double encoder_std, kinematic_prior_orientation_std, kinematic_prior_position_std;
        std::string project_root_dir;
        nh_->param<double>("/inekf/encoder_std", encoder_std, 0.0174533); // 1 deg std
        nh_->param<double>("/inekf/kinematic_prior_orientation_std", kinematic_prior_orientation_std, 0.174533); // 10 deg std
        nh_->param<double>("/inekf/kinematic_prior_position_std", kinematic_prior_position_std, 0.05); // 5cm std
        nh_->param<bool>("/settings/lcm_enable_debug_output", debug_enabled_, false);
        nh_->param<std::string>("/settings/project_root_dir", project_root_dir, "../../../");

        //Debugging ROS messages
        // imu_publisher_ = nh_->advertise<sensor_msgs::Imu>("imu", 10);
        // joint_state_publisher_ = nh_->advertise<sensor_msgs::JointState>("joint_state", 10);
        // kinematics_publisher_ = nh_->advertise<inekf_msgs::KinematicsArray>("kinematics", 10);
        // contact_publisher_ = nh_->advertise<inekf_msgs::ContactArray>("contact", 10);

        cov_encoders_ = encoder_std*encoder_std*Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM>::Identity(); 
        cov_prior_ = Eigen::Matrix<double,6,6>::Identity();
        cov_prior_.block<3,3>(0,0) = kinematic_prior_orientation_std*kinematic_prior_orientation_std*Eigen::Matrix<double,3,3>::Identity();
        cov_prior_.block<3,3>(3,3) = kinematic_prior_position_std*kinematic_prior_position_std*Eigen::Matrix<double,3,3>::Identity();

        ROS_INFO("Cheetah_Lcm initialized."); 
        if (debug_enabled_) {
            std::cout << project_root_dir << "/tests/kinematics/lcmlog.out" << '\n';
            kinematics_debug_.open(project_root_dir + "/tests/kinematics/lcmlog.out");
            assert(kinematics_debug_.is_open());
        }
    }

    void synced_msgs_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const synced_proprioceptive_lcmt* msg);
  
  private:
    lcm::LCM* lcm_;
    ros::NodeHandle* nh_;
    // ros::Publisher imu_publisher_;
    // ros::Publisher joint_state_publisher_;
    // ros::Publisher contact_publisher_;
    // ros::Publisher kinematics_publisher_;

    boost::mutex* cdata_mtx_;

    Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM> cov_encoders_;
    Eigen::Matrix<double,6,6> cov_prior_;

    uint64_t seq_imu_data_;
    uint64_t seq_joint_state_;
    uint64_t seq_contact_;

    cheetah_lcm_data_t* cheetah_buffer_;

    //Debugging
    bool debug_enabled_;
    std::ofstream kinematics_debug_;
};

} // namespace mini_cheetah
