#pragma once

// Utility libraries
#include <boost/circular_buffer.hpp>>

// ROS related
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "inekf_msgs/ContactArray.h"
#include "KinematicsHandler.hpp"

// LCM related
#include <lcm/lcm-cpp.hpp>
#include "lcm-types/imu_t.hpp"
#include "lcm-types/legcontrol_t.hpp"
#include "lcm-types/contact_t.hpp"

namespace cheetah_inekf_ros {
  template <unsigned int ENCODER_DIM>
class InEKF_lcm {
  public:
    InEKF_lcm(LCM::lcm& lcm) : nh_("~"), lcm_(lcm) {
      
      ROS_INFO("Cheetah_Lcm ready to initialize...."); 
	    
	    seq_imu_data_ = 0;
      seq_joint_state_ = 0;
      seq_contact_ = 0;

      //Set private variables
      double encoder_std, kinematic_prior_orientation_std, kinematic_prior_position_std;
      nh_.param<double>("encoder_std", encoder_std, 0.0174533); // 1 deg std
      nh_.param<double>("kinematic_prior_orientation_std", kinematic_prior_orientation_std, 0.174533); // 10 deg std
      nh_.param<double>("kinematic_prior_position_std", kinematic_prior_position_std, 0.05); // 5cm std

      //TODO: make size a parameter
      imu_queue.resize(100);
      kin_queue.resize(100);
      contact_queue.resize(100);
      imu_queue_pos_, kin_queue_pos_, contact_queue_pos_ = 0, 0, 0;

      cov_encoders_ = encoder_std*encoder_std*Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM>::Identity(); 
      cov_prior_ = Eigen::Matrix<double,6,6>::Identity();
      cov_prior_.block<3,3>(0,0) = kinematic_prior_orientation_std*kinematic_prior_orientation_std*Eigen::Matrix<double,3,3>::Identity();
      cov_prior_.block<3,3>(3,3) = kinematic_prior_position_std*kinematic_prior_position_std*Eigen::Matrix<double,3,3>::Identity();

      ROS_INFO("Cheetah_Lcm initialized."); 
    }

    void imu_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                        const std::string& channel_name,
                        const imu_t* msg);
    
    void joint_state_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const legcontrol_t* msg);

    void contact_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                               const std::string& channel_name,
                               const contact_t* msg);
  
  private:
    ros::NodeHandle nh_;

    Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM> cov_encoders_;
    Eigen::Matrix<double,6,6> cov_prior_;
    lcm::LCM lcm_;

  //std::string imu_lcm_topic, joint_state_lcm_topic, contact_lcm_topic;

    uint64_t seq_imu_data_;
    uint64_t seq_joint_state_;
    uint64_t seq_contact_;

    uint32_t imu_queue_pos_, kin_queue_pos_, contact_queue_pos_;
    boost::circular_buffer<sensor_msgs::Imu> imu_queue;
    boost::circular_buffer<sensor_msgs::inekf_msgs::KinematicsArray> kin_queue;
    boost::circular_buffer<inekf_msgs::ContactArray> contact_queue;
};

} // namespace mini_cheetah
