#pragma once

// Utility types
#include "utils/cheetah_data_t.hpp"

// ROS related
#include <ros/ros.h>
#include "communication/kinematics_handler.hpp"

// LCM related
#include <lcm/lcm-cpp.hpp>
#include "lcm-types/imu_t.hpp"
#include "lcm-types/legcontrol_t.hpp"
#include "lcm-types/contact_t.hpp"

namespace cheetah_inekf_lcm {

template <unsigned int ENCODER_DIM>
class InEKF_lcm {
  public:
      InEKF_lcm(lcm::LCM& lcm, cheetah_lcm_data_t* cheetah_data_in, boost::mutex* cdata_mtx) : lcm_(lcm), nh_("~"),
        cheetah_data_in(cheetah_data_in), cdata_mtx_(cdata_mtx) {
      
      ROS_INFO("Cheetah_Lcm ready to initialize...."); 
	    
	    seq_imu_data_ = 0;
      seq_joint_state_ = 0;
      seq_contact_ = 0;

      //Set private variables
      double encoder_std, kinematic_prior_orientation_std, kinematic_prior_position_std;
      nh_.param<double>("encoder_std", encoder_std, 0.0174533); // 1 deg std
      nh_.param<double>("kinematic_prior_orientation_std", kinematic_prior_orientation_std, 0.174533); // 10 deg std
      nh_.param<double>("kinematic_prior_position_std", kinematic_prior_position_std, 0.05); // 5cm std

      //Debugging ROS messages
      imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu", 10);
      joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("joint_state", 10);
      kinematics_publisher_ = nh_.advertise<inekf_msgs::KinematicsArray>("kinematics", 10);
      contact_publisher_ = nh_.advertise<inekf_msgs::ContactArray>("contact", 10);

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
    lcm::LCM lcm_;
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Publisher joint_state_publisher_;
    ros::Publisher contact_publisher_;
    ros::Publisher kinematics_publisher_;

    boost::mutex* cdata_mtx_;

    Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM> cov_encoders_;
    Eigen::Matrix<double,6,6> cov_prior_;

  //std::string imu_lcm_topic, joint_state_lcm_topic, contact_lcm_topic;

    uint64_t seq_imu_data_;
    uint64_t seq_joint_state_;
    uint64_t seq_contact_;

    cheetah_lcm_data_t* cheetah_data_in;
};

} // namespace mini_cheetah
