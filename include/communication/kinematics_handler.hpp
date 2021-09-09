#pragma once

#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>
#include "inekf_msgs/KinematicsArray.h"
#include "sensor_msgs/JointState.h"

// forward kinematics from FROST
#include "../kin/H_Body_to_FrontLeftFoot.h"
#include "../kin/H_Body_to_FrontRightFoot.h"
#include "../kin/H_Body_to_HindLeftFoot.h"
#include "../kin/H_Body_to_HindRightFoot.h"
#include "../kin/Jb_Body_to_FrontLeftFoot.h"
#include "../kin/Jb_Body_to_FrontRightFoot.h"
#include "../kin/Jb_Body_to_HindLeftFoot.h"
#include "../kin/Jb_Body_to_HindRightFoot.h"


namespace cheetah_inekf_lcm {
template <unsigned int ENCODER_DIM>
class KinematicsHandler {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        KinematicsHandler() {
        }

        static inekf_msgs::KinematicsArray callback_handler(std_msgs::Header & header,  const Eigen::Matrix<double, ENCODER_DIM, 1> & encoders,
                                                            const Eigen::Matrix<double, ENCODER_DIM, ENCODER_DIM> & cov_encoder,
                                                            const Eigen::Matrix<double, 6,6> & cov_prior) {
            inekf_msgs::Kinematics front_left_foot, front_right_foot, hind_left_foot, hind_right_foot;
            
            Eigen::Matrix<double,3,1> p_offset; p_offset << 0, -0.02, 0; // TODO: tune
            Eigen::Matrix<double,4,4> H_offset = Eigen::Matrix<double,4,4>::Identity();
            H_offset.block<3,1>(0,3) = p_offset;

            Eigen::Matrix<double,4,4> H_FL = H_Body_to_FrontLeftFoot(encoders)*H_offset;
            Eigen::Matrix<double,4,4> H_FR = H_Body_to_FrontRightFoot(encoders)*H_offset;
            Eigen::Matrix<double,4,4> H_HL = H_Body_to_HindLeftFoot(encoders)*H_offset;
            Eigen::Matrix<double,4,4> H_HR = H_Body_to_HindRightFoot(encoders)*H_offset;

            Eigen::Quaternion<double> q_FL(H_FL.block<3,3>(0,0)); q_FL.normalize();
            Eigen::Quaternion<double> q_FR(H_FR.block<3,3>(0,0)); q_FR.normalize();
            Eigen::Quaternion<double> q_HL(H_HL.block<3,3>(0,0)); q_HL.normalize();
            Eigen::Quaternion<double> q_HR(H_HR.block<3,3>(0,0)); q_HR.normalize();
  
            Eigen::Matrix<double,6,ENCODER_DIM> J_FL = Jb_Body_to_FrontLeftFoot(encoders); // body manipulator Jacobian
            Eigen::Matrix<double,6,ENCODER_DIM> J_FR = Jb_Body_to_FrontRightFoot(encoders); // body manipulator Jacobian
            Eigen::Matrix<double,6,ENCODER_DIM> J_HL = Jb_Body_to_HindLeftFoot(encoders); // body manipulator Jacobian
            Eigen::Matrix<double,6,ENCODER_DIM> J_HR = Jb_Body_to_HindRightFoot(encoders); // body manipulator Jacobian

            Eigen::Matrix<double,6,6> cov_FL = J_FL * cov_encoder * J_FL.transpose() + cov_prior;
            Eigen::Matrix<double,6,6> cov_FR = J_FR * cov_encoder * J_FR.transpose() + cov_prior;
            Eigen::Matrix<double,6,6> cov_HL = J_HL * cov_encoder * J_HL.transpose() + cov_prior;
            Eigen::Matrix<double,6,6> cov_HR = J_HR * cov_encoder * J_HR.transpose() + cov_prior;
            
            front_left_foot.id = 1;
            front_left_foot.pose.pose.orientation.w = q_FL.w();
            front_left_foot.pose.pose.orientation.x = q_FL.x();
            front_left_foot.pose.pose.orientation.y = q_FL.y();
            front_left_foot.pose.pose.orientation.z = q_FL.z();
            front_left_foot.pose.pose.position.x = H_FL(0,3);
            front_left_foot.pose.pose.position.y = H_FL(1,3);
            front_left_foot.pose.pose.position.z = H_FL(2,3);
            for (int i=0; i<6; ++i) {
                for (int j=0; j<6; ++j) {
                front_left_foot.pose.covariance[i*6+j] = cov_FL(i,j);
                }
            }

            front_right_foot.id = 0;
            front_right_foot.pose.pose.orientation.w = q_FR.w();
            front_right_foot.pose.pose.orientation.x = q_FR.x();
            front_right_foot.pose.pose.orientation.y = q_FR.y();
            front_right_foot.pose.pose.orientation.z = q_FR.z();
            front_right_foot.pose.pose.position.x = H_FR(0,3);
            front_right_foot.pose.pose.position.y = H_FR(1,3);
            front_right_foot.pose.pose.position.z = H_FR(2,3);
            for (int i=0; i<6; ++i) {
                for (int j=0; j<6; ++j) {
                front_right_foot.pose.covariance[i*6+j] = cov_FR(i,j);
                }
            }

            hind_left_foot.id = 3;
            hind_left_foot.pose.pose.orientation.w = q_HL.w();
            hind_left_foot.pose.pose.orientation.x = q_HL.x();
            hind_left_foot.pose.pose.orientation.y = q_HL.y();
            hind_left_foot.pose.pose.orientation.z = q_HL.z();
            hind_left_foot.pose.pose.position.x = H_HL(0,3);
            hind_left_foot.pose.pose.position.y = H_HL(1,3);
            hind_left_foot.pose.pose.position.z = H_HL(2,3);
            for (int i=0; i<6; ++i) {
                for (int j=0; j<6; ++j) {
                hind_left_foot.pose.covariance[i*6+j] = cov_HL(i,j);
                }
            }

            hind_right_foot.id = 2;
            hind_right_foot.pose.pose.orientation.w = q_HR.w();
            hind_right_foot.pose.pose.orientation.x = q_HR.x();
            hind_right_foot.pose.pose.orientation.y = q_HR.y();
            hind_right_foot.pose.pose.orientation.z = q_HR.z();
            hind_right_foot.pose.pose.position.x = H_HR(0,3);
            hind_right_foot.pose.pose.position.y = H_HR(1,3);
            hind_right_foot.pose.pose.position.z = H_HR(2,3);
            for (int i=0; i<6; ++i) {
                for (int j=0; j<6; ++j) {
                hind_right_foot.pose.covariance[i*6+j] = cov_HR(i,j);
                }
            }
 
            inekf_msgs::KinematicsArray kinematics_msg;
            kinematics_msg.frames.push_back(front_right_foot);
            kinematics_msg.frames.push_back(front_left_foot);
            kinematics_msg.frames.push_back(hind_right_foot);
            kinematics_msg.frames.push_back(hind_left_foot);
            //kinematic_pub_.publish(kinematics_msg);

            return kinematics_msg;
            
        }
        
    private:
        std::string imu_frame_id_;
        //const unsigned int encoder_dim_;
        Eigen::Matrix<double,ENCODER_DIM,ENCODER_DIM> cov_encoders_;
        Eigen::Matrix<double,6,6> cov_prior_;
  }; 
  
}
