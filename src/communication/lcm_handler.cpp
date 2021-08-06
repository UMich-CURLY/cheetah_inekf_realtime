#include "communication/lcm_handler.hpp"

namespace cheetah_inekf_lcm {
  template <unsigned int ENCODER_DIM>
  void InEKF_lcm<ENCODER_DIM>::imu_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                   const std::string& channel_name,
                                   const imu_t* msg) {
    ROS_DEBUG_STREAM("Receive new imu msg");
    seq_imu_data_++;

    /// TODO: just save imu_t directly into circular queue
    ImuMeasurement<double>* imu_msg = new ImuMeasurement<double>();
    imu_msg->header.seq = seq_imu_data_;
    imu_msg->header.stamp = ros::Time::now().toSec();
    imu_msg->header.frame_id = "/cheetah/imu";
    imu_msg->orientation.w = msg->quat[0];
    imu_msg->orientation.x = msg->quat[1];
    imu_msg->orientation.y = msg->quat[2];
    imu_msg->orientation.z = msg->quat[3];
    imu_msg->angular_velocity.x = msg->omega[0];
    imu_msg->angular_velocity.y = msg->omega[1];
    imu_msg->angular_velocity.z = msg->omega[2];
    imu_msg->linear_acceleration.x = msg->acc[0];
    imu_msg->linear_acceleration.y = msg->acc[1];
    imu_msg->linear_acceleration.z = msg->acc[2];

    if (debug_enabled_) {
        // ROS_INFO("imu acc x: %0.4f y: %0.4f z: %0.4f\n", imu_msg.linear_acceleration.x,
        //         imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
    }
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    cheetah_data_in_->imu_q.push_back(imu_msg);
  }

  template <unsigned int ENCODER_DIM>
  void InEKF_lcm<ENCODER_DIM>::joint_state_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                           const std::string& channel_name,
                                           const legcontrol_t* msg) {
    ROS_DEBUG_STREAM("Receive new joint_state msg");    
    seq_joint_state_++;

    KinematicsMeasurement<double>* kinematics_ptr = new KinematicsMeasurement<double>();
    std::vector<double> joint_position(ENCODER_DIM);
    std::copy(msg->q, msg->q+ENCODER_DIM, joint_position.begin());
    std::vector<double> joint_velocity(ENCODER_DIM);
    std::copy(msg->qd, msg->qd+ENCODER_DIM, joint_velocity.begin());
    std::vector<double> joint_effort(ENCODER_DIM);
    std::copy(msg->tau_est, msg->tau_est+ENCODER_DIM, joint_effort.begin());
    Eigen::Matrix<double, ENCODER_DIM, 1> encoder_pos;
    for (int j = 0; j < ENCODER_DIM; j++)
        encoder_pos(j) = joint_position[j];
    kinematics_ptr->position = joint_position;
    kinematics_ptr->velocity = joint_velocity;
    kinematics_ptr->effort = joint_effort;

    if (debug_enabled_) {
        sensor_msgs::JointState joint_state_msg;
        joint_state_msg.header.seq = seq_joint_state_;
        joint_state_msg.header.stamp = ros::Time::now();
        joint_state_msg.position = joint_position;
        joint_state_msg.velocity = joint_velocity;
        joint_state_msg.effort = joint_effort;
        joint_state_publisher_.publish(joint_state_msg);
        joint_state_msg.header.frame_id = "/cheetah/joint_state";
    }

    std_msgs::Header header;
    kinematics_ptr->setKinematicsArray(
        KinematicsHandler<ENCODER_DIM>::callback_handler(header, encoder_pos, cov_encoders_, cov_prior_)
    );

    kinematics_ptr->header.seq = seq_joint_state_;
    kinematics_ptr->header.stamp = ros::Time::now().toSec();
    kinematics_ptr->header.frame_id =  "/cheetah/imu";
    // kinematics_publisher_.publish(kinematics_arr);
    if (debug_enabled_) {
        kinematics_debug_ << kinematics_ptr->getKinematicsArray() << '\n';
        std::cout << kinematics_ptr->getKinematicsArray() << '\n';
    }

    boost::mutex::scoped_lock lock(*cdata_mtx_);
    cheetah_data_in_->kin_q.push_back(kinematics_ptr);
  }

  template <unsigned int ENCODER_DIM>
  void InEKF_lcm<ENCODER_DIM>::contact_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                       const std::string& channel_name,
                                       const contact_t* msg) {
    ROS_DEBUG_STREAM("Receive new contact msg");        
    seq_contact_++;

    cheetah_inekf_lcm::ContactsMeasurement* contact_ptr = new ContactsMeasurement();
    contact_ptr->header.seq = seq_contact_;
    contact_ptr->header.stamp = ros::Time::now().toSec();
    contact_ptr->header.frame_id = "/cheetah/contact";

    std::vector<inekf_msgs::Contact> contacts;

    for (int i = 0; i < 4; i++)
    {
      inekf_msgs::Contact ct;
      ct.id = i;
      ct.indicator =  msg->contact[i] > 0;
      contacts.push_back(ct);
    }
    contact_ptr->setContacts(contacts);

    if (debug_enabled_) {
        // std::cout << "Contacts " << contact_ptr->getContacts() << '\n';
    }
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    cheetah_data_in_->contact_q.push_back(contact_ptr);
  }



} // mini_cheetah

template class cheetah_inekf_lcm::KinematicsHandler<12>;
template class cheetah_inekf_lcm::InEKF_lcm<12>;
