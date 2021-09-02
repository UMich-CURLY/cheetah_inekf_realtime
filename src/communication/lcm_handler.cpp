#include "communication/lcm_handler.hpp"

namespace cheetah_inekf_lcm {

  template <unsigned int ENCODER_DIM>
  void lcm_handler<ENCODER_DIM>::synced_msgs_lcm_callback(const lcm::ReceiveBuffer* rbuf,
                                           const std::string& channel_name,
                                           const synced_proprioceptive_lcmt* msg) {
    ROS_DEBUG_STREAM("Receive new synchronized msg");    
    seq_joint_state_++;

    std::shared_ptr<ImuMeasurement<double>> imu_measurement_ptr = std::shared_ptr<ImuMeasurement<double>> (new ImuMeasurement<double>);
    std::shared_ptr<JointStateMeasurement> joint_state_ptr = std::shared_ptr<JointStateMeasurement> (new JointStateMeasurement(ENCODER_DIM));
    std::shared_ptr<ContactsMeasurement> contact_ptr = std::shared_ptr<ContactsMeasurement>(new ContactsMeasurement);
    
    /// TIMESTAMP:
    double timestamp = msg->timestamp;

    /// IMU:
    imu_measurement_ptr.get()->orientation.w = msg->quat[0];
    imu_measurement_ptr.get()->orientation.x = msg->quat[1];
    imu_measurement_ptr.get()->orientation.y = msg->quat[2];
    imu_measurement_ptr.get()->orientation.z = msg->quat[3];
    imu_measurement_ptr.get()->angular_velocity.x = msg->omega[0];
    imu_measurement_ptr.get()->angular_velocity.y = msg->omega[1];
    imu_measurement_ptr.get()->angular_velocity.z = msg->omega[2];
    imu_measurement_ptr.get()->linear_acceleration.x = msg->acc[0];
    imu_measurement_ptr.get()->linear_acceleration.y = msg->acc[1];
    imu_measurement_ptr.get()->linear_acceleration.z = msg->acc[2];

    /// LEG:
    joint_state_ptr.get()->joint_position = Eigen::Map<const Eigen::MatrixXf>(msg->q,ENCODER_DIM,1).cast<double>();
    joint_state_ptr.get()->joint_velocity = Eigen::Map<const Eigen::MatrixXf>(msg->qd,ENCODER_DIM,1).cast<double>();
    joint_state_ptr.get()->joint_effort = Eigen::Map<const Eigen::MatrixXf>(msg->tau_est,ENCODER_DIM,1).cast<double>();   

    /// CONTACTS:
    Eigen::Matrix<int8_t, 4, 1> contacts;
    for (int i = 0; i < msg->num_legs; ++i) {
        contacts[i] = msg->contact[i];
    }
    contact_ptr->setContacts(contacts);

    // push into queues:
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    cheetah_data_in_->timestamp_q.push(timestamp);
    cheetah_data_in_->imu_q.push(imu_measurement_ptr);
    cheetah_data_in_->joint_state_q.push(joint_state_ptr);
    cheetah_data_in_->contact_q.push(contact_ptr);
  }
} // mini_cheetah

//   template <unsigned int ENCODER_DIM>
//   void lcm_handler<ENCODER_DIM>::imu_lcm_callback(const lcm::ReceiveBuffer* rbuf,
//                                    const std::string& channel_name,
//                                    const imu_t* msg) {
//     ROS_DEBUG_STREAM("Receive new imu msg");
//     seq_imu_data_++;

//     /// TODO: just save imu_t directly into circular queue
//     ImuMeasurement<double>* imu_msg = new ImuMeasurement<double>();
//     // imu_msg->header.seq = seq_imu_data_;
//     // imu_msg->header.stamp = ros::Time::now().toSec();
//     // imu_msg->header.frame_id = "/cheetah/imu";
//     // imu_msg->orientation.w = msg->quat[0];
//     // imu_msg->orientation.x = msg->quat[1];
//     // imu_msg->orientation.y = msg->quat[2];
//     // imu_msg->orientation.z = msg->quat[3];
//     // imu_msg->angular_velocity.x = msg->omega[0];
//     // imu_msg->angular_velocity.y = msg->omega[1];
//     // imu_msg->angular_velocity.z = msg->omega[2];
//     // imu_msg->linear_acceleration.x = msg->acc[0];
//     // imu_msg->linear_acceleration.y = msg->acc[1];
//     // imu_msg->linear_acceleration.z = msg->acc[2];

//     std::shared_ptr<ImuMeasurement<double>> imu_measurement_ptr = std::shared_ptr<ImuMeasurement<double>> (new ImuMeasurement<double>);
//     imu_measurement_ptr.get()->orientation.w = msg->quat[0];
//     imu_measurement_ptr.get()->orientation.x = msg->quat[1];
//     imu_measurement_ptr.get()->orientation.y = msg->quat[2];
//     imu_measurement_ptr.get()->orientation.z = msg->quat[3];
//     imu_measurement_ptr.get()->angular_velocity.x = msg->omega[0];
//     imu_measurement_ptr.get()->angular_velocity.y = msg->omega[1];
//     imu_measurement_ptr.get()->angular_velocity.z = msg->omega[2];
//     imu_measurement_ptr.get()->linear_acceleration.x = msg->acc[0];
//     imu_measurement_ptr.get()->linear_acceleration.y = msg->acc[1];
//     imu_measurement_ptr.get()->linear_acceleration.z = msg->acc[2];

//     if (debug_enabled_) {
//         // ROS_INFO("imu acc x: %0.4f y: %0.4f z: %0.4f\n", imu_msg.linear_acceleration.x,
//         //         imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
//     }
//     boost::mutex::scoped_lock lock(*cdata_mtx_);
//     cheetah_data_in_->imu_q.push(imu_measurement_ptr);
//   }

//   template <unsigned int ENCODER_DIM>
//   void lcm_handler<ENCODER_DIM>::joint_state_lcm_callback(const lcm::ReceiveBuffer* rbuf,
//                                            const std::string& channel_name,
//                                            const legcontrol_t* msg) {
//     ROS_DEBUG_STREAM("Receive new joint_state msg");    
//     seq_joint_state_++;

//     std::shared_ptr<JointStateMeasurement> joint_state_ptr = std::shared_ptr<JointStateMeasurement> (new JointStateMeasurement(ENCODER_DIM));
//     // JointStateMeasurement<double>* joint_state_ptr = new JointStateMeasurement<double>();
//     // std::vector<double> joint_position(ENCODER_DIM);
//     // std::copy(msg->q, msg->q+ENCODER_DIM, joint_position.begin());
//     // std::vector<double> joint_velocity(ENCODER_DIM);
//     // std::copy(msg->qd, msg->qd+ENCODER_DIM, joint_velocity.begin());
//     // std::vector<double> joint_effort(ENCODER_DIM);
//     // std::copy(msg->tau_est, msg->tau_est+ENCODER_DIM, joint_effort.begin());
//     // Eigen::Matrix<double, ENCODER_DIM, 1> encoder_pos;
//     // for (int j = 0; j < ENCODER_DIM; j++)
//     //     encoder_pos(j) = joint_position[j];

//     joint_state_ptr.get()->joint_position = Eigen::Map<const Eigen::MatrixXf>(msg->q,ENCODER_DIM,1).cast<double>();
//     joint_state_ptr.get()->joint_velocity = Eigen::Map<const Eigen::MatrixXf>(msg->qd,ENCODER_DIM,1).cast<double>();
//     joint_state_ptr.get()->joint_effort = Eigen::Map<const Eigen::MatrixXf>(msg->tau_est,ENCODER_DIM,1).cast<double>();

//     // if (debug_enabled_) {
//     //     sensor_msgs::JointState joint_state_msg;
//     //     joint_state_msg.header.seq = seq_joint_state_;
//     //     joint_state_msg.header.stamp = ros::Time::now();
//     //     joint_state_msg.position = joint_position;
//     //     joint_state_msg.velocity = joint_velocity;
//     //     joint_state_msg.effort = joint_effort;
//     //     joint_state_publisher_.publish(joint_state_msg);
//     //     joint_state_msg.header.frame_id = "/cheetah/joint_state";
//     // }

//     // std_msgs::Header header;
//     // joint_state_ptr->setKinematicsArray(
//     //     KinematicsHandler<ENCODER_DIM>::callback_handler(header, encoder_pos, cov_encoders_, cov_prior_)
//     // );

//     // joint_state_ptr->header.seq = seq_joint_state_;
//     // joint_state_ptr->header.stamp = ros::Time::now().toSec();
//     // joint_state_ptr->header.frame_id =  "/cheetah/joint_state";
//     // // kinematics_publisher_.publish(kinematics_arr);
//     // if (debug_enabled_) {
//     //     kinematics_debug_ << joint_state_ptr->getKinematicsArray() << '\n';
//     //     std::cout << joint_state_ptr->getKinematicsArray() << '\n';
//     // }

//     boost::mutex::scoped_lock lock(*cdata_mtx_);
//     cheetah_data_in_->joint_state_q.push(joint_state_ptr);
//   }

//   template <unsigned int ENCODER_DIM>
//   void lcm_handler<ENCODER_DIM>::contact_lcm_callback(const lcm::ReceiveBuffer* rbuf,
//                                        const std::string& channel_name,
//                                        const contact_t* msg) {
//     ROS_DEBUG_STREAM("Receive new contact msg");        
//     seq_contact_++;

//     std::shared_ptr<ContactsMeasurement> contact_ptr = std::shared_ptr<ContactsMeasurement>(new ContactsMeasurement);
//     // contact_ptr->header.seq = seq_contact_;
//     // contact_ptr->header.stamp = msg->timestamp;
//     // contact_ptr->header.frame_id = "/cheetah/contact";

//     // std::vector<inekf_msgs::Contact> contacts;
    
//     // for (int i = 0; i < 4; i++)
//     // {
//     //   inekf_msgs::Contact ct;
//     //   ct.id = i;
//     //   ct.indicator =  msg->contact[i] > 0;
//     //   contacts.push_back(ct);
//     // }


//     // Eigen::VectorXi contacts = Eigen::Map<const Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic>>(msg->contact, 4, 1).cast<int8_t>();

//     Eigen::Matrix<int8_t, 4, 1> contacts;
//     for (int i = 0; i < msg->num_legs; ++i) {
//         contacts[i] = msg->contact[i];
//     }

//     contact_ptr->setContacts(contacts);

//     if (debug_enabled_) {
//         // std::cout << "Contacts " << contact_ptr->getContacts() << '\n';
//     }
//     boost::mutex::scoped_lock lock(*cdata_mtx_);
//     cheetah_data_in_->contact_q.push(contact_ptr);
//   }



// } // mini_cheetah

template class cheetah_inekf_lcm::KinematicsHandler<12>;
template class cheetah_inekf_lcm::lcm_handler<12>;
