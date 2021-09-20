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
    Eigen::Matrix<bool, 4, 1> contacts;
    for (int i = 0; i < msg->num_legs; ++i) {
        std::cout << msg->contact[i];
        contacts[i] = msg->contact[i];
    }
    std::cout << std::endl;
    // std::cout << "Corresponding contacts: " << contacts[0] << contacts[1] << contacts[2] << contacts[3] << std::endl; 
    
    contact_ptr->setContacts(contacts);

    // push into queues:
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    cheetah_buffer_->timestamp_q.push(timestamp);
    cheetah_buffer_->imu_q.push(imu_measurement_ptr);
    cheetah_buffer_->joint_state_q.push(joint_state_ptr);
    cheetah_buffer_->contact_q.push(contact_ptr);
  }

} // mini_cheetah

// template class cheetah_inekf_lcm::KinematicsHandler<12>;
template class cheetah_inekf_lcm::lcm_handler<12>;
