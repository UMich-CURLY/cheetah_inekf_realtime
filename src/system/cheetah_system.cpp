#include "system/cheetah_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>
#include <numeric>

CheetahSystem::CheetahSystem(lcm::LCM* lcm, boost::mutex* cdata_mtx, cheetah_lcm_data_t* cheetah_buffer): 
    lcm_(lcm), ts_(0.05, 0.05), cheetah_buffer_(cheetah_buffer), cdata_mtx_(cdata_mtx), estimator_(lcm) {
        file_name_ = "/home/tingjun/Desktop/cheetah_inekf_pose_kitti.txt";
        tum_file_name_ = "/home/tingjun/Desktop/cheetah_inekf_pose_tum.txt";
        std::ofstream outfile(file_name_);
        std::ofstream tum_outfile(tum_file_name_);
        outfile.close();
        tum_outfile.close();
    }

void CheetahSystem::step() {
    //Copy data to be handled in queues (lock/unlock)
    updateNextPacket();

    //Set state using encoder data todo: create state matrices
    //Use invariant-ekf-ros for reference on state matrices RobotState

    if (estimator_.enabled()) {
        cdata_mtx_->lock();
        cheetah_packet_.imu = cheetah_buffer_->imu_q.front();
        cheetah_packet_.joint_state = cheetah_buffer_->joint_state_q.front();
        cheetah_packet_.contact = cheetah_buffer_->contact_q.front();

        cheetah_buffer_->imu_q.pop();
        cheetah_buffer_->joint_state_q.pop();
        cheetah_buffer_->contact_q.pop();
        cdata_mtx_->unlock();

        state_.set(cheetah_packet_);
        estimator_.setContacts(state_);
        estimator_.propagateIMU(cheetah_packet_, state_);
        estimator_.correctKinematics(state_);

        poseCallback(state_);
    }
}

void CheetahSystem::poseCallback(const CheetahState& state_) {
    // if ((int)msg->header.seq%pose_skip_!=0) { return; }
    // geometry_msgs::PoseStamped pose;
    // pose.header = msg->header;
    // pose.pose = msg->pose.pose;
    //std::cout<<"publishing: "<<pose.pose.position.x<<", "<<pose.pose.position.y<<", "<<pose.pose.position.z<<std::endl;
    
    // std::lock_guard<std::mutex> lock(cdata_mtx_);

    if (file_name_.size() > 0) {
        // ROS_INFO_STREAM("write new pose\n");
        std::ofstream outfile(file_name_,std::ofstream::out | std::ofstream::app );
        outfile << "1 0 0 "<< state_.x() <<" 0 1 0 "<< state_.y() <<" 0 0 1 "<< state_.z() <<std::endl<<std::flush;
        outfile.close();
        // tum style
        std::ofstream tum_outfile(tum_file_name_,std::ofstream::out | std::ofstream::app );
        tum_outfile << cheetah_packet_.getTime() << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().x() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;
        
        tum_outfile.close();
    }
    
    // poses_.push_back(pose);
}
// Private Functions

void CheetahSystem::updateNextPacket() {
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    // Defined as the length of measurement type enum
    std::vector<double> times(4, std::numeric_limits<double>::max());
    double timestamp = cheetah_buffer_->timestamp_q.front();
    cheetah_buffer_->timestamp_q.pop();

    // int index = 0;
    // if (!cheetah_buffer_->imu_q.empty()) {
    //     index = static_cast<int>(IMU);
    //     // std::cout << "IMU GET " << index << "\n";
    //     times[index] = cheetah_buffer_->imu_q.front()->getTime();
    // }
    // if (!cheetah_buffer_->joint_state_q.empty()) {
    //     index = static_cast<int>(JOINT_STATE);
    //     // std::cout << "KIN GET " << index << "\n";
    //     times[index] = cheetah_buffer_->joint_state_q.front()->getTime();
    // }
    // if (!cheetah_buffer_->contact_q.empty()) {
    //     index = static_cast<int>(CONTACT);
    //     // std::cout << "CONTACT GET " << index << "\n";
    //     times[index] = cheetah_buffer_->contact_q.front()->getTime();
    // }

    // // Take most recent time when updating t
    // double min_time = times[EMPTY];
    // MeasurementType min_index = EMPTY;
    // for (int i = 0; i < 4; ++i) {
    //     if (times[i] < min_time) {
    //         min_time = times[i];
    //         min_index = static_cast<MeasurementType>(i);
    //     }
    // }

    // // Updates next type and time
    // cheetah_packet_.setType(min_index);

    cheetah_packet_.setTime(timestamp);

    // Enable filter once first imu measurement is received
    // if (!estimator_.enabled() && cheetah_packet_.getType()==IMU) {
    estimator_.enableFilter();
    // }
}