#include "system/cheetah_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>
#include <numeric>

CheetahSystem::CheetahSystem(lcm::LCM* lcm, ros::NodeHandle* nh, boost::mutex* cdata_mtx, cheetah_lcm_data_t* cheetah_buffer): 
    lcm_(lcm), nh_(nh), ts_(0.05, 0.05), cheetah_buffer_(cheetah_buffer), cdata_mtx_(cdata_mtx), estimator_(lcm), pose_publisher_node_(nh) {
        file_name_ = "/media/jetson256g/data/inekf_result/cheetah_inekf_pose_kitti.txt";
        tum_file_name_ = "/media/jetson256g/data/inekf_result/cheetah_inekf_pose_tum.txt";
        std::ofstream outfile(file_name_);
        std::ofstream tum_outfile(tum_file_name_);
        outfile.close();
        tum_outfile.close();
    }

void CheetahSystem::step() {
    //Copy data to be handled in queues (lock/unlock)
    // updateNextPacket();

    //Set state using encoder data todo: create state matrices
    //Use invariant-ekf-ros for reference on state matrices RobotState

    cdata_mtx_->lock();
    
    double timestamp = cheetah_buffer_->timestamp_q.front();
    cheetah_packet_.setTime(timestamp);
    cheetah_packet_.imu = cheetah_buffer_->imu_q.front();
    cheetah_packet_.joint_state = cheetah_buffer_->joint_state_q.front();
    cheetah_packet_.contact = cheetah_buffer_->contact_q.front();

    cheetah_buffer_->timestamp_q.pop();
    cheetah_buffer_->imu_q.pop();
    cheetah_buffer_->joint_state_q.pop();
    cheetah_buffer_->contact_q.pop();
    cdata_mtx_->unlock();

    state_.set(cheetah_packet_);

    if (estimator_.enabled()) {

        estimator_.setContacts(state_);
        estimator_.propagateIMU(cheetah_packet_, state_);
        // estimator_.correctKinematics(state_);
        pose_publisher_node_.posePublish(state_);
        poseCallback(state_);
    } else {
        std::cout << "Initialized initState" << std::endl;
        if (estimator_.biasInitialized()) {
            estimator_.initState(cheetah_packet_.getTime(), cheetah_packet_, state_);
            estimator_.enableFilter();
        } else {
            estimator_.initBias(cheetah_packet_);
        }
    }
}

void CheetahSystem::poseCallback(const CheetahState& state_) {
    if (file_name_.size() > 0) {
        // ROS_INFO_STREAM("write new pose\n");
        std::ofstream outfile(file_name_,std::ofstream::out | std::ofstream::app );
        outfile << "1 0 0 "<< state_.x() <<" 0 1 0 "<< state_.y() <<" 0 0 1 "<< state_.z() <<std::endl<<std::flush;
        outfile.close();
        // tum style
        std::ofstream tum_outfile(tum_file_name_,std::ofstream::out | std::ofstream::app );
        tum_outfile << cheetah_packet_.getTime() << " "<< state_.x()<<" "<< state_.y() << " "<<state_.z() << " "<<state_.getQuaternion().x()\
        <<" "<< state_.getQuaternion().y() <<" "<< state_.getQuaternion().z() <<" "<< state_.getQuaternion().w() <<std::endl<<std::flush;
        
        tum_outfile.close();
    }
}

// Private Functions

void CheetahSystem::updateNextPacket() {
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    // Defined as the length of measurement type enum
    std::vector<double> times(4, std::numeric_limits<double>::max());
    double timestamp = cheetah_buffer_->timestamp_q.front();
    cheetah_buffer_->timestamp_q.pop();

    cheetah_packet_.setTime(timestamp);
}
