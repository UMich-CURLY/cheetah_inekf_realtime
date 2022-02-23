#include "system/cheetah_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>
#include <numeric>

CheetahSystem::CheetahSystem(lcm::LCM* lcm, ros::NodeHandle* nh, boost::mutex* cdata_mtx, cheetah_lcm_data_t* cheetah_buffer): 
    lcm_(lcm), nh_(nh), ts_(0.05, 0.05), cheetah_buffer_(cheetah_buffer), cdata_mtx_(cdata_mtx), estimator_(lcm), pose_publisher_node_(nh),state_publisher_node_(nh) {
    // Initialize inekf pose file printouts
    nh_->param<std::string>("/settings/system_inekf_pose_filename", file_name_, 
        "/media/jetson256g/data/inekf_result/cheetah_inekf_pose.txt");
    nh_->param<std::string>("/settings/system_inekf_tum_pose_filename", tum_file_name_, 
        "/media/jetson256g/data/inekf_result/cheetah_inekf_tum_pose.txt");
    std::ofstream outfile(file_name_);
    std::ofstream tum_outfile(tum_file_name_);
    outfile.close();
    tum_outfile.close();

    // Initialize pose publishing if requested
    nh_->param<bool>("/settings/system_enable_pose_publisher", enable_pose_publisher_, false);
    nh_->param<bool>("/settings/system_enable_state_publisher", enable_state_publisher_, false);
    nh_->param<bool>("/settings/system_enable_time_match",enable_time_match_,false);

    //set up ros subscriber 
    // TODO: change subscribe topic to a parameter
    if (enable_time_match_){
        rosbag_subscriber_ = nh->subscribe("/Imu", 1000,  &CheetahSystem::timesyncCallback, this);
    }
    matched_ = false;
    updated_ = false;
    buffered_ = false;
    timestamp_ = ros::Time(1630968850.898508728 - 1.08527);
    std::cout<<timestamp_ << std::endl;
}

void CheetahSystem::step() {
    bool hasUpdate = updateNextPacket();

    if (hasUpdate) {
        updated_ = true;
        state_.set(cheetah_packet_);

        if (estimator_.enabled()) {
            estimator_.setContacts(state_);

            // estimator.update propagate and correct (if contact exists) the filter
            estimator_.update(cheetah_packet_, state_);
            state_.setBasetime(timestamp_);
            if (enable_pose_publisher_) {
                pose_publisher_node_.posePublish(state_);
                poseCallback(state_);
            }
            if (enable_state_publisher_) {
                state_publisher_node_.statePublish(state_);
            }
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

bool CheetahSystem::updateNextPacket() {
    //Copy data to be handled in queues (lock/unlock)
    bool hasUpdated = false;
    cdata_mtx_->lock();
    if (!cheetah_buffer_->timestamp_q.empty() &&
        !cheetah_buffer_->imu_q.empty() &&
        !cheetah_buffer_->joint_state_q.empty() &&
        !cheetah_buffer_->contact_q.empty()) 
    {
        hasUpdated = true;
        double timestamp = cheetah_buffer_->timestamp_q.front();
        cheetah_packet_.setTime(timestamp);
        cheetah_packet_.imu = cheetah_buffer_->imu_q.front();
        cheetah_packet_.joint_state = cheetah_buffer_->joint_state_q.front();
        cheetah_packet_.contact = cheetah_buffer_->contact_q.front();

        cheetah_buffer_->timestamp_q.pop();
        cheetah_buffer_->imu_q.pop();
        cheetah_buffer_->joint_state_q.pop();
        cheetah_buffer_->contact_q.pop();
    }
    cdata_mtx_->unlock();

    return hasUpdated;
}

//time sync callback
void CheetahSystem::timesyncCallback(const sensor_msgs::Imu::ConstPtr& imu_message){
    if (updated_ == true && buffered_ == false){
        imu_buffer_.header.stamp = imu_message->header.stamp;
        imu_buffer_.angular_velocity = imu_message->angular_velocity;
        std::cout <<"lcm_time" << state_.getTime()<< std::endl;
        std::cout << "ros_time"  << imu_buffer_.header.stamp << std::endl;
        buffered_ = true;
    }
    const double epsilon = 0.000001;
    if (matched_ == false && updated_ == true){
        bool match = true;
        if (fabs(cheetah_packet_.imu->angular_velocity.x- imu_buffer_.angular_velocity.x) > epsilon){
            match = false;
        }else if (fabs(cheetah_packet_.imu->angular_velocity.y - imu_buffer_.angular_velocity.y) > epsilon){
            match = false;
        }else if (fabs(cheetah_packet_.imu->angular_velocity.z - imu_buffer_.angular_velocity.z) > epsilon){
            match = false;
        }
        if (match == true){
            std::cout <<"find matching time - >> " << state_.getTime()<< std::endl;
            std::cout << "rostime matching - >> " << imu_buffer_.header.stamp << std::endl;
        }
        timestamp_ = imu_buffer_.header.stamp;
       
    }

}