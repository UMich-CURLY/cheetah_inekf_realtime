#include "system/cheetah_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

CheetahSystem::CheetahSystem(boost::mutex* cdata_mtx, cheetah_lcm_data_t* cheetah_buffer): 
    ts_(0.05, 0.05), cdata_mtx_(cdata_mtx), cheetah_buffer_(cheetah_buffer) {}

void CheetahSystem::step() {
    //Copy data to be handled in queues (lock/unlock)
    updateCheetahPacket();

    //Set state using encoder data todo: create state matrices
    //Use invariant-ekf-ros for reference on state matrices RobotState

    if (estimator_!=nullptr) {
        if (estimator_->enabled()) {
            //Update estimator 
            estimator_->update(cheetah_packet_, state_);
            //Publish data over ROS & LCM
        } else {
            if (/*estimator_->biasInitialized()*/ 1) {
                //Initialize if a contact is made
                if (/*state_.getRightContact()==*/1) {
                   
                }
            } else {
                //Initialize InEKF bias estimate

            }
        }
    }
}

// Private Functions

void CheetahSystem::updateCheetahPacket() {
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    if (!cheetah_buffer_->imu_q.empty()) {
        cheetah_packet_.imu_q = *cheetah_buffer_->imu_q.front();
        delete cheetah_buffer_->imu_q.front();
        cheetah_buffer_->imu_q.pop_front();
    }
    if (!cheetah_buffer_->kin_q.empty()) {
        cheetah_packet_.kin_q = *cheetah_buffer_->kin_q.front();
        cheetah_buffer_->kin_q.pop_front();
    }
    if (!cheetah_buffer_->contact_q.empty()) {
        cheetah_packet_.contact_q = *cheetah_buffer_->contact_q.front();
        cheetah_buffer_->contact_q.pop_front();
    }

    // Take most recent time when updating t
    ros::Time new_time = cheetah_packet_.t;
    new_time = std::max(new_time, std::max(cheetah_packet_.imu_q.header.stamp,
                            std::max(cheetah_packet_.kin_q.header.stamp,
                                cheetah_packet_.contact_q.header.stamp)));
    cheetah_packet_.t = new_time;
}