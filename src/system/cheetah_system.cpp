#include "system/cheetah_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

#include <numeric>

CheetahSystem::CheetahSystem(boost::mutex* cdata_mtx, cheetah_lcm_data_t* cheetah_buffer): 
    ts_(0.05, 0.05), cdata_mtx_(cdata_mtx), cheetah_buffer_(cheetah_buffer) {}

void CheetahSystem::step() {
    //Copy data to be handled in queues (lock/unlock)
    updateNextPacket();

    //Set state using encoder data todo: create state matrices
    //Use invariant-ekf-ros for reference on state matrices RobotState

    if (estimator_.enabled()) {
        switch (cheetah_packet_.getType()) {
            case EMPTY: {
                break;
            }
            case IMU: {
                cdata_mtx_->lock();
                cheetah_packet_.imu_q = *cheetah_buffer_->imu_q.front();
                delete cheetah_buffer_->imu_q.front();
                cheetah_buffer_->imu_q.pop_front();
                cdata_mtx_->unlock();
                // Updated InEKF and initializes bias from first imu measurement
                estimator_.propagateIMU(cheetah_packet_, state_);
                break;
            }
            case KINEMATIC: {
                if (estimator_.biasInitialized()) {
                    cdata_mtx_->lock();
                    cheetah_packet_.kin_q = *cheetah_buffer_->kin_q.front();
                    delete cheetah_buffer_->kin_q.front();
                    cheetah_buffer_->kin_q.pop_front();
                    cdata_mtx_->unlock();
                    ///TODO: Add function for updating state with kinematics
                    
                    estimator_.correctKinematics(state_);
                }
                break;
            } 
            case CONTACT: {
                cdata_mtx_->lock();
                cheetah_packet_.contact_q = *cheetah_buffer_->contact_q.front();
                delete cheetah_buffer_->contact_q.front();
                cheetah_buffer_->contact_q.pop_front();
                cdata_mtx_->unlock();
                ///TODO: Add function for updating state with contacts

                estimator_.setContacts(state_);
                break;
            }
            default: {
                break;
            }
        }
    }
}

// Private Functions

void CheetahSystem::updateNextPacket() {
    boost::mutex::scoped_lock lock(*cdata_mtx_);
    // Defined as the length of measurement type enum
    double times[4] = { std::numeric_limits<double>::max() };
    if (!cheetah_buffer_->imu_q.empty()) {
        times[IMU] = cheetah_buffer_->imu_q.front()->getTime();
    }
    if (!cheetah_buffer_->kin_q.empty()) {
        times[KINEMATIC] = cheetah_buffer_->kin_q.front()->getTime();
    }
    if (!cheetah_buffer_->contact_q.empty()) {
        times[CONTACT] = cheetah_buffer_->contact_q.front()->getTime();
    }

    // Take most recent time when updating t
    double min_time = times[EMPTY];
    MeasurementType min_index = EMPTY;
    for (int i = 0; i < 4; ++i) {
        if (times[i] < min_time) {
            min_time = times[i];
            min_index = static_cast<MeasurementType>(i);
        }
    }

    // Updates next type and time
    cheetah_packet_.setType(min_index);
    cheetah_packet_.setTime(min_time);

    // Enable filter once first imu measurement is received
    if (!estimator_.enabled() && cheetah_packet_.getType()==IMU) {
        estimator_.enableFilter();
    }
}