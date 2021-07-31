#include "system/cheetah_system.hpp"
// Thread safe locking
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/circular_buffer.hpp>

CheetahSystem::CheetahSystem(boost::mutex* cdata_mtx, cheetah_lcm_data_t* cheetah_data): 
    ts_(0.05, 0.05), cdata_mtx_(cdata_mtx), cheetah_data_(cheetah_data) {}

void CheetahSystem::step() {
    //Copy data to be handled in queues (lock/unlock)

    //Set state using encoder data todo: create state matrices
    //Use invariant-ekf-ros for reference on state matrices RobotState

    if (estimator_!=nullptr) {
        if (/*estimator_->enabled()*/ 1) {
            //Update estimator 

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