#pragma once
#include "utils/measurement.h"
#include <stdint.h>
#include <string>

namespace cheetah_inekf_lcm {
    class KinematicsMeasurement : public Measurement {
        public:
            // Construct KINEMATICS measurement
            KinematicsMeasurement() {
                type_ = KINEMATIC;
            }
        
            void setKinematicsArray(const inekf_msgs::KinematicsArray& kinematics) {
                kin_arr = kinematics;
            }

            const inekf_msgs::KinematicsArray& getKinematicsArray() {
                return kin_arr;
            }

        private:
            inekf_msgs::KinematicsArray kin_arr;
    };
}