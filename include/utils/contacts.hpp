#pragma once
#include "utils/measurement.h"
#include <stdint.h>
#include <string>

namespace cheetah_inekf_lcm {
    class ContactsMeasurement : public Measurement {
        public:
            // Construct CONTACT measurement
            ContactsMeasurement() {
                type_ = CONTACT;
            }


            // void setContacts(const std::vector<inekf_msgs::Contact>& contacts) {
            //     contacts_arr.contacts = contacts;
            // }

            // std::vector<inekf_msgs::Contact>& getContacts() {
            //     return contacts_arr.contacts;
            // }

            // // const inekf_msgs::Contact& getContact(int index) const {
            // //     return contacts_arr.contacts[index];
            // // }

            // const inekf_msgs::Contact &operator[](int index) const {
            //     return contacts_arr.contacts[index];
            // }

            // const inekf_msgs::ContactArray& getContactArray() {
            //     return contacts_arr;
            // }

            void setContacts(const Eigen::Matrix<int8_t, Eigen::Dynamic, 1> &contacts) {
                contacts_ = contacts;
            }
            
            Eigen::Matrix<int8_t, Eigen::Dynamic, 1> getContacts() {
                return contacts_;
            }

        private:
            // inekf_msgs::ContactArray contacts_arr;
            Eigen::Matrix<int8_t, Eigen::Dynamic, 1> contacts_;
    };
}