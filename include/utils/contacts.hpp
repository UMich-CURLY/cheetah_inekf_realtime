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

            void setContacts(const std::vector<inekf_msgs::Contact>& contacts) {
                contacts_arr.contacts = contacts;
            }

            std::vector<inekf_msgs::Contact>& getContacts() {
                return contacts_arr.contacts;
            }

            const inekf_msgs::ContactArray& getContactArray() {
                return contacts_arr;
            }

        private:
            inekf_msgs::ContactArray contacts_arr;
    };
}