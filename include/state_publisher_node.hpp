#include "ros/ros.h"
#include <string>
#include <sstream> 
#include <fstream>
#include <array>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include "system/cheetah_state.hpp"
#include <Eigen/Dense>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>
#include "inekf_msgs/State.h"
class StatePublisherNode{
    public:
        StatePublisherNode(ros::NodeHandle* n);
        ~StatePublisherNode();

        // Publishes pose
        void statePublish(const CheetahState& state_);

    private:
        ros::NodeHandle* n_;
        ros::Publisher state_pub_;
        ros::Publisher velocity_pub_;
        std::string state_frame_;
        uint32_t seq_ = 0;
        double publish_rate_;
        std::mutex poses_mutex_;
        std::array<float,3> first_pose_;
        std::thread state_publishing_thread_;
};

