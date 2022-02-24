#include "state_publisher_node.hpp"

StatePublisherNode::StatePublisherNode(ros::NodeHandle* n) : n_(n) {
    // Create private node handle
    ros::NodeHandle nh("~");
    // std::string pose_csv_file, init_rot_file;
    std::string state_topic, state_frame,velocity_topic;

    nh.param<std::string>("state_topic", state_topic, "/cheetah/inekf_estimation/inekf_state");
    nh.param<std::string>("velocity_topic", velocity_topic, "/cheetah/inekf_estimation/inekf_velocity");
    nh.param<std::string>("state_frame", state_frame, "odom");

    nh.param<bool>("/settings/system_enable_time_match",enable_time_match_,false);
    std::cout <<"time match:"  << enable_time_match_ << std::endl;
    nh.param<double>("publish_rate", publish_rate_, 1000); 
    first_pose_ = {0, 0, 0};
    // first_pose_ = pose_from_csv_.front();
    // std::cout<<"first pose is: "<<first_pose_[0]<<", "<<first_pose_[1]<<", "<<first_pose_[2]<<std::endl;
    state_frame_ = state_frame;
    state_pub_ = n_->advertise<inekf_msgs::State>(state_topic, 1000);
    velocity_pub_ = n_->advertise<geometry_msgs::TwistStamped>(velocity_topic, 1000);
    //pose_pub_ = n_->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 1000);
    // this->pose_publishing_thread_ = std::thread([this]{this->posePublishingThread();});
}

StatePublisherNode::~StatePublisherNode() {}

// Publishes pose
void StatePublisherNode::statePublish(const CheetahState& state_) {
    // inekf message header
    inekf_msgs::State state_msgs;
    state_msgs.header.seq = seq_;

    // ros::Time().fromSec(timestamp)
    //double timestamp = state.
    if (enable_time_match_ == true){
        state_msgs.header.stamp = state_.getRosTime();
        //std::cout <<"time match:"  << enable_time_match_ << std::endl;

    }else{
        state_msgs.header.stamp = ros::Time::now();
                std::cout <<"time match:"  << enable_time_match_ << std::endl;

    }
    state_msgs.header.frame_id = state_frame_;

    // get orientation
    state_msgs.orientation.w = state_.getQuaternion().w();
    state_msgs.orientation.x = state_.getQuaternion().x();
    state_msgs.orientation.y = state_.getQuaternion().y();
    state_msgs.orientation.z = state_.getQuaternion().z();

    //get position
    state_msgs.position.x = state_.x() - first_pose_[0];
    state_msgs.position.y = state_.y() - first_pose_[1];
    state_msgs.position.z = state_.z() - first_pose_[2];

    //get velocity
    state_msgs.velocity.x = state_.dx();
    state_msgs.velocity.y = state_.dy();
    state_msgs.velocity.z = state_.dz();


   

    // covirance
    Eigen::MatrixXd P = state_.P();
    
    for (int i=0; i<9; ++i) {
        for (int j=0; j<9; ++j) {
            state_msgs.covariance[9*i+j] = P(i,j);
        }
    }

    //publish state
    state_pub_.publish(state_msgs);
    seq_++;

    geometry_msgs::TwistStamped twist_msgs;
    twist_msgs.header.stamp = ros::Time::now();
    twist_msgs.header.frame_id = state_frame_;
    twist_msgs.twist.linear.x = state_.dx();
    twist_msgs.twist.linear.y = state_.dy();
    twist_msgs.twist.linear.z = state_.dz();
    twist_msgs.twist.angular.x = state_.droll();
    twist_msgs.twist.angular.y = state_.dpitch();
    twist_msgs.twist.angular.z = state_.dyaw();
    velocity_pub_.publish(twist_msgs);
    //for contact 
    // inekf_msgs::VectorWithId contact_leftFront;
    // contact_rightFront.id = 0
    // contact_rightFront
    // for (auto it=estimated_contacts.begin(); it!=estimated_contacts.end(); ++it) {
    //     inekf_msgs::VectorWithId contact;
    //     contact.id = it->first;
    //     contact.position.x = X(0,it->second);
    //     contact.position.y = X(1,it->second);
    //     contact.position.z = X(2,it->second);
    //     state_msg.contacts.push_back(contact);
    // }
    // state_msg.gyroscope_bias.x = bg(0); 
    // state_msg.gyroscope_bias.y = bg(1); 
    // state_msg.gyroscope_bias.z = bg(2); 
    // state_msg.accelerometer_bias.x = ba(0); 
    // state_msg.accelerometer_bias.y = ba(1); 
    // state_msg.accelerometer_bias.z = ba(2); 
    // state_pub_.publish(state_msg);
    // state_pub_.publish(pose_msg);
    // seq_++;

}


// Pose message callback
// void PosePublisherNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
//     if ((int)msg->header.seq%pose_skip_!=0) { return; }
//     geometry_msgs::PoseStamped pose;
//     pose.header = msg->header;
//     pose.pose = msg->pose.pose;
//     std::lock_guard<std::mutex> lock(poses_mutex_);
//     poses_.push_back(pose);
// }


// Path publishing thread
// void PosePublisherNode::posePublishingThread(){
//     // Loop and publish data
//     ros::Rate loop_rate(publish_rate_);
//     while(ros::ok()){
//         posePublish();
//         loop_rate.sleep();
//     }
// }

// int main(int argc, char **argv) {
//     // Initialize ROS node
//     ros::init(argc, argv, "pose_publisher");  
//     ros::NodeHandle n;
//     PosePublisherNode pose_publisher_node(n);
//     ros::spin();
//     return 0;
// }
