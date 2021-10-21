#include "communication/lcm_handler.hpp"


namespace cheetah_inekf_lcm
{
    lcm_handler::lcm_handler(lcm::LCM* lcm, ros::NodeHandle* n, cheetah_lcm_data_t* cheetah_buffer, boost::mutex* cdata_mtx) : 
        lcm_(lcm), nh_(n), cheetah_buffer_(cheetah_buffer), cdata_mtx_(cdata_mtx) {
        assert(lcm_);  // confirm a nullptr wasn't passed in
        ROS_INFO("Cheetah_Lcm ready to initialize...."); 

        /// NOSYNC:
        std::string mode;
        std::string lcm_leg_channel;
        std::string lcm_imu_channel;
        std::string contact_ground_truth;
        bool run_synced;

        nh_->param<std::string>("settings/mode", mode, "normal");
        nh_->param<std::string>("settings/lcm_leg_channel", lcm_leg_channel, "leg_control_data");
        nh_->param<std::string>("settings/lcm_imu_channel", lcm_imu_channel, "microstrain");
        nh_->param<std::string>("settings/contact_ground_truth", contact_ground_truth, "wbc_lcm_data");
        nh_->param<bool>("settings/run_synced", run_synced, false);


        if (!run_synced && mode == "normal")
        {
            lcm_->subscribe(lcm_leg_channel, &cheetah_inekf_lcm::lcm_handler::receiveLegControlMsg, this);
            lcm_->subscribe(lcm_imu_channel, &cheetah_inekf_lcm::lcm_handler::receiveMicrostrainMsg, this);
            lcm_->subscribe(contact_ground_truth, &cheetah_inekf_lcm::lcm_handler::receiveContactGroundTruthMsg, this);
        }

        start_time_ = 0;

        nh_->param<int>("settings/leg_q_dimension", q_dim, 12);
        nh_->param<int>("settings/leg_qd_dimension", qd_dim, 12);
        nh_->param<int>("settings/leg_tau_dimension", tau_est_dim, 12);
        nh_->param<int>("settings/imu_acc_dimension", acc_dim, 3);
        nh_->param<int>("settings/imu_omega_dimension", omega_dim, 3);
        nh_->param<int>("settings/imu_rpy_dimension", rpy_dim, 3);
        nh_->param<int>("settings/imu_quat_dimension", quat_dim, 4);
        

        std::cout << "Subscribed to channels" << std::endl;

        /// SYNCED:
        if (run_synced)
            lcm_->subscribe("synced_proprioceptive_data", &cheetah_inekf_lcm::lcm_handler::synced_msgs_lcm_callback, this);


	    seq_imu_data_ = 0;
        seq_joint_state_ = 0;
        seq_contact_ = 0;

        //Set private variables
        double encoder_std, kinematic_prior_orientation_std, kinematic_prior_position_std;
        std::string project_root_dir;
        nh_->param<double>("/inekf/encoder_std", encoder_std, 0.0174533); // 1 deg std
        nh_->param<double>("/inekf/kinematic_prior_orientation_std", kinematic_prior_orientation_std, 0.174533); // 10 deg std
        nh_->param<double>("/inekf/kinematic_prior_position_std", kinematic_prior_position_std, 0.05); // 5cm std
        nh_->param<bool>("/settings/lcm_enable_debug_output", debug_enabled_, false);
        nh_->param<std::string>("/settings/project_root_dir", project_root_dir, "../../../");

        // Debugging ROS messages
        // imu_publisher_ = nh_->advertise<sensor_msgs::Imu>("imu", 10);
        // joint_state_publisher_ = nh_->advertise<sensor_msgs::JointState>("joint_state", 10);
        // kinematics_publisher_ = nh_->advertise<inekf_msgs::KinematicsArray>("kinematics", 10);
        // contact_publisher_ = nh_->advertise<inekf_msgs::ContactArray>("contact", 10);

        cov_encoders_ = encoder_std*encoder_std*Eigen::Matrix<double,12,12>::Identity(); 
        cov_prior_ = Eigen::Matrix<double,6,6>::Identity();
        cov_prior_.block<3,3>(0,0) = kinematic_prior_orientation_std*kinematic_prior_orientation_std*Eigen::Matrix<double,3,3>::Identity();
        cov_prior_.block<3,3>(3,3) = kinematic_prior_position_std*kinematic_prior_position_std*Eigen::Matrix<double,3,3>::Identity();

        ROS_INFO("Cheetah_Lcm initialized."); 
        if (debug_enabled_) {
            std::cout << project_root_dir << "/tests/kinematics/lcmlog.out" << '\n';
            kinematics_debug_.open(project_root_dir + "/tests/kinematics/lcmlog.out");
            assert(kinematics_debug_.is_open());
        }

    }

    lcm_handler::~lcm_handler() {}

    void lcm_handler::receiveLegControlMsg(const lcm::ReceiveBuffer *rbuf,
                                          const std::string &chan,
                                          const leg_control_data_lcmt *msg)
    {   
        // std::cout << "Received leg data" << std::endl;
        if (start_time_ == 0)
        {
            start_time_ = rbuf->recv_utime;
        }
        if (cheetah_buffer_->joint_state_q.size() <= cheetah_buffer_->contact_q.size())
        {
            std::shared_ptr<JointStateMeasurement> joint_state_ptr = std::shared_ptr<JointStateMeasurement>(new JointStateMeasurement(q_dim));

            /// LEG:
            joint_state_ptr.get()->joint_position = Eigen::Map<const Eigen::MatrixXf>(msg->q, q_dim, 1).cast<double>();
            joint_state_ptr.get()->joint_velocity = Eigen::Map<const Eigen::MatrixXf>(msg->qd, qd_dim, 1).cast<double>();
            joint_state_ptr.get()->joint_effort = Eigen::Map<const Eigen::MatrixXf>(msg->tau_est, tau_est_dim, 1).cast<double>();

            /// LOW: 500Hz version:
            boost::mutex::scoped_lock lock(*cdata_mtx_);
            cheetah_buffer_->joint_state_q.push(joint_state_ptr);

        }
    }

    void lcm_handler::receiveMicrostrainMsg(const lcm::ReceiveBuffer *rbuf,
                                           const std::string &chan,
                                           const microstrain_lcmt *msg)
    {
        /// LOW: 500Hz version:
        // std::cout << "Received IMU data" << std::endl;
        if (start_time_ == 0)
        {
            start_time_ = rbuf->recv_utime;
        }
        if (cheetah_buffer_->joint_state_q.size() > cheetah_buffer_->imu_q.size())
        {
            // std::shared_ptr<LcmIMUStruct> microstrain_data = std::make_shared<LcmIMUStruct>();
            std::shared_ptr<ImuMeasurement<double>> imu_measurement_ptr = std::shared_ptr<ImuMeasurement<double>>(new ImuMeasurement<double>);

            imu_measurement_ptr.get()->orientation.w = msg->quat[0];
            imu_measurement_ptr.get()->orientation.x = msg->quat[1];
            imu_measurement_ptr.get()->orientation.y = msg->quat[2];
            imu_measurement_ptr.get()->orientation.z = msg->quat[3];
            imu_measurement_ptr.get()->angular_velocity.x = msg->omega[0];
            imu_measurement_ptr.get()->angular_velocity.y = msg->omega[1];
            imu_measurement_ptr.get()->angular_velocity.z = msg->omega[2];
            imu_measurement_ptr.get()->linear_acceleration.x = msg->acc[0];
            imu_measurement_ptr.get()->linear_acceleration.y = msg->acc[1];
            imu_measurement_ptr.get()->linear_acceleration.z = msg->acc[2];

            double timestamp = (1.0 * (rbuf->recv_utime - start_time_)) / pow(10, 6);
            
            boost::mutex::scoped_lock lock(*cdata_mtx_);
            cheetah_buffer_->timestamp_q.push(timestamp);
            cheetah_buffer_->imu_q.push(imu_measurement_ptr);
        }

    }

    void lcm_handler::receiveContactGroundTruthMsg(const lcm::ReceiveBuffer *rbuf,
                                                const std::string &chan,
                                                const wbc_test_data_t *msg)
    {
        // std::cout << "Received contact data" << std::endl;
        std::shared_ptr<ContactsMeasurement> contact_ptr = std::shared_ptr<ContactsMeasurement>(new ContactsMeasurement);
        
        /// CONTACTS:
        Eigen::Matrix<bool, 4, 1> contacts;
        for (int i = 0; i < 4; ++i)
        {
            // std::cout << msg->contact_est[i];
            contacts[i] = msg->contact_est[i];
        }
        // std::cout << std::endl;
        contact_ptr->setContacts(contacts);
        boost::mutex::scoped_lock lock(*cdata_mtx_);
        cheetah_buffer_->contact_q.push(contact_ptr);
    }

    void lcm_handler::synced_msgs_lcm_callback(const lcm::ReceiveBuffer *rbuf,
                                                            const std::string &channel_name,
                                                            const synced_proprioceptive_lcmt *msg)
    {
        ROS_DEBUG_STREAM("Receive new synchronized msg");

        seq_joint_state_++;
        std::shared_ptr<ImuMeasurement<double>> imu_measurement_ptr = std::shared_ptr<ImuMeasurement<double>>(new ImuMeasurement<double>);
        std::shared_ptr<JointStateMeasurement> joint_state_ptr = std::shared_ptr<JointStateMeasurement>(new JointStateMeasurement(q_dim));
        std::shared_ptr<ContactsMeasurement> contact_ptr = std::shared_ptr<ContactsMeasurement>(new ContactsMeasurement);

        /// TIMESTAMP:
        double timestamp = msg->timestamp;

        /// IMU:
        imu_measurement_ptr.get()->orientation.w = msg->quat[0];
        imu_measurement_ptr.get()->orientation.x = msg->quat[1];
        imu_measurement_ptr.get()->orientation.y = msg->quat[2];
        imu_measurement_ptr.get()->orientation.z = msg->quat[3];
        imu_measurement_ptr.get()->angular_velocity.x = msg->omega[0];
        imu_measurement_ptr.get()->angular_velocity.y = msg->omega[1];
        imu_measurement_ptr.get()->angular_velocity.z = msg->omega[2];
        imu_measurement_ptr.get()->linear_acceleration.x = msg->acc[0];
        imu_measurement_ptr.get()->linear_acceleration.y = msg->acc[1];
        imu_measurement_ptr.get()->linear_acceleration.z = msg->acc[2];

        /// LEG:
        joint_state_ptr.get()->joint_position = Eigen::Map<const Eigen::MatrixXf>(msg->q, q_dim, 1).cast<double>();
        joint_state_ptr.get()->joint_velocity = Eigen::Map<const Eigen::MatrixXf>(msg->qd, qd_dim, 1).cast<double>();
        joint_state_ptr.get()->joint_effort = Eigen::Map<const Eigen::MatrixXf>(msg->tau_est, tau_est_dim, 1).cast<double>();

        /// CONTACTS:
        Eigen::Matrix<bool, 4, 1> contacts;
        std::cout << 1 << std::endl;
        for (int i = 0; i < msg->num_legs; ++i)
        {
            std::cout << msg->contact[i];
            contacts[i] = msg->contact[i];
        }
        std::cout << std::endl;
        // std::cout << "Corresponding contacts: " << contacts[0] << contacts[1] << contacts[2] << contacts[3] << std::endl;

        contact_ptr->setContacts(contacts);

        // push into queues:
        boost::mutex::scoped_lock lock(*cdata_mtx_);
        cheetah_buffer_->timestamp_q.push(timestamp);
        cheetah_buffer_->imu_q.push(imu_measurement_ptr);
        cheetah_buffer_->joint_state_q.push(joint_state_ptr);
        cheetah_buffer_->contact_q.push(contact_ptr);
    }

} // mini_cheetah

// template class cheetah_inekf_lcm::KinematicsHandler<12>;
// template class cheetah_inekf_lcm::lcm_handler;
