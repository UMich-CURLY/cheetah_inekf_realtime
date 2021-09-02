#include "system/cheetah_state.hpp"

// Default Constructor
CheetahState::CheetahState() {
    this->clear();
}

// Constructor from cassie_out_t
CheetahState::CheetahState(const cheetah_lcm_packet_t& cheetah_data) {
    this->set(cheetah_data);
}

// Set q and dq from cheetah_lcm_data_t
void CheetahState::set(const cheetah_lcm_packet_t& cheetah_data) { 

    const std::shared_ptr<cheetah_inekf_lcm::ImuMeasurement<double>> imu_data = cheetah_data.imu;
    
    // Set orientation
    Eigen::Quaternion<double> quat(imu_data.get()->orientation.w, 
                            imu_data.get()->orientation.x,
                            imu_data.get()->orientation.y,
                            imu_data.get()->orientation.z); 
    Eigen::Vector3d euler = Rotation2Euler(quat.toRotationMatrix()); // Eigen's eulerAngles function caused discontinuities in signal  
    q_.block<3,1>(3,0) = euler;

    // Set orientation rates
    Eigen::Vector3d angularVelocity, eulerRates;
    angularVelocity << imu_data.get()->angular_velocity.x, imu_data.get()->angular_velocity.y, imu_data.get()->angular_velocity.z;
    eulerRates = AngularVelocity2EulerRates(euler, angularVelocity);
    dq_.block<3,1>(3,0) = eulerRates;

    const std::shared_ptr<cheetah_inekf_lcm::JointStateMeasurement> joint_state_data = cheetah_data.joint_state;
    // Set encoders position & rates:
    q_.block<12,1>(6,0) = joint_state_data->joint_position;
    dq_.block<12,1>(6,0) = joint_state_data->joint_velocity;


    const std::shared_ptr<cheetah_inekf_lcm::ContactsMeasurement> contact_data = cheetah_data.contact;
    right_front_contact_ = contact_data.get()->getContacts()[0];
    left_front_contact_  = contact_data.get()->getContacts()[1];
    right_hind_contact_  = contact_data.get()->getContacts()[2];
    left_hind_contact_   = contact_data.get()->getContacts()[3];
    return;
}



//     /// REMARK: to use the following code, delete the return above
//     const cheetah_inekf_lcm::ImuMeasurement<double>* imu_data = cheetah_data.imu.get();
    
//     const cheetah_inekf_lcm::JointStateMeasurement* joint_state_data = cheetah_data.joint_state.get();

//     const cheetah_inekf_lcm::ContactsMeasurement* contact_data = cheetah_data.contact.get();


//     // ---------- Set Positions ------------//
//     // Set orientation
//     Eigen::Quaternion<double> quat(imu_data->orientation.w, 
//                                    imu_data->orientation.x,
//                                    imu_data->orientation.y,
//                                    imu_data->orientation.z); 
//     Eigen::Vector3d euler = Rotation2Euler(quat.toRotationMatrix()); // Eigen's eulerAngles function caused discontinuities in signal
//     // q_(3) = euler(0);
//     // q_(4) = euler(1);
//     // q_(5) = euler(2);
//     q_.block<3,1>(3,0) = euler;
//     q_.block<12,1>(6,0) = joint_state_data->joint_position;
//     // // Set encoders right front leg
//     // q_(6) = joint_state_data->joint_position[0]; 
//     // q_(7) = joint_state_data->joint_position[1];
//     // q_(8) = joint_state_data->joint_position[2];

//     // // Set encoders left front leg:
//     // q_(9) = joint_state_data->joint_position[3];
//     // q_(10) = joint_state_data->joint_position[4];
//     // q_(11) = joint_state_data->joint_position[5];

//     // // Set encoders right hind leg:
//     // q_(12) = joint_state_data->joint_position[6];
//     // q_(13) = joint_state_data->joint_position[7];
//     // q_(14) = joint_state_data->joint_position[8];

//     // // Set encoders left hind leg
//     // q_(15) = joint_state_data->joint_position[9];
//     // q_(16) = joint_state_data->joint_position[10];
//     // q_(17) = joint_state_data->joint_position[11];

//     // ---------- Set Velocities ------------//
//     // Set orientation rates
//     Eigen::Vector3d angularVelocity, eulerRates;
//     angularVelocity << imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z;
//     eulerRates = AngularVelocity2EulerRates(euler, angularVelocity);
//     // dq_(3) = eulerRates(0);
//     // dq_(4) = eulerRates(1);
//     // dq_(5) = eulerRates(2);

//     dq_.block<3,1>(3,0) = eulerRates;
//     dq_.block<12,1>(6,0) = joint_state_data->joint_velocity;

//     // // Set encoder velocities right front leg
//     // dq_(6) = joint_state_data->joint_velocity[0];
//     // dq_(7) = joint_state_data->joint_velocity[1];
//     // dq_(8) = joint_state_data->joint_velocity[2];

//     // // Set encoder velocities left front leg
//     // dq_(9) = joint_state_data->joint_velocity[3];
//     // dq_(10) = joint_state_data->joint_velocity[4];
//     // dq_(11) = joint_state_data->joint_velocity[5];

//     // // Set encoder velocities right hind leg
//     // dq_(12) = joint_state_data->joint_velocity[6];
//     // dq_(13) = joint_state_data->joint_velocity[7];
//     // dq_(14) = joint_state_data->joint_velocity[8];

//     // // Set encoder velocities left hind leg
//     // dq_(15) = joint_state_data->joint_velocity[9];
//     // dq_(16) = joint_state_data->joint_velocity[10];
//     // dq_(17) = joint_state_data->joint_velocity[11];

//     // Set base velocity using kinematic estimate DO NOT NEED
//     // Eigen::Vector3d i_v_wi = this->getKinematicVelocity();
//     // Eigen::Vector3d i_p_ib; i_p_ib <<-0.0316; 0; 0.08;
//     // Eigen::Vector3d v = this->getRotation() * (i_v_wi + skew(angularVelocity)*i_p_ib);


//     // Compute ground reaction force and contact estimates if indicated
//     /// REMARK: This will be given by lcm_cnn_interface
//     // right_front_contact_    = (*contact_data)[0].indicator;
//     // left_front_contact_     = (*contact_data)[1].indicator;
//     // right_hind_contact_     = (*contact_data)[2].indicator;
//     // left_hind_contact_      = (*contact_data)[3].indicator;
//     return;
// }


// Set q and dq to 0
void CheetahState::clear() {
    q_ = Eigen::Matrix<double,18,1>::Zero();
    dq_ = Eigen::Matrix<double,18,1>::Zero();
    // GRF_ = Eigen::Matrix<double,4,1>::Zero();
    right_front_contact_ = 0;
    left_front_contact_ = 0;
    right_hind_contact_ = 0;
    left_hind_contact_ = 0;
    return; 
}


// Get q and dq
Eigen::Matrix<double,18,1>  CheetahState::q() const { return q_; }
Eigen::Matrix<double,18,1>  CheetahState::dq() const { return dq_; }

// Get base position
Eigen::Vector3d  CheetahState::getPosition() const { return q_.segment<3>(0); }

// Get base quaternion
Eigen::Quaternion<double>  CheetahState::getQuaternion() const { return Eigen::Quaternion<double>(this->getRotation()); }

// Get rotation matrix
Eigen::Matrix3d  CheetahState::getRotation() const { return Euler2Rotation(this->getEulerAngles()); }

// Extract Euler angles and rates
Eigen::Vector3d CheetahState::getEulerAngles() const { return q_.segment<3>(3); }
Eigen::Vector3d CheetahState::getEulerRates() const { return dq_.segment<3>(3); }

Eigen::Vector3d CheetahState::getAngularVelocity() const { 
    return EulerRates2AngularVelocity(this->getEulerAngles(), this->getEulerRates()); 
}

// Extract encoder positions
Eigen::Matrix<double, 12, 1> CheetahState::getEncoderPositions() const{
    return q_.segment<12>(6); //<! take 12 elements start from idx = 6
}

Eigen::Matrix<double,12,1> CheetahState::getEncoderVelocities() const {
    return dq_.segment<12>(6); //<! take 12 elements start from idx = 6
}

Eigen::Vector3d CheetahState::getKinematicVelocity() const {
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d w = this->getAngularVelocity();
    Eigen::Matrix<double,12,1> e = this->getEncoderPositions();
    Eigen::Matrix<double,12,1> e_dot = this->getEncoderVelocities();
    ///TODO: figure out how to compute this later if needed
    // if (left_contact_ == 1) {
    //     Eigen::Vector3d pL = p_VectorNav_to_LeftToeBottom(e); // {I}_p_{IL}
    //     Eigen::Matrix<double,3,12> J_pL = Jp_VectorNav_to_LeftToeBottom(e);
    //     velocity = -J_pL*e_dot - skew(w)*pL; // {I}_v_{WI}
    // } else if (right_contact_ == 1) {
    //     Eigen::Vector3d pR = p_VectorNav_to_RightToeBottom(e); // {I}_p_{IR}
    //     Eigen::Matrix<double,3,12> J_pR = Jp_VectorNav_to_RightToeBottom(e);
    //     velocity = -J_pR*e_dot - skew(w)*pR; // {I}_v_{WI}
    // } 
    return velocity;
}

uint8_t CheetahState::getLeftFrontContact() const {
    return left_front_contact_;
}

uint8_t CheetahState::getLeftHindContact() const {
    return left_hind_contact_;
}

uint8_t CheetahState::getRightFrontContact() const {
    return right_front_contact_;
}

uint8_t CheetahState::getRightHindContact() const {
    return right_hind_contact_;
}

void CheetahState::setBaseRotation(const Eigen::Matrix3d& R) {
    q_.segment<3>(3) = Rotation2Euler(R);
}

void CheetahState::setBasePosition(const Eigen::Vector3d& p) {
    q_.segment<3>(0) = p;
}

void CheetahState::setBaseVelocity(const Eigen::Vector3d& v) {
    dq_.segment<3>(0) = v;
}

Eigen::Vector3d CheetahState::getBodyVelocity() const { 
    Eigen::Vector3d v_world = dq_.segment<3>(0);
    Eigen::Matrix3d Rwb = this->getRotation();
    return Rwb.transpose() * v_world;
}

// Extract each DOF position by name
double CheetahState::x() const { return q_(0); }
double CheetahState::y() const { return q_(1); }
double CheetahState::z() const { return q_(2); }
double CheetahState::yaw() const { return q_(3); }
double CheetahState::pitch() const { return q_(4); }
double CheetahState::roll() const { return q_(5); }
double CheetahState::rightFrontMotor1() const { return q_(6); }
double CheetahState::rightFrontMotor2() const { return q_(7); }
double CheetahState::rightFrontMotor3() const { return q_(8); }
double CheetahState::leftFrontMotor1() const { return q_(9); }
double CheetahState::leftFrontMotor2() const { return q_(10); }
double CheetahState::leftFrontMotor3() const { return q_(11); }
double CheetahState::rightHindMotor1() const { return q_(12); }
double CheetahState::rightHindMotor2() const { return q_(13); }
double CheetahState::rightHindMotor3() const { return q_(14); }
double CheetahState::leftHindMotor1() const { return q_(15); }
double CheetahState::leftHindMotor2() const { return q_(16); }
double CheetahState::leftHindMotor3() const { return q_(17); }

// Extract each DOF velocity by name
double CheetahState::dx() const { return dq_(0); }
double CheetahState::dy() const { return dq_(1); }
double CheetahState::dz() const { return dq_(2); }
double CheetahState::dyaw() const { return dq_(3); }
double CheetahState::dpitch() const { return dq_(4); }
double CheetahState::droll() const { return dq_(5); }
double CheetahState::drightFrontMotor1() const { return dq_(6); }
double CheetahState::drightFrontMotor2() const { return dq_(7); }
double CheetahState::drightFrontMotor3() const { return dq_(8); }
double CheetahState::dleftFrontMotor1() const { return dq_(9); }
double CheetahState::dleftFrontMotor2() const { return dq_(10); }
double CheetahState::dleftFrontMotor3() const { return dq_(11); }
double CheetahState::drightHindMotor1() const { return dq_(12); }
double CheetahState::drightHindMotor2() const { return dq_(13); }
double CheetahState::drightHindMotor3() const { return dq_(14); }
double CheetahState::dleftHindMotor1() const { return dq_(15); }
double CheetahState::dleftHindMotor2() const { return dq_(16); }
double CheetahState::dleftHindMotor3() const { return dq_(17); }


// Print out state information
std::ostream& operator<<(std::ostream& os, const  CheetahState& obj) {
    os << "q: [";
    for (int i=0; i<obj.q_.rows()-1; ++i) {
        os << obj.q_(i) << ", ";
    } 
    os << obj.q_(obj.q_.rows()-1) << "]\n";

    os << "dq: [";
    for (int i=0; i<obj.dq_.rows()-1; ++i) {
        os << obj.dq_(i) << ", ";
    } 
    os << obj.dq_(obj.dq_.rows()-1) << "]";
}
