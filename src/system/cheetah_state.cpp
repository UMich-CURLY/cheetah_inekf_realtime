#include <system/cheetah_state.hpp>

// void set(cheetah_lcm_data_t* cheetah_data);

// Eigen::Vector3d getKinematicVelocity();

Eigen::Matrix<double, 12, 1> CheetahState::getEncoderPositions() {
    Eigen::Matrix<double, 12, 1> temp;
    return temp;
}

double CheetahState::getLeftFrontContact() const {
    return 0.0;
}

double CheetahState::getLeftBackContact() const {
    return 0.0;
}

double CheetahState::getRightFrontContact() const {
    return 0.0;
}

double CheetahState::getRightBackContact() const {
    return 0.0;
}

void CheetahState::setBaseRotation(const Eigen::Matrix3d& R) {

}

void CheetahState::setBasePosition(const Eigen::Vector3d& p) {

}

void CheetahState::setBaseVelocity(const Eigen::Vector3d& v) {

}