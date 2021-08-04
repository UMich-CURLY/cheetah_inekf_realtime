#include <system/cheetah_state.hpp>

void set(cheetah_lcm_data_t* cheetah_data);

Eigen::Vector3d getKinematicVelocity();

Eigen::Matrix<double, 12, 1> getEncoderPositions();

double getLeftFrontContact() const;

double getLeftBackContact() const;

double getRightFrontContact() const;

double getRightBackContact() const;

void setBaseRotation(Eigen::Matrix3d);

void setBasePosition(Eigen::Vector3d);

void setBaseVelocity(Eigen::Vector3d);