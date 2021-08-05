#ifndef BODYESTIMATOR_H
#define BODYESTIMATOR_H


#include <Eigen/Dense>
#include <vector>
#include "ros/ros.h"
#include "utils/imu.hpp"
#include "system/cheetah_state.hpp"
#include "InEKF.h"

class BodyEstimator {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        BodyEstimator();
        bool enabled();
        void disable();
        bool biasInitialized();
        void initBias();
        void initState();
        void update(cheetah_lcm_packet_t& cheetah_data, CheetahState& state);
        inekf::InEKF getFilter() const;
        inekf::RobotState getState() const;

    private:
        inekf::InEKF filter_;
        bool enabled_ = false;
        bool bias_initialized_ = false;
        bool static_bias_initialization_ = false;
        std::vector<Eigen::Matrix<double,6,1>,Eigen::aligned_allocator<Eigen::Matrix<double,6,1>>> bias_init_vec_;
        Eigen::Vector3d bg0_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d ba0_ = Eigen::Vector3d::Zero();
        double t_prev_;
        Eigen::Matrix<double,6,1> imu_prev_;
        const Eigen::Matrix<double,12,12> encoder_cov_ = 0.0003 * Eigen::Matrix<double,12,12>::Identity(); // 1 deg std dev 
        const Eigen::Matrix<double,3,3> prior_kinematics_cov_ = 0.003 * Eigen::Matrix<double,3,3>::Identity(); // Adds to FK covariance
};

#endif // BODYESTIMATOR_H