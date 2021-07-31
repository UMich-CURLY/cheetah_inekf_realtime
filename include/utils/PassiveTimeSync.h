/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   PassiveTimeSync.h
 *  @author Ross Hartley
 *  @brief  Implements the passive time synchronization algorithm from Ed Olson
 *  @inproceedings{olson2010passive,
        title={A passive solution to the sensor synchronization problem},
        author={Olson, Edwin},
        booktitle={Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference on},
        pages={1059--1064},
        year={2010},
        organization={IEEE}
    }
 *  @date   November 08, 2018
 **/

#ifndef PASSIVETIMESYNC_H
#define PASSIVETIMESYNC_H 

#include <algorithm>
#include <iostream>

class PassiveTimeSync {
    public:
        PassiveTimeSync(double alpha1, double alpha2) : alpha1_(alpha1), alpha2_(alpha2), f0_(f(0)) {}
        double offset() { return A_; }
        double correct(double pi, double qi) {
            if (initialized_) {
                if ((pi-qi) >= 0) {
                    // Measurement time is greater than host time
                    if ((pi-qi-f0_) > (p_-q_-f(pi-p_))) {
                        p_ = pi; 
                        q_ = qi;
                        A_ = pi-qi-f(pi-p_);
                    } else {
                        A_ = p_-q_-f(pi-p_);
                    }
                    return pi-A_; 
                } else {
                    // Measurement time is smaller than host time 
                    if ((qi-pi+f0_) < (q_-p_+f(pi-p_))) {
                        q_ = qi; 
                        p_ = pi;
                        A_ = qi-pi+f(pi-p_);
                    } else {
                        A_ = q_-p_+f(pi-p_);
                    }
                    return pi+A_; 
                }
            } else {
                // Store first timestamps
                p_ = pi;
                q_ = qi;
                A_ = pi-qi;
                initialized_ = true;
                if (A_ > 0) {
                    return pi-A_;
                } else {
                    A_ = -A_;
                    return pi+A_;
                }
            }
        }

    private:
        double A_ = 0; // Estimated time offset
        double alpha1_;
        double alpha2_;
        double f0_;
        double p_;
        double q_;
        bool initialized_ = false;

        double f(double delta) {
            return std::max(alpha2_*delta/(1+alpha2_), alpha1_*delta/(1-alpha1_));
        }
};


#endif 
