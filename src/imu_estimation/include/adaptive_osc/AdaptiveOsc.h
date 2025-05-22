<<<<<<< HEAD
#ifndef _ADAPTIVEOSC_H
#define _ADAPTIVEOSC_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "common/gesture.h"
#include <cmath>
#include "ros/ros.h"
#include <iostream>

using namespace std;

class AdaptiveOscillator{
    public:
    int NUM;
    AdaptiveOscillator(double init_freq = 2 * M_PI * 0.5, double dt = 1/400)
        : dt_(dt), order_(3), base_freq_(init_freq) {

        para_ = Eigen::MatrixXd::Zero(order_, 3);
        para_dot_ = Eigen::MatrixXd::Zero(order_, 3);
        q_esti_ = Eigen::VectorXd::Zero(order_);
        y_ = 0.0;

        double T = 2 * M_PI / init_freq;
        double T_aefa = 2 * T;
        double T_amig = 2 * T;
        yita_ = 2 / T_aefa;
        vaomig_ = 20 / (T_amig * T_amig);
        vfai_ = std::sqrt(24.2 * vaomig_);

        // Initial values
        para_(0, 0) = 0;              // alpha_0 = 0
        para_(0, 2) = M_PI / 2;       // phase for offset
        para_(1, 0) = 0.4;            // amplitude
        para_(1, 1) = init_freq;      // frequency
        para_(1, 2) = 0.0;
        para_(2, 1) = 2 * init_freq;  // 2nd harmonic frequency
    }

    void reset(double q, double dq, double freq) {
        double T = 2 * M_PI / freq;
        double T_aefa = 2 * T;
        double T_amig = 2 * T;
        yita_ = 2 / T_aefa;
        vaomig_ = 20 / (T_amig * T_amig);
        vfai_ = std::sqrt(24.2 * vaomig_);
        base_freq_ = 2 * M_PI * 0.5;

        para_(0, 0) = 0;
        para_(0, 2) = M_PI / 2;
        para_(1, 0) = 0.4;
        para_(1, 1) = freq;
        para_(1, 2) = std::atan2(freq * q, dq);
        para_(1, 0) = std::max(q / std::sin(para_(1, 2)), 0.3);

        for (int j = 2; j < order_; ++j) {
            para_(j, 1) = para_(1, 1) * j; // harmonic frequencies
        }
    }

    void update(double q) {
        for (int j = 0; j < order_; ++j)
            q_esti_(j) = para_(j, 0) * std::sin(para_(j, 2));

        y_ = q_esti_.sum();
        double F = q - y_;

        para_dot_(0, 0) = yita_ * F;

        for (int j = 1; j < order_; ++j) {
            para_dot_(j, 0) = yita_ * F * std::sin(para_(j, 2));
            if (j == 1) {
                para_dot_(j, 1) = vaomig_ * F * std::cos(para_(j, 2));
                para_dot_(j, 2) = para_(j, 1) + vfai_ * F * std::cos(para_(j, 2));
            } else {
                para_dot_(j, 2) = para_(1, 1) * j; // higher harmonics use base freq
            }
        }

        para_(0, 0) += para_dot_(0, 0) * dt_;
        para_(0, 0) = 0;

        for (int j = 1; j < order_; ++j) {
            para_(j, 0) += para_dot_(j, 0) * dt_;
        }

        para_(1, 1) += para_dot_(1, 1) * dt_;
        if (para_(1, 1) < 0) para_(1, 1) = 0;
        double w_e = base_freq_ - para_(1, 1);
        para_(1, 1) += 5 * w_e * dt_;
        
        for (int j = 1; j < order_; ++j) {
            para_(j, 2) += para_dot_(0, 1) * (j);
        }

        for (int j = 1; j < order_; ++j) {
            para_(j, 2) += para_dot_(j, 2) * dt_;
        }
    }

    double getOutput() const {
        return y_;
    }

    double getPhase() const {
        return std::fmod(para_(1, 2), 2 * M_PI);
    }

    double getFrequency() const {
        return para_(1, 1);
    }

private:
    double dt_;
    int order_;
    double yita_, vaomig_, vfai_;
    Eigen::MatrixXd para_;      // [order][3] -> amplitude, frequency, phase
    Eigen::MatrixXd para_dot_;  // same shape as para_
    Eigen::VectorXd q_esti_;
    double y_;
    double base_freq_; // external reference freq for correction
};
=======
#ifndef _ADAPTIVEOSC_H
#define _ADAPTIVEOSC_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "common/gesture.h"
#include <cmath>
#include "ros/ros.h"
#include <iostream>

using namespace std;

class AdaptiveOscillator{
    public:
    int NUM;
    AdaptiveOscillator(double init_freq = 2 * M_PI * 0.5, double dt = 1/400)
        : dt_(dt), order_(3), base_freq_(init_freq) {

        para_ = Eigen::MatrixXd::Zero(order_, 3);
        para_dot_ = Eigen::MatrixXd::Zero(order_, 3);
        q_esti_ = Eigen::VectorXd::Zero(order_);
        y_ = 0.0;

        double T = 2 * M_PI / init_freq;
        double T_aefa = 2 * T;
        double T_amig = 2 * T;
        yita_ = 2 / T_aefa;
        vaomig_ = 20 / (T_amig * T_amig);
        vfai_ = std::sqrt(24.2 * vaomig_);

        // Initial values
        para_(0, 0) = 0;              // alpha_0 = 0
        para_(0, 2) = M_PI / 2;       // phase for offset
        para_(1, 0) = 0.4;            // amplitude
        para_(1, 1) = init_freq;      // frequency
        para_(1, 2) = 0.0;
        para_(2, 1) = 2 * init_freq;  // 2nd harmonic frequency
    }

    void reset(double q, double dq, double freq) {
        double T = 2 * M_PI / freq;
        double T_aefa = 2 * T;
        double T_amig = 2 * T;
        yita_ = 2 / T_aefa;
        vaomig_ = 20 / (T_amig * T_amig);
        vfai_ = std::sqrt(24.2 * vaomig_);
        base_freq_ = 2 * M_PI * 0.5;

        para_(0, 0) = 0;
        para_(0, 2) = M_PI / 2;
        para_(1, 0) = 0.4;
        para_(1, 1) = freq;
        para_(1, 2) = std::atan2(freq * q, dq);
        para_(1, 0) = std::max(q / std::sin(para_(1, 2)), 0.3);

        for (int j = 2; j < order_; ++j) {
            para_(j, 1) = para_(1, 1) * j; // harmonic frequencies
        }
    }

    void update(double q) {
        for (int j = 0; j < order_; ++j)
            q_esti_(j) = para_(j, 0) * std::sin(para_(j, 2));

        y_ = q_esti_.sum();
        double F = q - y_;

        para_dot_(0, 0) = yita_ * F;

        for (int j = 1; j < order_; ++j) {
            para_dot_(j, 0) = yita_ * F * std::sin(para_(j, 2));
            if (j == 1) {
                para_dot_(j, 1) = vaomig_ * F * std::cos(para_(j, 2));
                para_dot_(j, 2) = para_(j, 1) + vfai_ * F * std::cos(para_(j, 2));
            } else {
                para_dot_(j, 2) = para_(1, 1) * j; // higher harmonics use base freq
            }
        }

        para_(0, 0) += para_dot_(0, 0) * dt_;
        para_(0, 0) = 0;

        for (int j = 1; j < order_; ++j) {
            para_(j, 0) += para_dot_(j, 0) * dt_;
        }

        para_(1, 1) += para_dot_(1, 1) * dt_;
        if (para_(1, 1) < 0) para_(1, 1) = 0;
        double w_e = base_freq_ - para_(1, 1);
        para_(1, 1) += 5 * w_e * dt_;

        for (int j = 1; j < order_; ++j) {
            para_(j, 2) += para_dot_(j, 2) * dt_;
        }
    }

    double getOutput() const {
        return y_;
    }

    double getPhase() const {
        return std::fmod(para_(1, 2), 2 * M_PI);
    }

    double getFrequency() const {
        return para_(1, 1);
    }

private:
    double dt_;
    int order_;
    double yita_, vaomig_, vfai_;
    Eigen::MatrixXd para_;      // [order][3] -> amplitude, frequency, phase
    Eigen::MatrixXd para_dot_;  // same shape as para_
    Eigen::VectorXd q_esti_;
    double y_;
    double base_freq_; // external reference freq for correction
};
>>>>>>> 0af27662387a1404fe8b6f6909ba863f190f46ef
#endif