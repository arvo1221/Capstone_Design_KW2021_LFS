#pragma once

// standard Header
#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

// External Header
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

// ros Header
#include <ros/ros.h>
#include <ros/time.h>


class KF_drone_estimator
{
    public:
    KF_drone_estimator();
    ~KF_drone_estimator();

    void AddObservation(Eigen::Vector3d);
    
    inline void setTheadCondition(bool sig){ thread_join = sig; };

    inline Eigen::Vector3d GetPosition(){ return result_position; };

    // atomic operator 사용 --> C++20 부터 double , float 지원
    void excute_timerThread();

    private:

    /////   parameter  ///////////////////////////////////////

    double m_dt;
    double m_dt_old;
    double m_xf;
    double m_yf;
    double m_zf;

    unsigned int m_cnt;
    unsigned int m_old_cnt;

    // 나중에 스레드 활용시 사용
    //std::atomic<int> cnt_{0};

    int cnt_;
    int old_cnt_;

    std::mutex mutex_;

    bool call_kalman;
    bool thread_join;

    Eigen::MatrixXd A;
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd x_hat;
    Eigen::MatrixXd x_tilde;
    Eigen::MatrixXd x_hat_t;
    Eigen::MatrixXd P_hat;
    Eigen::MatrixXd P_tilde;
    Eigen::MatrixXd K;
    Eigen::MatrixXd mat_temp;
    Eigen::MatrixXd z;

    Eigen::Vector3d result_position;
};
