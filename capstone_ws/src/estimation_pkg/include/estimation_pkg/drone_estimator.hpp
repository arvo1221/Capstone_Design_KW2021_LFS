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
#include <ros/time.h>

class KF_drone_estimator
{
    public:
    KF_drone_estimator();
    ~KF_drone_estimator(){};

    void AddObservation(Eigen::Vector3d);
    
    inline void setTheadCondition(bool sig){ thread_join = sig; };

    //inline Eigen::Vector3d GetPosition(){ return result_position; };
    inline double getTarget_Y(){ return target_Y; };
    inline double getTarget_P(){ return target_P; };
    inline double getTarget_Tilt(){return target_tilt;};
    // atomic operator 사용 --> C++20 부터 double , float 지원
    void excute_timerThread();

    //////   public params //////////////////////////////////////////////

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

    double target_Y;
    double target_P;
    double target_tilt;

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

    // Hardware Frames
    Eigen::MatrixXf base_frame_link_;
    Eigen::MatrixXf rail_frame_link_;
    Eigen::MatrixXf pitch_frame_link_;
    Eigen::MatrixXf camera_pitch_frame_link_;
    Eigen::MatrixXf camera_frame_link_;

    Eigen::VectorXf result_position;
    Eigen::VectorXf T_BO_result_position;
};
