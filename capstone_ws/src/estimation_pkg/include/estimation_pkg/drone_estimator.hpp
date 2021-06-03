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

// KDL header
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

class KF_drone_estimator
{
    public:
    KF_drone_estimator();
    ~KF_drone_estimator(){};

    void AddObservation(Eigen::Vector4f postion);
    
    inline void setTheadCondition(bool sig){ thread_join = sig; };

    //inline Eigen::Vector3d GetPosition(){ return result_position; };
    inline double getTarget_Y(){ return target_Y; };
    inline double getTarget_P(){ return target_P; };
    inline double getTarget_Tilt(){return target_tilt;};
    inline int    getShoot() {return shoot;}; 
    inline Eigen::Matrix4f getWordTransform(){return T_BC;};

    inline void updateKinematics_Y(double _Y_val)
    {  
        cam_joint_val_(0)  = _Y_val; 
        rail_joint_val_(0) = _Y_val;
        UpdateKinematics();
    };

    inline void updateKinematics_P(double _P_val)
    {  
        rail_joint_val_(1) = _P_val;
        UpdateKinematics(); 
    };

    inline void updateKinematics_Tilt(double _tilt_val)
    { 
        cam_joint_val_(1) = _tilt_val;
        UpdateKinematics();
    };

    inline int getMode() {return mode;};
    
    //////////////////////////////////////////////////////////////////

    // atomic operator 사용 --> C++20 부터 double , float 지원
    void excute_timerThread();

    //////   public params //////////////////////////////////////////////

    private:

    // Utils
    Eigen::Matrix4f Frame2Eigen(KDL::Frame &frame);
    void UpdateKinematics();

    /////   parameter  ///////////////////////////////////////
    double m_dt;
    double m_dt_old;
    double m_xf;
    double m_yf;
    double m_zf;

    unsigned int m_cnt;
    unsigned int m_old_cnt;

    int mode;
    // 나중에 스레드 활용시 사용
    //std::atomic<int> cnt_{0};

    int cnt_;
    int old_cnt_;
    int lost_cnt;

    std::mutex mutex_;

    bool call_kalman;
    bool call_lost;
    bool thread_join;

    double target_Y;
    double target_P;
    double target_tilt;
    int    shoot;

    Eigen::MatrixXd A;
    Eigen::MatrixXd A_hat;
    Eigen::MatrixXd A_shoot;
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


    Eigen::Matrix4f T_BC;

    // Hardware Frames
    Eigen::VectorXf result_position;
    Eigen::VectorXf T_BO_result_position;

    KDL::Chain cam_chain_;
    KDL::JntArray cam_joint_val_;

    KDL::Chain rail_chain_;
    KDL::JntArray rail_joint_val_;


    KDL::Rotation rot_;
    KDL::Frame base_frame_;
    KDL::Frame yaw_frame_;
    KDL::Frame rail_frame_;
    KDL::Frame pitch_frame_;
    KDL::Frame cam_pitch_frame_;
    KDL::Frame cam_frame_;

};
