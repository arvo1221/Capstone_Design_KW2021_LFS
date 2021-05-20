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
#include <ros/package.h>
#include <ros/node_handle.h>

// // RBDL Set
// #include "../estimation_pkg/YamlConfig.h"
// #include <urdf/model.h>
// #include <rbdl/rbdl.h>
// #include <rbdl/addons/urdfreader/urdfreader.h>

// //tracker IK includes
// #include <boost/date_time.hpp>
// #include <trac_ik/trac_ik.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl_parser/kdl_parser.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>

// using namespace RigidBodyDynamics;

class KF_drone_estimator
{
    public:
    KF_drone_estimator();
    ~KF_drone_estimator();

    void AddObservation(Eigen::Vector3d);
    
    inline void setTheadCondition(bool sig){ thread_join = sig; };

    //inline Eigen::Vector3d GetPosition(){ return result_position; };
    inline double getTarget_Y(){ return target_Y; };
    inline double getTarget_P(){ return target_P; };
    inline double getTarget_Tilt(){return target_tilt;};
    // atomic operator 사용 --> C++20 부터 double , float 지원
    void excute_timerThread();

    // void initModel();

    //////   public params //////////////////////////////////////////////

    ///// CAD Model Sturcture //////////////////////////////////////////
    
    // YAMLConfig config_;
    // Model rbdl_model_;
	// unsigned int rail_frame_end_id_;
	// unsigned int base_frame_start_id_;
    // Eigen::Vector3d rail_frame_end_;
	// Eigen::Vector3d base_frame_start_;
    // std::string urdf_param_;

    // KDL::JntArray IK_lb, IK_ub;
	// KDL::Chain IK_chain;
	// KDL::Tree IK_tree;
  	// urdf::Model IK_robot_model;

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

    Eigen::Vector3f result_position;
    Eigen::Vector3f T_BO_result_position;
};
