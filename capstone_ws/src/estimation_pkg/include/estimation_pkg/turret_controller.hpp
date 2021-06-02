#pragma once

// basic headers
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <string>

// External Library
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

// ros msgs hearder
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/JointState.h>

// msgs filter
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// custom Header
#include "drone_estimator.hpp"

//serial Comm
#include "../include/estimation_pkg/serialComm/serial_turret.hpp"
#include "../include/estimation_pkg/serialComm/cserial.h"
#include "../include/estimation_pkg/serialComm/cserial2.h"


class turret_controller_interface
{
    public:

    turret_controller_interface(ros::NodeHandle &nh_,double hz_);
    ~turret_controller_interface();

    ////////////////methods/////////////////////////////////////////////
    inline void OpenTurretSerial(std::string port, int baudrate) {Turret_serial_.Open(port,baudrate);};
    inline void OpenCameraSerial(std::string port, int baudrate) {Camera_serial_.Open(port,baudrate);};
    
    void killProcess();

    //////   rosCallback   //////////////////////////////////////////////

    void vision_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &pose);
   // void object_track();

    //////   public params //////////////////////////////////////////////


    private:

    /////    Method //////////////////////

    int thread_num;

    bool thread_condition;

    //////  private params //////////////////////////////////////////////
    ros::Rate rate_;

    KF_drone_estimator position_estimator;

    int turret_mode;
    // serial Comm
    cserial Turret_serial_;
    cserial2 Camera_serial_;

    // Thread
    std::vector<std::thread> threads;
    std::mutex mutex_;

    Eigen::Vector4f curConfig;
    Eigen::Vector4f preConfig;
    /////   ros publisher  //////////////////////////////////////////////

    ros::Publisher visualization_pub_;

    ///// Motor Control target

    ros::Publisher servo_target_pub_;
    ros::Publisher q1_target_pub_;
    ros::Publisher q2_target_pub_;
    ros::Publisher q3_target_pub_;

    /////   ros subscriber /////////////////////////////////////////////
    ros::Subscriber yolo_detection_sub;

    ////     msgs filter Sub //////////////////////////////////////////

    // message_filters::Subscriber<sensor_msgs::Image> aligned_image_sub;
    // message_filters::Subscriber<sensor_msgs::CameraInfo> aligned_info_sub;
    // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> *sync;

    /////   private method /////////////////////////////////////////////
    Eigen::VectorXd droneConfig;

    void SerailThread();
};