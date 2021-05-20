#pragma once
#include "../include/estimation_pkg/turret_controller.hpp"

turret_controller_interface::turret_controller_interface(ros::NodeHandle &nh_,double hz_):
    rate_(hz_),
    position_estimator()
{
    ////////   publisher ////////////////////////////////

    visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/VisionX/markers", 100);

    /////////  subsciber  ////////////////////////////////

    yolo_detection_sub = nh_.subscribe("/darknet_ros_3d/bounding_boxes",1, &turret_controller_interface::vision_cb, this);

    ////////  msgs filter Sub ////////////////////////////

    // aligned_image_sub.subscribe(nh_,"/camera/aligned_depth_to_color/image_raw",1);
    // aligned_info_sub.subscribe(nh_,"/camera/aligned_depth_to_color/camera_info",1);
    
    // sync = &message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>(aligned_image_sub,aligned_info_sub,10);
    // sync->registerCallback(boost::bind(&turret_controller_interface::object_track, _1, _2));

    ///////    parmeter Initialize ////////////////////////
    droneConfig.resize(4);

    Cam_objtrack  = false;

    //////     set Thread         /////////////////////////

    threads.push_back(std::thread(&KF_drone_estimator::excute_timerThread, &position_estimator));
    threads.push_back(std::thread(&turret_controller_interface::SerailThread, this));
}

turret_controller_interface::~turret_controller_interface()
{
    position_estimator.setTheadCondition(true);
    threads.at(0).join();
}

void turret_controller_interface::vision_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &pose)
{
    // at least one object detected by YOLO KF filter estimate pose
    if(pose->bounding_boxes.size() <= 0)
        return;
    
    Eigen::Vector3d curConfig;
    curConfig.resize(3);

    curConfig(0) = (pose->bounding_boxes.at(0).xmax + pose->bounding_boxes.at(0).xmin)/2.;
    curConfig(1) = (pose->bounding_boxes.at(0).xmax + pose->bounding_boxes.at(0).xmin)/2.;
    curConfig(2) = (pose->bounding_boxes.at(0).xmax + pose->bounding_boxes.at(0).xmin)/2.;

    position_estimator.AddObservation(curConfig);
}
void turret_controller_interface::SerailThread() {
    Turret_serial_.m_target.position_Y = position_estimator.getTarget_Y();
    Turret_serial_.m_target.position_P = position_estimator.getTarget_P();
    Camera_serial_.m_target.position_P = position_estimator.getTarget_Tilt();
    
    Turret_serial_.Execute();
    Camera_serial_.Execute();

}


/*
void turret_controller_interface::object_track()
{
    //obj tracker Model
    //droneConfig = position_estimator.GetPosition();
    droneConfig(3) = 1;

    // trans form to word Model
    Eigen::Affine3f T_CB;
    T_CB.translation() << 0, 0 , 0;
    T_CB.linear() = Eigen::Quaternionf(0,0,0,1).toRotationMatrix();

    // droneConfig = T_CB*droneConfig;
    double target_pen = -atan2(droneConfig(0),droneConfig(1));
     
}*/