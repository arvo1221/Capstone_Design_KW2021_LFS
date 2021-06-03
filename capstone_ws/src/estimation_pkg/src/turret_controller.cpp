#pragma once
#include "../include/estimation_pkg/turret_controller.hpp"

turret_controller_interface::turret_controller_interface(ros::NodeHandle &nh_,double hz_):
    rate_(hz_),
    position_estimator()
{
    ////////   publisher ////////////////////////////////

    visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/VisionX/markers", 100);

    /////////  subsciber  ////////////////////////////////

    yolo_detection_sub = nh_.subscribe("/darknet_ros_3d/bounding_boxes",10, &turret_controller_interface::vision_cb, this);

    ////////  msgs filter Sub ////////////////////////////

    // aligned_image_sub.subscribe(nh_,"/camera/aligned_depth_to_color/image_raw",1);
    // aligned_info_sub.subscribe(nh_,"/camera/aligned_depth_to_color/camera_info",1);
    
    // sync = &message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>(aligned_image_sub,aligned_info_sub,10);
    // sync->registerCallback(boost::bind(&turret_controller_interface::object_track, _1, _2));

    // nh_.param("robot_description", robot_desc_string, std::string());
    // joint_name ={"base_link","Yaw_Link","Rail_Link","Pitch_Link"
    //             ,"Camera_Pitch_Link","Camera_Link"};

    ///////    parmeter Initialize ////////////////////////
    curConfig.resize(4);
    preConfig.resize(4);
    droneConfig.resize(4);
    thread_num = 2;
    thread_condition  = false;

    //////     set Thread         /////////////////////////

    threads.push_back(std::thread(&KF_drone_estimator::excute_timerThread, &position_estimator));
    threads.push_back(std::thread(&turret_controller_interface::SerailThread, this));
}

turret_controller_interface::~turret_controller_interface()
{
    thread_condition = true;
    position_estimator.setTheadCondition(true);
    for(auto & thread : threads)
    {   
        thread.join();
    }
}

void turret_controller_interface::vision_cb(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &pose)
{
    // at least one object detected by YOLO KF filter estimate pose
    if(pose->bounding_boxes.size() <= 0){
        //std::cout<< "callback not ok " << std::endl;
        return;
    }

   // std::cout<< "callback ok 1" << std::endl;
    Eigen::Matrix4f T_BC = position_estimator.getWordTransform();
    double min_dist = 5e3;
    int idx = 0;

    for(unsigned int i = 0; i < pose->bounding_boxes.size(); i++)
    {
        curConfig(0) = (pose->bounding_boxes.at(i).xmax + pose->bounding_boxes.at(i).xmin)/2.;
        curConfig(1) = (pose->bounding_boxes.at(i).ymax + pose->bounding_boxes.at(i).ymin)/2.;
        curConfig(2) = (pose->bounding_boxes.at(i).zmax + pose->bounding_boxes.at(i).zmin)/2.;
        curConfig(3) = 1;

        double dist = (preConfig - T_BC*curConfig).norm();

        if(dist < min_dist)
        {
            min_dist = dist;
            idx = i;
        }
    }

    curConfig(0) = (pose->bounding_boxes.at(idx).xmax + pose->bounding_boxes.at(idx).xmin)/2.;
    curConfig(1) = (pose->bounding_boxes.at(idx).ymax + pose->bounding_boxes.at(idx).ymin)/2.;
    curConfig(2) = (pose->bounding_boxes.at(idx).zmax + pose->bounding_boxes.at(idx).zmin)/2.;
    curConfig(3) = 1;

    // T_BO = T_BC * T_CO
    // std::cout<<"transform \n"<<T_BC<<std::endl;
    curConfig = T_BC*curConfig;
    preConfig = curConfig;
    //std::cout<<"position \n"<<curConfig.transpose()<<std::endl;
    //std::cout << "artan ( y, x)" << atan2(curConfig(1),curConfig(0)) << std::endl;
    // std::cout<< "callback ok 2" << std::endl;
    position_estimator.AddObservation(curConfig);
    //if(turret_mode ==0) turret_mode = 1;
}

void turret_controller_interface::SerailThread() {  
    while(!thread_condition)
    {
        std::chrono::system_clock::time_point thread_start = std::chrono::system_clock::now();
        static int count;
        turret_mode = position_estimator.getMode();
        switch(turret_mode) {
            case 0:             // stay 
                if(count % 5 == 0){
                   std::cout << "wating detect drone" << std::endl;
                }
            break;
            case 1:             // tracking
            Turret_serial_.m_target.position_Y = position_estimator.getTarget_Y()*180./M_PI;//+Turret_serial_.m_current.position_Y * M_PI/180;

           // Turret_serial_.m_target.position_Y = atan2(curConfig(1),curConfig(0))*180./M_PI;
            Turret_serial_.m_target.position_P = -position_estimator.getTarget_P()*180./M_PI;
          //  Camera_serial_.m_target.position_P = position_estimator.getTarget_Tilt();
            Camera_serial_.m_target.shooting = 0; 
         //   std::cout << "Shoot desired : " << position_estimator.getShoot() << std::endl;

           // std::cout << "drone dected" << std::endl;
            break;
            case 2:             //  shooting
            std::cout << "Shooting ........" << std::endl;
            Turret_serial_.m_target.position_Y = position_estimator.getTarget_Y()*180./M_PI;//+Turret_serial_.m_current.position_Y * M_PI/180;
            Turret_serial_.m_target.position_P = -position_estimator.getTarget_P()*180./M_PI;
            std::cout << "Target_pos_Y = " << Turret_serial_.m_target.position_Y << std::endl;
            std::cout << "Target_vel_P = " << Turret_serial_.m_target.position_P<< std::endl;

            break;
            case 3:
            Camera_serial_.m_target.shooting = position_estimator.getShoot(); 
         //   std::cout << "Shoot End : " << position_estimator.getShoot() << std::endl;

            break;
        }


        Turret_serial_.Execute();
        Camera_serial_.Execute();

        //mutex_.lock();
        position_estimator.updateKinematics_Y(Turret_serial_.m_current.position_Y * M_PI/180);
        position_estimator.updateKinematics_P(Turret_serial_.m_current.position_P * M_PI/180);
        position_estimator.updateKinematics_Tilt(0);
        //mutex_.unlock();

        if(count % 5 == 0){
            count = 0;
           // std::cout<< Turret_serial_.m_sendPacket.data.pos_Y*180/M_PI << std::endl;
         //   std::cout<< Turret_serial_.m_sendPacket.data.pos_P << std::endl;
        //    std::cout << "Target_pos_Y = " << Turret_serial_.m_target.position_Y << std::endl;
         //   std::cout << "Target_vel_P = " << Turret_serial_.m_target.position_P<< std::endl;
        }
        count++;
        std::chrono::duration<double> sec = std::chrono::milliseconds(50) 
                                    + thread_start - std::chrono::system_clock::now();
        //std::cout << "sec : " << sec.count() << std::endl;

        if(sec.count() >= 0 ) std::this_thread::sleep_for(sec);
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
            ///sdfasdfasf
    }
}

void turret_controller_interface::killProcess()
{
    thread_condition = true;
    position_estimator.setTheadCondition(true);
    for(auto & thread : threads)
    {   
        thread.join();
    }
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