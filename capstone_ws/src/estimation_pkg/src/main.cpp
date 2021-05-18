
#include <ros/ros.h>
#include <ros/time.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/QR>

#include <visualization_msgs/MarkerArray.h>

#define PI 3.14159265358979323846264338327950288419716939937510582097


using Eigen::MatrixXd;

ros::Publisher estimateBox_pub;
double g_dt = 0;
double g_dt_old = 0;
double g_xf = 0.0;
double g_yf = 0.0;
double g_zf = 0.0;
unsigned int g_cnt = 0;
unsigned int old_cnt = 0;
bool call_kalman = false;
//bool offset = false;

MatrixXd A(6,6);
MatrixXd F(6,6);
MatrixXd H(3,6);
MatrixXd Q(6,6);
MatrixXd R(3,3);
MatrixXd x_hat(6,1);
MatrixXd x_tilde(6,1);
MatrixXd x_hat_t(6,1);
MatrixXd P_hat(6,6);
MatrixXd P_tilde(6,6);
MatrixXd K(6,3);
MatrixXd mat_temp(3,3);
MatrixXd z(3,1);

double g_x_length;
double g_y_length;
double g_z_length;
void * stamp;
void * header;
//double h_time = 0;
//double h_pre_time = 0;


visualization_msgs::MarkerArray msg;
visualization_msgs::Marker bbx_makers;

void counter(const ros::TimerEvent&){
     
     g_cnt++;
     if(g_cnt > 30000) {
          g_cnt = g_cnt - old_cnt;
          old_cnt = 0;
     }
     if(call_kalman == true) {
          x_hat_t = x_hat;
          call_kalman = false;
     }
     else {
          x_hat_t = F*x_hat_t;
     }

     

     g_xf = x_hat_t(0);
     g_yf = x_hat_t(2);
     g_zf = x_hat_t(4);

     bbx_makers.ns = "darknet3d";
     bbx_makers.id = 1;
     bbx_makers.type = visualization_msgs::Marker::CUBE;
     bbx_makers.action = visualization_msgs::Marker::ADD;
     bbx_makers.pose.position.x = g_xf;
     bbx_makers.pose.position.y = g_yf;
     bbx_makers.pose.position.z = g_zf;
     bbx_makers.pose.orientation.x = 0.0;
     bbx_makers.pose.orientation.y = 0.0;
     bbx_makers.pose.orientation.z = 0.0;
     bbx_makers.pose.orientation.w = 1.0;
     bbx_makers.scale.x = g_x_length;
     bbx_makers.scale.y = g_y_length;
     bbx_makers.scale.z = g_z_length;
     bbx_makers.color.b = 255.0;
     bbx_makers.color.g = 0;
     bbx_makers.color.r = 255.0;
     bbx_makers.color.a = 0.4;
     bbx_makers.lifetime = ros::Duration(0.5);
     bbx_makers.header.frame_id = "camera_link";
     msg.markers.push_back(bbx_makers);

     estimateBox_pub.publish(msg);
}

void Vision_CallBack(const gb_visual_detection_3d_msgs::BoundingBoxes3dConstPtr &pose)
{
    
    if(pose->bounding_boxes.size() > 0)
    {
         //prevent overflow 
          g_dt = (double)0.05*(g_cnt - old_cnt);
          old_cnt = g_cnt;

//          h_time = pose->header.stamp.toSec();

          //std::cout << h_time - h_pre_time << "\n";
          //h_pre_time = h_time;
          
          
          z(0) = (pose->bounding_boxes.at(0).xmax + pose->bounding_boxes.at(0).xmin)/2.;
          z(1) = (pose->bounding_boxes.at(0).ymax + pose->bounding_boxes.at(0).ymin)/2.;
          z(2) = (pose->bounding_boxes.at(0).zmax + pose->bounding_boxes.at(0).zmin)/2.;


          //ROS_INFO("dt is %.3f",g_dt);

          A << 1, g_dt, 0, 0,0,0,
               0, 1, 0, 0,0,0,
               0, 0, 1, g_dt,0,0,
               0, 0, 0, 1,0,0,
               0, 0, 0, 0, 1, g_dt,
               0, 0, 0, 0, 0, 1;

          x_tilde = A*x_hat;
          P_tilde = A*P_hat*A.transpose() + Q;
          //mat_temp = ;

          K = P_tilde*H.transpose()*(H*P_tilde*H.transpose() + R).inverse();

          x_hat = x_tilde + K*(z - H*x_tilde);
          P_hat = P_tilde - K*H*P_tilde;
          //x_hat_t = x_hat;
          //g_xf = x_hat(0);
          //g_yf = x_hat(2);
          //g_zf = x_hat(4);

          call_kalman = true;    

          g_x_length = pose->bounding_boxes.at(0).xmax - pose->bounding_boxes.at(0).xmin;
          g_y_length = pose->bounding_boxes.at(0).ymax - pose->bounding_boxes.at(0).ymin;
          g_z_length = pose->bounding_boxes.at(0).zmax - pose->bounding_boxes.at(0).zmin;
          //ROS_INFO();
          //bbx_makers.header.frame_id = pose->header.frame_id;
          //std::cerr<<pose->header.frame_id<<std::endl;
          //bbx_makers.header.stamp = pose->header.stamp;
          
          
    }
}

int main(int argc, char **argv)
{



    ros::init(argc,argv,"Vision");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/darknet_ros_3d/bounding_boxes",1,Vision_CallBack);
    ros::Timer timerCounter = nh.createTimer(ros::Duration(0.05),counter);//50ms
    estimateBox_pub = nh.advertise<visualization_msgs::MarkerArray>("/VisionX/markers", 100);
    
     A << 1, g_dt, 0, 0,0,0,
         0, 1, 0, 0,0,0,
         0, 0, 1, g_dt,0,0,
         0, 0, 0, 1,0,0,
         0, 0, 0, 0, 1, g_dt,
         0, 0, 0, 0, 0, 1;
     F << 1, 0.01, 0, 0,0,0,
         0, 1, 0, 0,0,0,
         0, 0, 1, 0.01,0,0,
         0, 0, 0, 1,0,0,
         0, 0, 0, 0, 1, 0.01,
         0, 0, 0, 0, 0, 1;

     H << 1, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 1, 0;
     Q << (double)(0.00000001/4), (double)(0.000001/2), 0, 0, 0, 0,
          (double)(0.000001/2), 0.0001, 0, 0,0,0,
          0, 0, (double)(0.00000001/4), (double)(0.000001/2),0,0,
          0, 0, (double)(0.000001/2), 0.0001,0,0,
          0,0,0,0,(double)(0.00000001/4), (double)(0.000001/2),
          0,0,0,0,(double)(0.000001/2), 0.0001;
     Q = 70225*Q;
     x_hat << 1, 0, 1, 0, 1, 0;
     x_hat_t = x_hat;
     R << 1, 0, 0, 
          0, 1.1,0,
          0,0,0.9;
     R = 0.5*R;
     P_hat << 1, 0, 0, 0, 0, 0, 
               0, 1, 0, 0, 0, 0,
               0, 0, 1, 0, 0, 0,
               0, 0, 0, 1, 0, 0,
               0, 0, 0, 0, 1, 0,
               0, 0, 0, 0, 0, 1;
      ROS_INFO("Say Hello to Future!\n");
      ros::spin();

    return 0;
}