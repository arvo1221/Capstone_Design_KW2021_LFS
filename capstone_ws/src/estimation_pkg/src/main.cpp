#include "../include/estimation_pkg/turret_controller.hpp"
#include "../include/estimation_pkg/Kinematics.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh("~");

    // std::string yaml_path;
    // nh.getParam("yaml_path", yaml_path);

    // YAMLConfig config;
    // config.loadConfig(yaml_path);
    std::string port;
    std::string port2;
    int baudrate;

    
    nh.getParam("port",port);
    nh.getParam("port2",port2);
    nh.getParam("baudrate",baudrate);

    turret_controller_interface CommandCenter(nh, 10);
    //std::string port = "/dev/ttyUSB0";
   // std::string port2 = "/dev/ttyUSB1";

    CommandCenter.OpenTurretSerial(port,115200);
    CommandCenter.OpenCameraSerial(port2,115200);

    while(ros::ok()){
        ros::spinOnce();
    }

    CommandCenter.killProcess();
    return 0;
}