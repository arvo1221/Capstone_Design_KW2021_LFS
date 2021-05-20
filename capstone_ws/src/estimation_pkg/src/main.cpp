#include "../include/estimation_pkg/turret_controller.hpp"

int main(int argc, char **argv)
{
    ros::NodeHandle nh;
    turret_controller_interface CommandCenter(nh, 10);
    std::string port = "/dev/ttyUSB0";
    std::string port2 = "/dev/ttyUSB1";

    CommandCenter.OpenCameraSerial(port2,115200);
    CommandCenter.OpenTurretSerial(port,115200);
    while(1);

    return 0;
}