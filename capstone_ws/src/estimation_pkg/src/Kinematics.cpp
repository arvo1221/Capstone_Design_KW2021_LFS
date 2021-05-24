#include "../include/estimation_pkg/Kinematics.h"

kinematics_sovler::kinematics_sovler(YAMLConfig &config):
config_(config)
{

}

void kinematics_sovler::initModel()
{
    std::string urdf_absolute_path;
	std::string mod_url = config_.urdf_path;
	if (config_.urdf_path.find("package://") == 0)
	{
		mod_url.erase(0, strlen("package://"));
		size_t pos = mod_url.find("/");
		if (pos == std::string::npos)
		{
			std::cout << "Could not parse package:// format into file:// format" << std::endl;;
		}
		std::string package = mod_url.substr(0, pos);
		mod_url.erase(0, pos);
		std::string package_path = ros::package::getPath(package);

		if (package_path.empty())
		{
			std::cout << "Package does not exist" << std::endl;;
		}

		urdf_absolute_path =  package_path + mod_url;
	}

     RigidBodyDynamics::Addons::URDFReadFromFile(urdf_absolute_path.c_str(), &rbdl_model_, false, false);
}