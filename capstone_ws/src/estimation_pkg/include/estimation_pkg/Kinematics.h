//ros Header
#include <ros/package.h>
#include <ros/node_handle.h>

// urdf header
#include "YamlConfig.h"
#include <urdf/model.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

// #include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

using namespace RigidBodyDynamics;

class kinematics_sovler
{
    public:
    kinematics_sovler(YAMLConfig &config);
    ~kinematics_sovler(){};

    void initModel();

    private:

    // URDF
    YAMLConfig config_;
    Model rbdl_model_;
    std::string urdf_param_;
	unsigned int rail_frame_id_;
	unsigned int turret_base_frame_id_;
    KDL::JntArray IK_lb, IK_ub;
	KDL::Chain IK_chain;
	KDL::Tree IK_tree;
  	urdf::Model IK_robot_model;

    //params
    double eps = 5e-3;
	double num_samples = 100;
};