#include <stdio.h>
#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include "lookup.hpp"
#include "module.hpp"
#include "group.hpp"
#include "command.hpp"
#include "mac_address.hpp"
#include "lookup_helpers.cpp"
#include "hebi_util.h"
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <sstream>
#include <deque>
#include <arm_feedback/CommandML.h>
#include <arm_feedback/FeedbackML.h>
#include <arm_feedback/FI.h>

#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <ros/console.h>
#include <arm_dynamics.hpp>
#include <arm_kinematics.cpp>
#include <limits>

using namespace Eigen;
using namespace std;

class FI {

  ArmKinematics kin_solver;
  ros::NodeHandle nh_;

  // all the services
  ros::ServiceServer fds_;
  ros::ServiceServer ids_;
  ros::ServiceServer fks_;
  ros::ServiceServer iks_;

public:



  bool forwardDynamicsSrv( arm_feedback::FI::Request &req, arm_feedback::FI::Response &res)
  {
    VectorXd theta(5);
    theta << req.input[0],req.input[1],req.input[2],req.input[3],req.input[4];
    VectorXd omega(5);
    omega << req.input[5],req.input[6],req.input[7],req.input[8],req.input[9];
    VectorXd torque(5);
    torque << req.input[10],req.input[11],req.input[12],req.input[13],req.input[14];

    Eigen::VectorXd alpha;
    
    forwardDynamics(theta, omega, torque, alpha); 
    
    res.output.resize(5);

    for (int i =0; i < 5; ++i)
    {
      res.output[i] = alpha[i];
      ROS_INFO("joint[%i]: %f", i, res.output[i]);
    }

    res.status = true;

    return true;  
  }

  bool inverseDynamicsSrv( arm_feedback::FI::Request &req, arm_feedback::FI::Response &res)
  {
    VectorXd theta(5);
    theta << req.input[0],req.input[1],req.input[2],req.input[3],req.input[4];
    VectorXd omega(5);
    omega << req.input[5],req.input[6],req.input[7],req.input[8],req.input[9];
    VectorXd alpha(5);
    alpha << req.input[10],req.input[11],req.input[12],req.input[13],req.input[14];

    Eigen::VectorXd torque;
    
    inverseDynamics(theta, omega,  alpha,  torque); 
    
    res.output.resize(5);

    for (int i =0; i < 5; ++i)
    {
      res.output[i] = 2;//torque[i];
      ROS_INFO("joint[%i]: %f", i, res.output[i]);
    }

    res.status = true;

    return true;  
  }

  bool forwardKinematicsSrv( arm_feedback::FI::Request &req, arm_feedback::FI::Response &res)
  {
    vector<double> angles;
    
    for (int i =0; i < 5; ++i)
    {
      angles.push_back(req.input[i]);
    }

    Eigen::Affine3d end_effector_pose = kin_solver.forwardKinematics(angles);
    res.output.resize(3);
    res.output[0] = end_effector_pose(0,3);
    res.output[1] = end_effector_pose(1,3);
    res.output[2] = end_effector_pose(2,3);
    res.status = true;

    return true;
  }

  bool inverseKinematicsSrv( arm_feedback::FI::Request &req, arm_feedback::FI::Response &res)
  {
    
/*    vector<double> angles;
    bool success = kin_solver.inverseKinematics(end_effector_pose, angles);

    for (int i =0; i < 5; ++i)
    {
      res.output[i] = angles[i];
    }

    res.status = success;*/


    return true;
  }

  FI(ros::NodeHandle nh) :kin_solver(), nh_(nh) 
  {
    fds_ = nh_.advertiseService("forwardDynamics", &FI::forwardDynamicsSrv, this);
    ids_ = nh_.advertiseService("inverseDynamics", &FI::inverseDynamicsSrv, this);
    fks_ = nh_.advertiseService("forwardKinematics", &FI::forwardKinematicsSrv, this);
    iks_ = nh_.advertiseService("inverseKinematics", &FI::inverseKinematicsSrv, this);
  }

};




int main(int argc, char* argv[])
{
  ros::init(argc, argv, "kdc_arm_node");

  ros::NodeHandle nh;

  FI fi_service(nh);

  ros::spin();
  return 0;

}


