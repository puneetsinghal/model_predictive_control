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

#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <ros/console.h>
#include <arm_dynamics.hpp>
#include <arm_kinematics.cpp>
using namespace Eigen;
using namespace std;




int main(int argc, char* argv[])
{
  ros::init(argc, argv, "kdc_arm_node");

/*  ros::NodeHandle nh;
  ros::Publisher jointState_pub = nh.advertise<sensor_msgs::JointState>("jointState_fbk",0);
  ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("trajectoryCmd_fbk",0);
  ros::Publisher armcontroller_pub = nh.advertise<arm_feedback::FeedbackML>("armcontroller_fbk",0);
	ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate loop_rate(100);
	
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener listener(tfBuffer);*/

  /********************************* Dynamics Example *******************************/
  ROS_INFO(" Hello World Kalyan !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

  VectorXd theta(5);
  theta << 0,0,0,0,0;
  VectorXd omega(5);
  omega << 0,0,0,0,0;
  VectorXd alpha(5);
  alpha << 0,0,0,0,0;
  VectorXd torque;

  inverseDynamics(theta, omega, alpha, torque);

  /*forwardDynamics(const Eigen::VectorXd &theta, const Eigen::VectorXd &omega, 
            const Eigen::VectorXd &torque,  Eigen::VectorXd &alpha)*/


  /********************************* Kinematics Example *******************************/

  // clean tutorial on using the kinematics
  // http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/kinematics/src/doc/kinematic_model_tutorial.html  
  // https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/kinematics/src/kinematic_model_tutorial.cpp
  ArmKinematics kin_solver;
  vector<double> angles(5,0);

  // forward kinematics
  Eigen::Affine3d end_effector_pose = kin_solver.forwardKinematics(angles);

  // inverse kinematics
  bool success = kin_solver.inverseKinematics(end_effector_pose, angles);

  cout << "Inverse Kinematics case: "<< success << endl;

  return 0;

}


