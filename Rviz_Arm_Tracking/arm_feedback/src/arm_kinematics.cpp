/********************************* Kinematics Example *******************************/

  // clean tutorial on using the kinematics
  // http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/kinematics/src/doc/kinematic_model_tutorial.html  
  // https://github.com/ros-planning/moveit_tutorials/blob/indigo-devel/doc/pr2_tutorials/kinematics/src/kinematic_model_tutorial.cpp
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <vector>

using namespace std;

class ArmKinematics
{
private:
	robot_model_loader::RobotModelLoader robot_model_loader;
	robot_model::RobotModelPtr kinematic_model;
	robot_state::RobotStatePtr kinematic_state;
	robot_state::JointModelGroup* joint_model_group;

public:
	ArmKinematics()
	{
		robot_model_loader = robot_model_loader::RobotModelLoader("robot_description");
		kinematic_model = robot_model_loader.getModel();

		kinematic_state = robot_state::RobotStatePtr (new robot_state::RobotState(kinematic_model));
		joint_model_group = kinematic_model->getJointModelGroup("arm");

		kinematic_state->setToDefaultValues();
	}
	~ArmKinematics(){};

	Eigen::Affine3d forwardKinematics(const vector<double> &angles)
	{
		kinematic_state->setJointGroupPositions(joint_model_group, angles);
		return kinematic_state->getGlobalLinkTransform("EE"); 	
	}
	
	bool inverseKinematics(const Eigen::Affine3d &end_effector_state, vector<double> &angles)
	{
		bool res = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);
		
		if (res)
			kinematic_state->copyJointGroupPositions(joint_model_group, angles);
		else
			angles = vector<double>();

		return res;
	}
};