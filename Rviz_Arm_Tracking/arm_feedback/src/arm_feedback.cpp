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


using namespace Eigen;
using namespace std;

class armcontroller
{
public:
  long timeoutFbk_ms = 10;
  trajectory_msgs::JointTrajectoryPoint jointTrajectoryPoint;

  arm_feedback::CommandML currentCmd;

  arm_feedback::FeedbackML currentFbk;
  sensor_msgs::JointState jointState;
  trajectory_msgs::JointTrajectoryPoint trajectoryCmd;

  std::deque<double> single_vel_que1;
  std::deque<double> single_vel_que2;
  std::deque<double> single_vel_que3;
  std::deque<double> single_vel_que4;
  std::deque<double> single_vel_que5;

  std::deque<double> single_temp_que1;
  std::deque<double> single_temp_que2;
  std::deque<double> single_temp_que3;
  std::deque<double> single_temp_que4;
  std::deque<double> single_temp_que5;

  std::vector<double> WMA_vel_single;
  std::vector<double> numerator_vel_single;
  std::vector<double> total_vel_single;
  std::vector<double> WMA_temp_single;
  std::vector<double> numerator_temp_single;
  std::vector<double> total_temp_single;

  int num_vel_filt = 10;
  int num_temp_filt = 10;

  std::unique_ptr<hebi::Group> arm_group;
  bool initialize;
  std::vector<double> initial_position;

  armcontroller();
  void init();
  bool updateFeedback(const ros::WallTime &);
  void getFeedbackMsg(sensor_msgs::JointState&,trajectory_msgs::JointTrajectoryPoint&,
                      arm_feedback::FeedbackML&);
  void WMAFilter(const std::vector<double> &, const std::vector<double> &,
                         std::vector<double> &, std::vector<double> &);
  void movingAverage(const std::vector<double> &,const int &,const int &,
                  double &,double &,double &) const;
};

armcontroller::armcontroller()
{

  this->initialize = false;

  WMA_vel_single = {0,0,0,0,0};
  numerator_vel_single = {0,0,0,0,0};
  total_vel_single = {0,0,0,0,0};
  WMA_temp_single = {0,0,0,0,0};
  numerator_temp_single = {0,0,0,0,0};
  total_temp_single = {0,0,0,0,0};

  this->jointTrajectoryPoint.positions = {0,0,0,0,0};
  this->jointTrajectoryPoint.velocities = {0,0,0,0,0};
  this->jointTrajectoryPoint.accelerations = {0,0,0,0,0};
  this->currentCmd.epsTau = {0,0,0,0,0};
  this->currentCmd.motorOn = 0.;

  this->trajectoryCmd.positions = {0,0,0,0,0};
  this->trajectoryCmd.velocities = {0,0,0,0,0};
  this->trajectoryCmd.accelerations = {0,0,0,0,0};
  this->jointState.position = {0,0,0,0,0};
  this->jointState.velocity = {0,0,0,0,0};
  this->jointState.effort = {0,0,0,0,0};
  this->currentFbk.deflections = {0,0,0,0,0};
  this->currentFbk.velocityFlt = {0,0,0,0,0};
  this->currentFbk.windingTemp = {0,0,0,0,0};
  this->currentFbk.windingTempFlt = {0,0,0,0,0};
  this->currentFbk.torqueCmd = {0,0,0,0,0};
  this->currentFbk.torqueID = {0,0,0,0,0};
  this->currentFbk.accel = {0,0,0,0,0};
};

void armcontroller::init()
{
  // vec_module.clear();
  // vec_module.resize(5);
  // Setup the lookup
  long timeout_ms = 5000; // Give the modules plenty of time to appear.
  float freq_hz = 500;
  hebi::Lookup lookup;
  std::vector<hebi::MacAddress> macs;
  
  // Ky:Commented out IMR Arm 1 Mac Addresses
  /*(std::vector<std::string> modules = {"d8:80:39:65:ae:44",
                                      "d8:80:39:9d:64:fd",
                                      "d8:80:39:9d:59:c7",
                                      "d8:80:39:9d:4b:cd",
                                      "d8:80:39:9c:d7:0d"};*/

  // Ky: Adding in test research arm
  std::vector<std::string> modules = {"D8:80:39:E8:B3:3C",
                                      "D8:80:39:E9:06:36",
                                      "D8:80:39:9D:29:4E",
                                      "D8:80:39:9D:3F:9D",
                                      "D8:80:39:9D:04:78"};

  //Build the vector of mac addresses anc check for correct
  //hex strings
  for(int i=0;i<5;i++)
  {
    hebi::MacAddress mac;
    if(hebi::MacAddress::isHexStringValid(modules[i]))
    {
      mac.setToHexString(modules[i]);
      macs.emplace_back(mac);
    }
    else
    {
      perror("Module Hex String is Invalid");
    }
  }

  arm_group = lookup.getGroupFromMacs(macs,timeout_ms);
  hebi_sleep_ms(100);
  arm_group->setFeedbackFrequencyHz(freq_hz);
  hebi_sleep_ms(100);
  if(!arm_group)
  {
    perror("Failed to get module group");
  }

};

bool armcontroller::updateFeedback(const ros::WallTime &time_stamp)
{
  hebi::GroupFeedback groupFeedback(this->arm_group->size());
  bool fbkSuccess = true;

  if(!arm_group->requestFeedback(&groupFeedback,3)){fbkSuccess = false;}
  if(fbkSuccess)
  {
    if(!this->initialize)
    {
      std::cout << "Modules Fully Initialized" << std::endl;
      this->initialize = true;
    }
    this->currentFbk.header.stamp.sec = time_stamp.sec;
    this->currentFbk.header.stamp.nsec = time_stamp.nsec;
    this->jointState.header.stamp.sec = time_stamp.sec;
    this->jointState.header.stamp.nsec = time_stamp.nsec;
    this->jointState.name = {std::string("Joint1"),std::string("Joint2"),std::string("Joint3"),std::string("Joint4")};

    std::vector<double> velocityUpdate(5);
    std::vector<double> windingTempUpdate(5);

    for(int i=0;i<5;i++)
    {
      this->jointState.position[i] = groupFeedback[i].actuator().position().get();
      this->jointState.velocity[i] = groupFeedback[i].actuator().velocity().get();
      this->jointState.effort[i] = groupFeedback[i].actuator().torque().get();

      this->trajectoryCmd.positions[i] = this->jointTrajectoryPoint.positions[i];
      this->trajectoryCmd.velocities[i] = this->jointTrajectoryPoint.velocities[i];
      this->trajectoryCmd.accelerations[i] = this->jointTrajectoryPoint.accelerations[i];

      this->currentFbk.torqueCmd[i] = groupFeedback[i].actuator().torqueCommand().get();
      this->currentFbk.deflections[i] = groupFeedback[i].actuator().deflection().get();
      this->currentFbk.windingTemp[i] = groupFeedback[i].actuator().motorWindingTemperature().get();

      velocityUpdate[i] = this->jointState.velocity[i];
      windingTempUpdate[i] = this->currentFbk.windingTemp[i];
    }
    
    std::vector<double> velocityFiltered(5);
    std::vector<double> windingTempFiltered(5);
    WMAFilter(velocityUpdate, windingTempUpdate, velocityFiltered, windingTempFiltered); 
    
    for(int i=0;i<5;i++)
    {
      this->currentFbk.velocityFlt[i] = velocityFiltered[i];
      this->currentFbk.windingTempFlt[i] = windingTempFiltered[i];
    }
  }
  
  // if(fbkSuccess == false){
  //   printf("Did not receive feedback!\n");
  // }
  // else
  // {
  //   printf("Got feedback.\n");
  // }
  return fbkSuccess;
}

void armcontroller::getFeedbackMsg(sensor_msgs::JointState &jointState_fbk,
                                  trajectory_msgs::JointTrajectoryPoint &trajectoryCmd_fbk,
                                  arm_feedback::FeedbackML &armcontroller_fbk)
{
  jointState_fbk = this->jointState;
  trajectoryCmd_fbk = this->trajectoryCmd;
  armcontroller_fbk = this->currentFbk;
};

void armcontroller::WMAFilter(const std::vector<double> &velocity, const std::vector<double> &motorTemp,
                       std::vector<double> &velocityFlt, std::vector<double> &motorTempFlt)
{
  this->single_vel_que1.push_back(velocity[0]);
  this->single_vel_que2.push_back(velocity[1]);
  this->single_vel_que3.push_back(velocity[2]);
  this->single_vel_que4.push_back(velocity[3]);
  this->single_vel_que5.push_back(velocity[4]);

  if(this->single_vel_que1.size()>(this->num_vel_filt+1))
  {
    this->single_vel_que1.pop_front();
    this->single_vel_que2.pop_front();
    this->single_vel_que3.pop_front();
    this->single_vel_que4.pop_front();
    this->single_vel_que5.pop_front();
  }
  int vel_sum;
  if(this->single_vel_que1.size()==this->num_vel_filt+1)
  {
    vel_sum = (this->single_vel_que1.size()*(this->single_vel_que1.size()-1))/2;
  }
  else
  {
    vel_sum = (this->single_vel_que1.size()*(this->single_vel_que1.size()+1))/2;
  }
  
  movingAverage({this->single_vel_que1.begin(),this->single_vel_que1.end()},
                 this->num_vel_filt,vel_sum,this->WMA_vel_single[0],this->numerator_vel_single[0],this->total_vel_single[0]);
  movingAverage({this->single_vel_que2.begin(),this->single_vel_que2.end()},
                 this->num_vel_filt,vel_sum,this->WMA_vel_single[1],this->numerator_vel_single[1],this->total_vel_single[1]);
  movingAverage({this->single_vel_que3.begin(),this->single_vel_que3.end()},
                 this->num_vel_filt,vel_sum,this->WMA_vel_single[2],this->numerator_vel_single[2],this->total_vel_single[2]);
  movingAverage({this->single_vel_que4.begin(),this->single_vel_que4.end()},
                 this->num_vel_filt,vel_sum,this->WMA_vel_single[3],this->numerator_vel_single[3],this->total_vel_single[3]);
  movingAverage({this->single_vel_que5.begin(),this->single_vel_que5.end()},
                 this->num_vel_filt,vel_sum,this->WMA_vel_single[4],this->numerator_vel_single[4],this->total_vel_single[4]);

  this->single_temp_que1.push_back(motorTemp[0]);
  this->single_temp_que2.push_back(motorTemp[1]);
  this->single_temp_que3.push_back(motorTemp[2]);
  this->single_temp_que4.push_back(motorTemp[3]);
  this->single_temp_que5.push_back(motorTemp[4]);

  if(this->single_temp_que1.size()>(this->num_temp_filt+1))
  {
    this->single_temp_que1.pop_front();
    this->single_temp_que2.pop_front();
    this->single_temp_que3.pop_front();
    this->single_temp_que4.pop_front();
    this->single_temp_que5.pop_front();
  }
  int temp_sum;
  if(this->single_temp_que1.size()==this->num_temp_filt+1)
  {
    temp_sum = (int)(this->single_temp_que1.size()*(this->single_temp_que1.size()-1))/2;
  }
  else
  {
    temp_sum = (int)(this->single_temp_que1.size()*(this->single_temp_que1.size()+1))/2;
  }
  movingAverage({this->single_temp_que1.begin(),this->single_temp_que1.end()},
                 this->num_temp_filt,temp_sum,this->WMA_temp_single[0],this->numerator_temp_single[0],this->total_temp_single[0]);
  movingAverage({this->single_temp_que2.begin(),this->single_temp_que2.end()},
                 this->num_temp_filt,temp_sum,this->WMA_temp_single[1],this->numerator_temp_single[1],this->total_temp_single[1]);
  movingAverage({this->single_temp_que3.begin(),this->single_temp_que3.end()},
                 this->num_temp_filt,temp_sum,this->WMA_temp_single[2],this->numerator_temp_single[2],this->total_temp_single[2]);
  movingAverage({this->single_temp_que4.begin(),this->single_temp_que4.end()},
                 this->num_temp_filt,temp_sum,this->WMA_temp_single[3],this->numerator_temp_single[3],this->total_temp_single[3]);
  movingAverage({this->single_temp_que5.begin(),this->single_temp_que5.end()},
                 this->num_temp_filt,temp_sum,this->WMA_temp_single[4],this->numerator_temp_single[4],this->total_temp_single[4]);
  for(int i=0; i<5; i++)
  {
      velocityFlt[i] = this->WMA_vel_single[i];
      motorTempFlt[i] = this->WMA_temp_single[i];
  }
};

void armcontroller::movingAverage(const std::vector<double> &data_vector,const int &num,const int &vec_sum,
                  double &WMA,double &numerator,double &total) const
{
  if(((int)data_vector.size())<(num+1))
  {
    numerator = numerator + ((double)data_vector.size())*data_vector[data_vector.size()-1];
    total = total + data_vector[data_vector.size()-1];
    WMA = numerator/(double)vec_sum;
  }
  else
  {
    numerator = numerator + ((double)num)*data_vector[num]-total;
    total = total + data_vector[num] - data_vector[0];
    WMA = numerator/(double)vec_sum;
  }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "arm_1_traj_node");

  ros::NodeHandle nh;
  ros::Publisher jointState_pub = nh.advertise<sensor_msgs::JointState>("jointState_fbk",0);
  ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("trajectoryCmd_fbk",0);
  ros::Publisher armcontroller_pub = nh.advertise<arm_feedback::FeedbackML>("armcontroller_fbk",0);

	ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate loop_rate(100);
	
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener listener(tfBuffer);

  // init arm controller
  armcontroller ac;
  ac.init();

  cout << "enter ros spin" << endl;
 
	visualization_msgs::Marker path;
	path.ns = "trace";
	path.header.frame_id = "world";
	path.action = visualization_msgs::Marker::ADD;
	path.pose.orientation.w = 1.0;
	path.color.a = 1.0;
	path.color.r = 1.0;
	path.id = 0;
	path.type = visualization_msgs::Marker::LINE_STRIP;
	path.scale.x = 0.005;
 
  while(1)
  {
    sensor_msgs::JointState jointState_fbk;
    trajectory_msgs::JointTrajectoryPoint trajectoryCmd_fbk;
    arm_feedback::FeedbackML armcontroller_fbk;

    if(ac.updateFeedback(ros::WallTime::now()))
    {
      ac.getFeedbackMsg(jointState_fbk,trajectoryCmd_fbk,armcontroller_fbk);
      jointState_pub.publish(jointState_fbk);
      trajectory_pub.publish(trajectoryCmd_fbk);
      armcontroller_pub.publish(armcontroller_fbk);
		}

		geometry_msgs::TransformStamped transformStamped;
		try{
				transformStamped = tfBuffer.lookupTransform("wrist", "EE", ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		geometry_msgs::Point point;
		point.x = transformStamped.transform.translation.x;
		point.y = transformStamped.transform.translation.y;
		point.z = transformStamped.transform.translation.z;
		path.points.push_back(point);
		path_pub.publish(path);

		path.header.stamp = ros::Time::now();
		if (path.points.size() == 500) {
			path.points.erase(path.points.begin(), path.points.begin()+1);
		}

		loop_rate.sleep();
	
  }

  return 0;

}


