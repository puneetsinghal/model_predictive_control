#include <stdio.h>
#include <math.h>
#include <iostream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#include "lookup.hpp"
#include "group_command.hpp"
#include "command.hpp"
#include "mac_address.hpp"
#include "lookup_helpers.cpp"
#include "hebi_util.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"

class SideWind {
public:
  SideWind(std::unique_ptr<hebi::Group> input_gp, std::unique_ptr<hebi::Module> input_module);
  void Sidewinding(const std_msgs::Bool::ConstPtr& mis_init);
  void ReadFeedback();
private:
  std::unique_ptr<hebi::Group> group;
  std::unique_ptr<hebi::Module> module;
  hebi::Feedback feedback;

  float period;
  long timeout_ms;
  float t;
  double spat_freq_e;
  double spat_freq_o;
  double temp_freq;
    double TPO; // for sidewinding left
    double amplitude_e;
    double amplitude_o;
    int dir;// direction of wave propagation
    double position;
    double head_pitch;
    double head_pitch_target;
    double prev_pitch_command;
    double head_kp;
    double head_kd;
  };

  SideWind::SideWind(std::unique_ptr<hebi::Group> input_gp, std::unique_ptr<hebi::Module> input_module) : group(std::move(input_gp)), module(std::move(input_module)) {
    period = 0.0025f;
    timeout_ms = 1000;
    t = 0;
    spat_freq_e = 2*3.14159/16.0;
    spat_freq_o = spat_freq_e;
    temp_freq = 0.1;
     TPO = M_PI_2+0.1; // for sidewinding left
     amplitude_e = 0.9;
     amplitude_o = 0.4;
     dir = 1;// direction of wave propagation
     position = 0;
     head_pitch_target = 0.2;
     prev_pitch_command = 0.0;
     head_kp = 1.0;
     head_kd = 4.0;
   }

   void SideWind::ReadFeedback() {
  // In a loop, send requests for feedback to the module and wait for responses.
    if (module->requestFeedback(&feedback, timeout_ms)) {
      const auto& a_fbk = feedback.imu();
      double fbk_imu_x = a_fbk.accelerometer().get().getX();
      double fbk_imu_z = a_fbk.accelerometer().get().getZ();
      head_pitch = atan2(fbk_imu_z,-fbk_imu_x);

      //std::cout << "accelX: " << a_fbk.accelerometer().get().getX() << std::endl;
    } else {
      printf("Received no feedback from module!\n");
    }
  }

  void SideWind::Sidewinding(const std_msgs::Bool::ConstPtr& mis_init) {
    std_msgs::Bool is_init = *mis_init;
    std::cout << "sidewinding cb: " << is_init.data << std::endl;

    int num_modules = group->size();

    // Create a command object; this can be sent to the group
    hebi::GroupCommand command(num_modules);

    if(is_init.data) {
      while(t < 3.0/temp_freq) {
      // Set the actuator command position field for all the modules
      int sign_e = 0; // for SEA snake modules wind helically at pi/2 increm., 
      int sign_o = 0; // so switch the sign of the amplitude every other mod.
      for (int module_index = 0; module_index < num_modules-1; module_index++) {

        if (module_index%2 == 0)
        {
          position = pow(-1,sign_e) * amplitude_e * sin(2.0*M_PI*(dir*temp_freq*t + module_index*spat_freq_e));
          sign_e++;
        }
        else
        {
          position = pow(-1,sign_o) * amplitude_o * sin(2.0*M_PI*(dir*temp_freq*t + TPO + module_index*spat_freq_o));
          sign_o++;
        }
        command[module_index].actuator().position().set(position);
      }
      ReadFeedback();
      double curr_pitch_command = head_pitch - head_pitch_target;
      position = head_kp*curr_pitch_command + head_kd * (curr_pitch_command - prev_pitch_command); 
      prev_pitch_command = curr_pitch_command;
      command[num_modules-1].actuator().position().set(position);
      std::cout << head_pitch << std::endl;

      if (group->sendCommandWithAcknowledgement(command, timeout_ms)) {
        printf("Got acknowledgement.\n");
      } else {
        printf("Did not receive acknowledgement!\n");
      }
      if (t < period)
      {
        std::cout << "press any key to continue..." << std::endl;
        std::cin.get();
      }
      hebi_sleep_ms(period * 1000);
      t += period;
    }
  } else {
    int sign_e = 0; // for SEA snake modules wind helically at pi/2 increm., 
      int sign_o = 0; // so switch the sign of the amplitude every other mod.
      for (int module_index = 0; module_index < num_modules; module_index++) {

        if (module_index%2 == 0)
        {
          position = pow(-1,sign_e) * amplitude_e * sin(2.0*M_PI*(dir*temp_freq*t + module_index*spat_freq_e));
          sign_e++;
        }
        else
        {
          position = pow(-1,sign_o) * amplitude_o * sin(2.0*M_PI*(dir*temp_freq*t + TPO + module_index*spat_freq_o));
          sign_o++;
        }
        command[module_index].actuator().position().set(position);
      }
      ReadFeedback();
      position = head_pitch - head_pitch_target; 
      command[num_modules-1].actuator().position().set(position);

      if (group->sendCommandWithAcknowledgement(command, timeout_ms))
      {
        printf("Got acknowledgement.\n");
      }
      else
      {
        printf("Did not receive acknowledgement!\n");
      }
      if (t < period)
      {
        std::cout << "press any key to continue..." << std::endl;
        std::cin.get();
      }
      hebi_sleep_ms(period * 1000);
      t += period;
      while(t < 0.25/temp_freq) {
      // Set the actuator command position field for all the modules
      int sign_e = 0; // for SEA snake modules wind helically at pi/2 increm., 
      int sign_o = 0; // so switch the sign of the amplitude every other mod.
      for (int module_index = num_modules-2; module_index < num_modules-1; module_index++) {

        if (module_index%2 == 0)
        {
          position = pow(-1,sign_e) * amplitude_e * sin(2.0*M_PI*(dir*temp_freq*t + module_index*spat_freq_e));
          sign_e++;
        }
        else
        {
          position = pow(-1,sign_o) * amplitude_o * sin(2.0*M_PI*(dir*temp_freq*t + TPO + module_index*spat_freq_o));
          sign_o++;
        }
        command[module_index].actuator().position().set(position);
      }
      ReadFeedback();
      position = head_pitch - head_pitch_target; 
      command[num_modules-1].actuator().position().set(position);

      if (group->sendCommandWithAcknowledgement(command, timeout_ms))
      {
        printf("Got acknowledgement.\n");
      }
      else
      {
        printf("Did not receive acknowledgement!\n");
      }
      hebi_sleep_ms(period * 1000);
      t += period;
    }
    while(t > 0) {
      // Set the actuator command position field for all the modules
      int sign_e = 0; // for SEA snake modules wind helically at pi/2 increm., 
      int sign_o = 0; // so switch the sign of the amplitude every other mod.
      for (int module_index = num_modules-2; module_index < num_modules-1; module_index++) {

        if (module_index%2 == 0)
        {
          position = pow(-1,sign_e) * amplitude_e * sin(2.0*M_PI*(dir*temp_freq*t + module_index*spat_freq_e));
          sign_e++;
        }
        else
        {
          position = pow(-1,sign_o) * amplitude_o * sin(2.0*M_PI*(dir*temp_freq*t + TPO + module_index*spat_freq_o));
          sign_o++;
        }
        command[module_index].actuator().position().set(position);
      }
      ReadFeedback();
      position = head_pitch - head_pitch_target; 
      command[num_modules-1].actuator().position().set(position);

      if (group->sendCommandWithAcknowledgement(command, timeout_ms))
      {
        printf("Got acknowledgement.\n");
      }
      else
      {
        printf("Did not receive acknowledgement!\n");
      }
      
      hebi_sleep_ms(period * 1000);
      t -= period;
    }
  }

}

int main(int argc, char* argv[]) {

  // Try and get the requested group.
  std::unique_ptr<hebi::Group> group = getGroupFromArgs(argc, argv);
  std::unique_ptr<hebi::Module> module = getModuleFromArgs(argc, argv);

  if (!group) {
    printf("No group found!\n");
    return -1;
  }

  if (!module) {
    std::cout << "No module found!" << std::endl;
    return -1;
  }

  SideWind sw(std::move(group), std::move(module));

    // ROS settings
  ros::init(argc, argv, "command_sidewind_example");
  ros::start();

  ros::NodeHandle nh;
  ros::NodeHandle _nh("~");

  ros::Subscriber init_sub = nh.subscribe("/ORB_SLAM2/init_state", 1, &SideWind::Sidewinding, &sw);
    // ros::Publisher init_state_pub = nh.advertise

  ros::spin();

  // NOTE: destructors automatically clean up group command and group
  return 0;
}
