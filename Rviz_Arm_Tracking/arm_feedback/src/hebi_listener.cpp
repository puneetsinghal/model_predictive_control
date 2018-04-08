#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#include <boost/bind.hpp>

#include "lookup.hpp"
#include "group_command.hpp"
#include "command.hpp"
#include "mac_address.hpp"
#include "lookup_helpers.cpp"
#include "hebi_util.h"
#include "feedback_print_helpers.cpp"

class hebiSub
{
   public:
      std::vector<std_msgs::Float32> position_cmd;
      //float position_cmd[];
      int n_mod;
      hebiSub(int);
      void hebiCallback(const std_msgs::Float32MultiArray::ConstPtr&);
};

hebiSub::hebiSub(int n_modules)
{
    //std::cout << "constructor" << std::endl;
    //std_msgs::Float32 position_cmd[n_modules];
    position_cmd.resize(n_modules);
    n_mod = n_modules;    
}

void hebiSub::hebiCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::cout << "callback" << n_mod << std::endl;
    for(int k = 0; k < 16; k++)
    {
        //std::cout << n_mod << std::endl;
        position_cmd[k].data = msg->data[k];
        //std::cout << k << std::endl;
        //std::cout << msg->data[i] << std::endl;
        //std::cout << position_cmd[k].data << std::endl;
    } 
    return;

}


int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "hebi_listener");
    ros::NodeHandle nh;
    //std::vector<ros::Subscriber> sub;
    ROS_INFO_STREAM("Node initialized.");
    // initialize hebi command
    // Try and get the requested group.
    std::unique_ptr<hebi::Group> group = getGroupFromArgs(argc, argv);
    if (!group)
    {
      printf("No group found!\n");
      return -1;
    }

    int num_modules = group->size();

    // Create a group feedback object; this will be filled in during the request.
    hebi::GroupFeedback feedback(num_modules);   
    ROS_INFO_STREAM("About to resize subscribers");
    //sub.resize(num_modules);
    ROS_INFO_STREAM("subscribers resized.");
    // Create a command object; this can be sent to the group
    hebi::GroupCommand command(num_modules);

    hebiSub hs(16);
    std::cout << "before sub" << hs.n_mod << std::endl;
    ros::Subscriber sub = nh.subscribe("/joint_position_cmd", 10, &hebiSub::hebiCallback, &hs);
    float period_s = 0.25f;
    long timeout_ms = 1000;
    while (ros::ok())
    {
        /* individual topics
        for (int i = 0; i < num_modules; i++)
        {
            char sub_name[40];
            if (i < 10) 
            {
                sprintf(sub_name,"/snake/S_0%d_eff_pos_controller/command",i);
            } else {
                sprintf(sub_name,"/snake/S_%d_eff_pos_controller/command",i);
            }
            sub[i] = nh.subscribe(sub_name, 1, boost::bind(hebiCallback, _2, i, command));
            ROS_INFO_STREAM(sub_name);
        } 
        */
        for (int i = 0; i < num_modules; i++)
        {   
            //std::cout << "command angle: "<< hs.position_cmd[i] << std::endl;
            command[i].actuator().position().set(hs.position_cmd[i].data);
            std::cout << hs.position_cmd[i].data << std::endl;
        } 

        if (group->sendCommandWithAcknowledgement(command, 100))
        {
            printf("Got acknowledgement.\n");
        } else {
            printf("Did not receive acknowledgement!\n");
        }
        hebi_sleep_ms(10.0);


        for (int i = 0; i < num_modules; i++)
        {
            if (group->requestFeedback(&feedback, timeout_ms))
                print_group_feedback(feedback);
            else
                printf("Received no feedback from group!\n");
            hebi_sleep_ms(period_s * 1000);
        }
        ros::spinOnce();
    }

    /**
    * ros::spin() will enter a loop, pumping callbacks.  With this version, all
    * callbacks will be called from within this thread (the main one).  ros::spin()
    * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    */


    return 0;
}
