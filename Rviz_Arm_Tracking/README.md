# Rviz_Arm_Tracking
Visual representation of the HEBI 5-DOF arm in Rviz with marker tracking for use with the Model_Learning repository

Authors: Ky Woodard (arm_rviz/arm_feedback) and Megan Yu (trace_path)

## Dependencies

## How to Run
- Make sure the arm is on and connected to the network
- Run the following command to verify that the modules are connecting and sending feedback 
  (the following warning is standard: "wrist" passed to lookupTransform argument target_frame does not exist)
```
  rosrun arm_feedback arm_feedback
```
- The code will seg fault in the first few seconds if it cannot find the modules. Otherwise, it will show "Modules Fully Initialized" if the connection was successful. If this does not show up, restart and try it again.
- If the modules are connecting, launch the following
 ```
  roslaunch arm_rviz imr_arm_rviz.launch
 ```
 - You should see Rviz open and a ghost arm show up that follows the motions of the real arm
 - In order to get the reference trajectory to show up, launch instead
 ```
  roslaunch arm_rviz imr_arm_rviz_track.launch
 ```
 - A dashed red trajectory should now be shown in Rviz as well (this trajectory can be modified in trace_path/src/ref_path.cpp)
 
