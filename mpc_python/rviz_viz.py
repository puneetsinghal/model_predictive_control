import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# https://answers.ros.org/question/213075/publish-joint_state-with-python-to-rviz/

rospy.init_node('view',
                anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

#group = moveit_commander.MoveGroupCommander("arm")


display_trajectory_publisher = rospy.Publisher(
                                    '/path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)


display_trajectory = moveit_msgs.msg.DisplayTrajectory()

print ("############################")
display_trajectory.trajectory_start = robot.get_current_state()
#	display_trajectory.trajectory.append(plan1)
display_trajectory_publisher.publish(display_trajectory);
