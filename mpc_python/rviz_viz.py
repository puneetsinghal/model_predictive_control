import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# https://answers.ros.org/question/213075/publish-joint_state-with-python-to-rviz/


def talker(input_joints, task_space_way_points):
	rospy.init_node('joint_state_publisher_viz')
	pub = rospy.Publisher('joint_status', JointState, queue_size=10)
	way_pub = rospy.Publisher('ref_way_points', Marker, queue_size=10)


	# visualize the path in the form of waypoints

	robotMarker = Marker()
	robotMarker.header.frame_id = "world"
	robotMarker.header.stamp    = rospy.get_rostime()
	#robotMarker.ns = "robot"
	robotMarker.id = 1
	robotMarker.type = robotMarker.LINE_STRIP # sphere
	robotMarker.action = 0 #robotMarker.ADD
	# robotMarker.pose.orientation.x = 0
	# robotMarker.pose.orientation.y = 0
	# robotMarker.pose.orientation.z = 0
	# robotMarker.pose.orientation.w = 1.0
	robotMarker.scale.x = .05
	robotMarker.scale.y = .05
	robotMarker.scale.z = .05

	robotMarker.color.r = 1.0
	robotMarker.color.g = 0.0
	robotMarker.color.b = 0.0
	robotMarker.color.a = 1.0

	#robotMarker.lifetime = rospy.Duration()



	
	for i in xrange(len(task_space_way_points)):

		point = Point()
		point.x = task_space_way_points[i][0]
		point.y = task_space_way_points[i][1]
		point.z = task_space_way_points[i][2]

		
		robotMarker.points.append(point)

	

	way_pub.publish(robotMarker)
	print "Hellolllllllllllllll"




	# visualize the trajectory
	for inp in input_joints:
	    rate = rospy.Rate(0.5) # 10hz
	    hello_str = JointState()
	    hello_str.header = Header()
	    hello_str.header.stamp = rospy.Time.now()
	    hello_str.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4' , 'Joint5']
	    hello_str.position = inp
	    hello_str.velocity = []
	    hello_str.effort = []
	    pub.publish(hello_str)
	    way_pub.publish(robotMarker)
	    rate.sleep()

if __name__ == '__main__':
    try:
    	input_list = [[0,50.1,0,0,0], [0,150.1,0,0,0], [0,250.1,0,0,0]]
    	waypoints = [ [0,0,1], [0,1,0], [1,0,0],[0,0,1]]
    	talker(input_list, waypoints)
    except rospy.ROSInterruptException:
        pass


# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Header

# # https://answers.ros.org/question/213075/publish-joint_state-with-python-to-rviz/


# def talker(input_list):
#     pub = rospy.Publisher('joint_status', JointState, queue_size=10)
#     rospy.init_node('joint_state_publisher_viz')
#     rate = rospy.Rate(10) # 10hz
#     hello_str = JointState()
#     hello_str.header = Header()
#     hello_str.header.stamp = rospy.Time.now()
#     hello_str.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4' , 'Joint5']
#     input_list = [ 0, 100.1,0,0,0]
#     hello_str.position = input_list
#     hello_str.velocity = []
#     hello_str.effort = []
#     pub.publish(hello_str)
#     rate.sleep()

# if __name__ == '__main__':
#     try:
#     	# read from some numpy file or text file here you will modify the talker input argument accordingly
#     	input_list = [[0,0.1,0,0,0]]
#     	talker(input_list)
#     except rospy.ROSInterruptException:
#         pass