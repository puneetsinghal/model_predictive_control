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
import numpy as np

# https://answers.ros.org/question/213075/publish-joint_state-with-python-to-rviz/


def talker(input_joints, ref, waypoints): #task_space_way_points are ref points
	rospy.init_node('joint_state_publisher_viz')
	pub = rospy.Publisher('joint_status', JointState, queue_size=10)


	# visualize the path in the form of waypoints
	way_pub = rospy.Publisher('ref_way_points', MarkerArray, queue_size=10)

	markers = []
	for i in xrange(len(ref)):

		point = Point()
		#print ref[i]
		point.x = ref[i][0]
		point.y = ref[i][1]
		point.z = ref[i][2]

		robotMarker = Marker()
		robotMarker.header.frame_id = "world"
		robotMarker.header.stamp    = rospy.get_rostime()
		#robotMarker.ns = "robot"
		robotMarker.id = i
		robotMarker.type = 2# robotMarker.LINE_STRIP # sphere
		robotMarker.action = robotMarker.ADD
		robotMarker.pose.position = point
		robotMarker.pose.orientation.x = 0
		robotMarker.pose.orientation.y = 0
		robotMarker.pose.orientation.z = 0
		robotMarker.pose.orientation.w = 1.0
		robotMarker.scale.x = .025
		robotMarker.scale.y = .025
		robotMarker.scale.z = .025

		robotMarker.color.r = 1.0
		robotMarker.color.g = 0.0
		robotMarker.color.b = 0.0
		robotMarker.color.a = 1.0

		robotMarker.lifetime = rospy.Duration()
		markers.append(robotMarker)

	markerArray = MarkerArray()
	markerArray.markers = markers

	way_pub.publish(markerArray)
	


	way_pub2 = rospy.Publisher('ref_way_points2', Marker, queue_size=10)
	robotMarker2 = Marker()
	robotMarker2.header.frame_id = "world"
	robotMarker2.header.stamp    = rospy.get_rostime()
	robotMarker2.id = 1
	robotMarker2.type = robotMarker2.LINE_STRIP # sphere
	robotMarker2.action = 0 #robotMarker.ADD
	robotMarker2.scale.x = .01
	robotMarker2.scale.y = .01
	robotMarker2.scale.z = .01

	robotMarker2.color.r = 0.0
	robotMarker2.color.g = 1.0
	robotMarker2.color.b = 0.0
	robotMarker2.color.a = 1.0


	# visualize the trajectory
	for i in range(len(input_joints)):
	    rate = rospy.Rate(3.5) # 10hz
	    hello_str = JointState()
	    hello_str.header = Header()
	    hello_str.header.stamp = rospy.Time.now()
	    hello_str.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4' , 'Joint5']
	    hello_str.position = input_joints[i]
	    hello_str.velocity = []
	    hello_str.effort = []
	    pub.publish(hello_str)
	    way_pub.publish(markerArray)

	    point = Point()
	    point.x = waypoints[i][0]
	    point.y = waypoints[i][1]
	    point.z = waypoints[i][2]
	    robotMarker2.points.append(point)
	    way_pub2.publish(robotMarker2)

	    rate.sleep()

if __name__ == '__main__':
    try:
    	temp_j = np.load('1234.npy')
    	temp_j = np.resize(temp_j, (temp_j.size/5,5))
    	temp_j1 = temp_j.tolist()
    	input_list = [ x + [0] for x in temp_j1]

    	temp = np.load('xyzt.npy')
    	temp = np.resize(temp, (temp.size/3, 3))
    	temp = temp.tolist()
    	waypoints = [ [x[0],x[1], x[2]+ 0.04] for x in temp]

    	temp = np.load('ref.npy')
    	temp = temp.tolist()

    	
    	
    	refpoints = map(list, zip(*temp))

    	refpoints = []

    	i_s = [0,10,30,40,50, 60, 65, 68, 71, 80,90 ,100, 110, 120, 123,126, 129, 132, 142, 152, 160]

    	print (len(i_s))

    	for i in i_s:
    		
    		refpoints.append(waypoints[i])

    	#waypoints = np.a
    	#input_list = [[0,50.1,0,0,0], [0,150.1,0,0,0], [0,250.1,0,0,0]]
    	#waypoints = [ [0,0,1], [0,1,0], [1,0,0],[0,0,1]]
    	talker(input_list, refpoints , waypoints)
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