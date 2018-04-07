#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf2_listener");
	ros::NodeHandle nh;
	ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
	
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Rate rate(100.0);
	
	// Path for left arm
	visualization_msgs::Marker path1;
	path1.ns = "trace";
	path1.header.frame_id = "world";
	path1.action = visualization_msgs::Marker::ADD;
	path1.pose.orientation.w = 1.0;
	path1.color.a = 1.0;
	path1.color.r = 1.0;
	path1.id = 0;
	path1.type = visualization_msgs::Marker::LINE_STRIP;
	path1.scale.x = 0.005;

	// Path for right arm
	visualization_msgs::Marker path2;
	path2.ns = "trace";
	path2.header.frame_id = "world";
	path2.action = visualization_msgs::Marker::ADD;
	path2.pose.orientation.w = 1.0;
	path2.color.a = 1.0;
	path2.color.b = 1.0;
	path2.id = 1;
	path2.type = visualization_msgs::Marker::LINE_STRIP;
	path2.scale.x = 0.005;

	while(nh.ok()) {
		// Get end effector position for left arm
		geometry_msgs::TransformStamped transformStamped1;
		try {
			transformStamped1 = tfBuffer.lookupTransform("world", "EE", ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		// Get end effector position for right arm
		geometry_msgs::TransformStamped transformStamped2;
		try {
			transformStamped2 = tfBuffer.lookupTransform("world", "arm2/EE", ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		geometry_msgs::Point point;
		point.x = transformStamped1.transform.translation.x;
		point.y = transformStamped1.transform.translation.y;
		point.z = transformStamped1.transform.translation.z;
		path1.points.push_back(point);
		path_pub.publish(path1);
		
		path1.header.stamp = ros::Time::now();
		if (path1.points.size() == 500) {
			path1.points.erase(path1.points.begin(), path1.points.begin()+1);
		}

		point.x = transformStamped2.transform.translation.x;
		point.y = transformStamped2.transform.translation.y;
		point.z = transformStamped2.transform.translation.z;
		path2.points.push_back(point);
		path_pub.publish(path2);
		
		path2.header.stamp = ros::Time::now();
		if (path2.points.size() == 500) {
			path2.points.erase(path2.points.begin(), path2.points.begin()+1);
		}

		rate.sleep();
	}
	return 0;
}
