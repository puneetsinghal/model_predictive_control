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

	while(nh.ok()) {
		geometry_msgs::TransformStamped transformStamped;
		try {
			transformStamped = tfBuffer.lookupTransform("world", "EE", ros::Time(0));
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
		rate.sleep();
	}
	return 0;
}
