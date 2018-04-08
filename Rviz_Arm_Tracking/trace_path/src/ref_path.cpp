#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "ref_path");
	ros::NodeHandle nh;
	ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("ref_path", 10);

	ros::Rate rate(100.0);

	visualization_msgs::Marker path;
	path.ns = "trace";
	path.header.frame_id = "world";
	path.action = visualization_msgs::Marker::ADD;
	path.pose.orientation.w = 1.0;
	path.color.a = 1.0;
	path.color.r = 1.0;
	path.id = 0;
	path.type = visualization_msgs::Marker::LINE_LIST;
	path.scale.x = 0.005;

	double c_x, c_y, c_z, radius;
	c_x = 0;
	c_y = 0.3;
	c_z = -0.025;
	radius = 0.1;

	double N = 100;

	for(int i=0;i<N;i++)
	{
		geometry_msgs::Point point;

		point.x = c_x+radius*std::sin(2*M_PI*i/N);
		point.y = c_y+radius*std::cos(2*M_PI*i/N);
		point.z = c_z;
		path.points.push_back(point);
	}

	while(nh.ok()) {
		path.header.stamp = ros::Time::now();
		path_pub.publish(path);
		rate.sleep();
	}
	return 0;
}
