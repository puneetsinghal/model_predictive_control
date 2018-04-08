#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

class subscriberClass
{
public:
	subscriberClass() : tfListener(this->tfBuffer)
	{
		std::string ns = ros::this_node::getNamespace();
		this->path_pub = this->nh.advertise<visualization_msgs::Marker>(ns+"/path_trace", 10);
		this->color_sub = this->nh.subscribe(ns+"/color",1,&subscriberClass::subscriberCallback, this);

		this->path.ns = "trace";
		this->path.header.frame_id = "world";
		this->path.action = visualization_msgs::Marker::ADD;
		this->path.pose.orientation.w = 1.0;
		
		this->path.id = 0;
		this->path.type = visualization_msgs::Marker::LINE_STRIP;
		this->path.scale.x = 0.005;
		this->path.scale.y = 0.005;
		this->path.scale.z = 0.005;
	}

	void subscriberCallback(const std_msgs::ColorRGBA& color_msg)
	{
		geometry_msgs::TransformStamped transformStamped;
		try {
			transformStamped = tfBuffer.lookupTransform("world", "EE", ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		this->path.color.r = color_msg.r;
		this->path.color.g = color_msg.g;
		this->path.color.b = color_msg.b;
		this->path.color.a = color_msg.a;

		geometry_msgs::Point point;
		point.x = transformStamped.transform.translation.x;
		point.y = transformStamped.transform.translation.y;
		point.z = transformStamped.transform.translation.z;
		this->path.points.push_back(point);
		this->path_pub.publish(path);
	}
private:
	ros::NodeHandle nh;
	ros::Publisher path_pub;
	ros::Subscriber color_sub;
	visualization_msgs::Marker path;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "trace_path_interval");


	subscriberClass subClass;

	ros::spin();

	return 0;
}
