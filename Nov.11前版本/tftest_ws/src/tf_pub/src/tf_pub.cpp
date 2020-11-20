#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Pose.h>
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"


float x = 0.2;
float y = 0.2;
float z = 0.2;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "VirtualTfPub");
	ros::NodeHandle node;
	ros::Rate rate(10000);  
	tf::TransformBroadcaster broadcaster;
	tf::Transform tf_send;
	while(node.ok())
	{	
		tf_send.setOrigin(tf::Vector3(x, y, z));
		tf_send.setRotation(tf::Quaternion(0, 0, 0, 1));
		broadcaster.sendTransform(tf::StampedTransform(tf_send, ros::Time(0), "/laser2_link", "pc"));
		/*if (x <=10)
		{
			x += 0.1;	
		}
*/
		rate.sleep();
	}
}
