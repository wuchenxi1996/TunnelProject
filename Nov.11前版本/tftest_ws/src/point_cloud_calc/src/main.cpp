#include <stdlib.h>
#include <ros/ros.h>
#include "calc_header.h"
#include <math.h>
#include <malloc.h>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
float x = 0.0;
float y = 0.0;
float z = 0.0;
using namespace message_filters;
static volatile int keepRunning = 1;

void sig_handler(int sig)
{
	if (sig == SIGINT)
	{
		keepRunning = 0;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "PointCloudConvert");
	ros::NodeHandle node;

	tf::StampedTransform tf; 
	Convert* convertObject = new Convert();
	//node.getParam("time", convertObject->rotation_time)
 	//signal(SIGINT, sig_handler);
	message_filters::Subscriber<sensor_msgs::LaserScan> scan_(node,"/scan",100);
	message_filters::Subscriber<sensor_msgs::Imu> imu_sub_(node,"/ekf",100);
	 typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Imu> syncPolicy;
	message_filters::Synchronizer<syncPolicy> sync(syncPolicy(20), scan_, imu_sub_);  
	sync.registerCallback(boost::bind(&callback, _1, _2,convertObject));
 	ros::spin();
	
	//convertObject->finish();
	return 0;
}


