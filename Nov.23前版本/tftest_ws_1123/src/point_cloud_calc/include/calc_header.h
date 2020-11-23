#ifndef __CONVERT__
#define __CONVERT__

#include <ros/ros.h>
#include <cmath>
#include <math.h>
#include <fstream>
#include <iostream>
#include "csignal"
#include <map>
#include <numeric>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

//typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,sensor_msgs::Imu> SyncPolicy;
class Convert
{	
	public:
	Convert();
	~Convert();
	//void scanCallback(sensor_msgs::LaserScan scan);
	//void poseCallback(sensor_msgs::Imu imu);
	//void quaternionCallback(geometry_msgs::Vector3Stamped angle);
	void CombinedCallback(const sensor_msgs::LaserScan& scan, const sensor_msgs::Imu& imu);
	geometry_msgs::Pose get_position();
	tf::StampedTransform get_transform();
	tf::TransformBroadcaster broadcaster;
	tf::Transform tf_send;
	bool Tf_check;
	bool MotionFilter(geometry_msgs::Quaternion q_in);
	void finish(); 
	void get_floor(const sensor_msgs::PointCloud& pc_);
	int iteration;
	private:
	ros::NodeHandle node;
	ros::Publisher pub;
	ros::Subscriber sub;

	ros::Subscriber posesub;
	ros::Subscriber qsub;
	sensor_msgs::PointCloud pointcloud;
	sensor_msgs::PointCloud cloud_output,cloud_output_r,cloud_rviz,cloud_last;
	sensor_msgs::PointCloud2 pc2, pc2_last;
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	tf::TransformListener listener;                                                           
	//tf::TransformBroadcaster broadcaster;
	//tf::Transform tf_send;
	tf::StampedTransform transform;
	sensor_msgs::Imu imu_get;
	geometry_msgs::Pose pose;
	geometry_msgs::Quaternion q;
	geometry_msgs::Quaternion q_mid;
	laser_geometry::LaserProjection projector;
	
	double rotation_time;
	long int pointcloud_size;
	int point_num;
	long int current_size;
	int iterate_time; 
	double x,y,z;
	double Roll,Pitch;		//Eular angles calculated by the quaternion
	double Yaw = 0;
	double q0= 0;
	double q1 = 0;
	double q2 = 0;
	double q3 = 0;
	double floor, floor_sum = 0;
	double width_l, width_r;		//temp variable for doc the left and right width of single scan 
	double width_ls, width_rs;	//shortest left and right width 
	double W;					//width of the tunnel 
	double height_1, height_2, height_3, height_4, height_5, height_6, height_7, height_8, height_9, height_10; 
	double sum_temp1, sum_temp2, sum_temp3, sum_temp4, sum_temp5;
	int cnt_wl, cnt_wr, cnt_1, cnt_2, cnt_3, cnt_4, cnt_5, cnt_6, cnt_7, cnt_8, cnt_9, cnt_10;
	double *_length; 							//store width of each scan
	geometry_msgs::Quaternion *_qua;  		//store quaternion of each scan
	geometry_msgs::Quaternion *_delta_q; 
	double 	*_theta;							//store angles for widths
 	double _width_sum;
	double _width_final;
 	double _width_l_sum;
	double _width_l_final;
 	double _width_r_sum;
	double _width_r_final;
	double *_width_calc;
	double *_width_ls;
	double *_width_rs;
	double *_roll;
	double *_pitch;
	double qs_w = 0.707;
	double qs_x = 0.0;
	double qs_y = 0.707;
	double qs_z = 0.0;
	
};

void callback(const sensor_msgs::LaserScanConstPtr &scan, const sensor_msgs::ImuConstPtr &imu, Convert* &convertob_);
void signalHandler(int signum);
#endif
