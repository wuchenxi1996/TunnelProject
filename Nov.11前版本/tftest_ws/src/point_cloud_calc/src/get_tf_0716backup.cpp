#include "get_tf.h"
#include <stdlib.h>
geometry_msgs::Pose Convert::get_position() // get robot pose
{
	geometry_msgs::TransformStamped transform_pose;
	tf::transformStampedTFToMsg(transform, transform_pose);
	pose.position.x = transform.getOrigin().x();
	pose.position.y = transform.getOrigin().y();
	pose.position.z = transform.getOrigin().z();
	pose.orientation = transform_pose.transform.rotation;
	return pose;
}

tf::StampedTransform Convert::get_transform() //get transform object
{
	try
	{
	listener.waitForTransform("/base_link", "/pc", ros::Time::now(), ros::Duration(3.0));
	listener.lookupTransform("/base_link", "/pc", ros::Time::now(), transform);
	Tf_check = true;
	}
	catch(tf::TransformException &ex)
	{
	ROS_ERROR("%s", ex.what());
	ros::Duration(1.0).sleep();
	Tf_check = false;
	}

	return transform;
}

Convert::Convert()
{
	pub = node.advertise<sensor_msgs::PointCloud>("/Cloud", 1000);
	sub = node.subscribe("/scan2", 1, &Convert::scanCallback, this);
	posesub = node.subscribe("/Imu_path", 1, &Convert::poseCallback, this);
	qsub = node.subscribe("/imu/rpy/filtered", 1, &Convert::quaternionCallback, this);
	
	point_num = 720;
	cloud_output.points.resize(324000);
	cloud_output_n.points.resize(324000);
	cloud_output_r.points.resize(324000);
	cloud_output.header.frame_id = "base_link";
	cloud_output_n.header.frame_id = "base_link";
	current_size = 0;
	iterate_time = 1;
	width_l = 0.0;
	width_r = 0.0;
	cnt_wl = cnt_wr = cnt_l1 = cnt_l2 = cnt_l3 = cnt_l4 = cnt_mid = cnt_r1 = cnt_r2 = cnt_r3 = cnt_r4 =0;
	sum_temp1 = sum_temp2 = sum_temp3 = sum_temp4 = sum_temp5 = 0.0;
	height_l1 = height_l2 = height_l3 = height_l4 = height_mid = height_r1 = height_r2 = height_r3 = height_r4 = 0.0;
	W = 20;
	std::cout << "Initialization Complete." << std::endl;
}

Convert::~Convert()
{

}

void Convert::scanCallback(sensor_msgs::LaserScan scan)
{	
	//listener.waitForTransform("/laser_link2", "/pc", /*scan.header.stamp*/ros::Time(0), ros::Duration(3.0));
	//listener.lookupTransform("/laser_link", "/pc", ros::Time(0), transform);
	//std::cout << "Transform got." << std::endl;
	projector.transformLaserScanToPointCloud("/base_link", scan, pointcloud, listener);
	double q0,q1,q2,q3;
	q0 = pose.orientation.w;
	q1 = pose.orientation.x;
	q2 = pose.orientation.y;
	q3 = pose.orientation.z;
	cnt_wl = cnt_wr = 0;
	sum_temp1 = sum_temp2 = sum_temp3 = sum_temp4 = sum_temp5 = 0.0;
	yaw = yaw ;
/* Take out the isolated points */
/*
	for(int j = 0; j < point_num; ++j)
	{
		if(pointcloud.points[j].y >= 5.0 | pointcloud.points[j].x >= 5.0 | pointcloud.points[j].z >= 5.0)
		{
			pointcloud.points[j].x = pointcloud.points[j-1].x;
			pointcloud.points[j].y = pointcloud.points[j-1].y;
			pointcloud.points[j].z = pointcloud.points[j-1].z;		
		}		
	}
*/
/*---------Caculate the width of the pointcloud----------*/


	for(int k = 0;k < point_num; ++k)
	{
		if((pointcloud.points[k].z >  0.10) && (pointcloud.points[k].z < 0.15) && (pointcloud.points[k].y > 0))
		{
			sum_temp1 += pointcloud.points[k].y;
			++cnt_wl; 
		}
		else if((pointcloud.points[k].z >  0.10) && (pointcloud.points[k].z < 0.15) && (pointcloud.points[k].y < 0))
		{
			sum_temp2 += pointcloud.points[k].y;
			++cnt_wr; 	
		}
	}

	width_l = sum_temp1 / cnt_wl;
	width_r = sum_temp2 / cnt_wr;
	if(W > width_l - width_r)
	{
		W = width_l - width_r;
		q_temp.w = pose.orientation.w;
		q_temp.x = -pose.orientation.x;
		q_temp.y = -pose.orientation.y;
		q_temp.z = -pose.orientation.z;
		width_ls = width_l;
		width_rs = width_r;
	}
	std::cout << "diff = " << width_l - width_r << " " << " W = " << W << std::endl;	


/*------------Assemble the pointcloud into a big single one------------*/

	if (current_size <= 323280)
	{
		for(int i = 0; i < point_num; ++i)
		{
		/*------------Calc the new rotation center-------------*/	
			if(yaw > 0 && yaw < 1.57079)
			{				
				x = 0.08 * cos(yaw);
				y = 0.08 * sin(yaw);
			}
			else if(yaw >= 1.57079)
			{
				x = -0.08 * cos(yaw);
				y = -0.08 * sin(yaw);
			}
			else if(yaw < 0 && yaw > -1.57079)
			{
				x = 0.08 * cos(yaw);
				y = 0.08 * sin(yaw); 
			}
			else if(yaw <= -1.57079)
			{
				
				y = 0.08 * sin(yaw);
				x = 0.08 * cos(yaw);
			}
			cloud_output.header.stamp = ros::Time::now();
		
			cloud_output.points[i + current_size].x = (1-2*q2*q2-2*q3*q3)*(pointcloud.points[i].x) + \
													  (2*q1*q2-2*q0*q3)*pointcloud.points[i].y + \
													  (2*q1*q3+2*q0*q2)*pointcloud.points[i].z  ;
			cloud_output.points[i + current_size].y = (2*q1*q2+2*q0*q3)*(pointcloud.points[i].x) + \
													  (1-2*q1*q1-2*q3*q3)*pointcloud.points[i].y + \
													  (2*q2*q3-2*q0*q1)*pointcloud.points[i].z ;
			cloud_output.points[i + current_size].z = (2*q1*q3-2*q0*q2)*(pointcloud.points[i].x) + \
													  (2*q2*q3+2*q0*q1)*pointcloud.points[i].y + \
													  (1-2*q1*q1-2*q2*q2)*pointcloud.points[i].z;
			cloud_output_n.points[i + current_size].x = pointcloud.points[i].x ;
			cloud_output_n.points[i + current_size].y = pointcloud.points[i].y ;
			cloud_output_n.points[i + current_size].z = pointcloud.points[i].z ;
		}

	current_size = current_size + point_num;	
	
	}
	else 
	{
		std::cout << "Reached maximum point number" << std::endl;

		for(int n = 0;n <= current_size; ++n)
		{
			cloud_output_r.points[n].x = (1-2*q_temp.y*q_temp.y-2*q_temp.z*q_temp.z)*cloud_output.points[n].x + \
									     (2*q_temp.x*q_temp.y-2*q_temp.w*q_temp.z)*cloud_output.points[n].y + \
									     (2*q_temp.x*q_temp.z+2*q_temp.w*q_temp.y)*cloud_output.points[n].z ;
			cloud_output_r.points[n].y = (2*q_temp.x*q_temp.y+2*q_temp.w*q_temp.z)*cloud_output.points[n].x + \
									     (1-2*q_temp.x*q_temp.x-2*q_temp.z*q_temp.z)*cloud_output.points[n].y + \
										 (2*q_temp.y*q_temp.z-2*q_temp.w*q_temp.x)*cloud_output.points[n].z ;
			cloud_output_r.points[n].z = (2*q_temp.x*q_temp.z-2*q_temp.w*q_temp.y)*cloud_output.points[n].x + \
						    			 (2*q_temp.y*q_temp.z+2*q_temp.w*q_temp.x)*cloud_output.points[n].y + \
									     (1-2*q_temp.x*q_temp.x-2*q_temp.y*q_temp.y)*cloud_output.points[n].z;

			/*----------Calculate the height of 9 points----------*/
			if(cloud_output_n.points[n].y >= width_ls/5-0.05 && cloud_output_n.points[n].y <= width_ls/5+0.05 && cloud_output_n.points[n].z > 0)
			{
				height_l4 += cloud_output_n.points[n].z;
				++ cnt_l4;
			}
			else if(cloud_output_n.points[n].y >= 2*width_ls/5-0.05 && cloud_output_n.points[n].y <= 2*width_ls/5+0.05 && cloud_output_n.points[n].z > 0)
			{
				height_l3 += cloud_output_n.points[n].z;
				++ cnt_l3;
			}
			else if(cloud_output_n.points[n].y >= 3*width_ls/5-0.05 && cloud_output_n.points[n].y <= 3*width_ls/5+0.05 && cloud_output_n.points[n].z > 0)
			{
				height_l2 += cloud_output_n.points[n].z;
				++ cnt_l2;
			}
			else if(cloud_output_n.points[n].y >= 4*width_ls/5-0.05 && cloud_output_n.points[n].y <= 4*width_ls/5+0.05 && cloud_output_n.points[n].z > 0)
			{
				height_l1 += cloud_output_n.points[n].z;
				++ cnt_l1;
			}
			else if(cloud_output_n.points[n].y >= -0.05 && cloud_output_n.points[n].y <= 0.05 && cloud_output_n.points[n].z > 0)
			{
				height_mid += cloud_output_n.points[n].z;
				++ cnt_mid;
			}
			else if(cloud_output_n.points[n].y >= width_rs/5-0.05 && cloud_output_n.points[n].y <= width_rs/5+0.05 && cloud_output_n.points[n].z > 0)
			{
				height_r4 += cloud_output_n.points[n].z;
				++ cnt_r4;
			}
			else if(cloud_output_n.points[n].y >= 2*width_rs/5-0.05 && cloud_output_n.points[n].y <= 2*width_rs/5+0.05 && cloud_output_n.points[n].z > 0)
			{
				height_r3 += cloud_output_n.points[n].z;
				++ cnt_r3;
			}
			else if(cloud_output_n.points[n].y >= 3*width_rs/5-0.05 && cloud_output_n.points[n].y <= 3*width_rs/5+0.05 && cloud_output_n.points[n].z > 0)
			{
				height_r2 += cloud_output_n.points[n].z;
				++ cnt_r2;
			}
			else if(cloud_output_n.points[n].y >= 4*width_rs/5-0.05 && cloud_output_n.points[n].y <= 4*width_rs/5+0.05 && cloud_output_n.points[n].z > 0)
			{
				height_r1 += cloud_output_n.points[n].z;
				++ cnt_r1;
			}
		}
		height_l1 = height_l1 / cnt_l1;
		height_l2 = height_l2 / cnt_l2;
		height_l3 = height_l3 / cnt_l3;
		height_l4 = height_l4 / cnt_l4;
		height_mid = height_mid / cnt_mid;
		height_r1 = height_r1 / cnt_r1;
		height_r2 = height_r2 / cnt_r2;
		height_r3 = height_r3 / cnt_r3;
		height_r4 = height_r4 / cnt_r4;
		std::cout << width_ls << " " << width_rs << std::endl;
 		std::cout << height_l1 << " " << height_l2 << " " << height_l3 <<  " " << height_l4 \
				  << height_mid << " " << height_r1 << " " << height_r2 << " " << height_r3 << " " << height_r4 << std::endl;
		sensor_msgs::convertPointCloudToPointCloud2(cloud_output_r, pc2);
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl::fromROSMsg(pc2, pcl_cloud);
		pcl::io::savePCDFileASCII ("/home/pibot/cloud.pcd", pcl_cloud);
		ros::shutdown();
			
	}	
	++iterate_time;
	
	//std::cout << "cloud_output:x" << cloud_output.points[current_size-1].x << std::endl;
	pub.publish(cloud_output);
	//std::cout << pose.position.x << std::endl;
	//std::cout << hd.header.stamp << std::endl;

}

void Convert::poseCallback(nav_msgs::Odometry odom)
{
	pose.position.x = odom.pose.pose.position.x;
	pose.position.y = odom.pose.pose.position.y;
	pose.position.z = odom.pose.pose.position.z;
	pose.orientation = odom.pose.pose.orientation;
	hd.header.stamp = odom.header.stamp;
}

void Convert::quaternionCallback(geometry_msgs::Vector3Stamped angle)
{
	roll = angle.vector.x;
	pitch = angle.vector.y;
	yaw = angle.vector.z;
}

