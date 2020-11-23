#include "calc_header.h"
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
	std::cout << "Please input  maximum time for Sampling: " << std::endl;
	std::cin >> rotation_time;
	pointcloud_size = 14380 * rotation_time;//14380
	pub = node.advertise<sensor_msgs::PointCloud>("/Cloud", 100);
	point_num = 719;//719
	cloud_output.points.resize(pointcloud_size);
	cloud_output_r.points.resize(pointcloud_size);
	cloud_output.header.frame_id = "base_link";
	cloud_output_r.header.frame_id = "base_link";
	current_size = 0;
	iterate_time = 0;
	width_l = width_r = 0.0;
	Roll = Pitch = Yaw = 0.0;
	cnt_wl = cnt_wr = cnt_1 = cnt_2 = cnt_3 = cnt_4 = cnt_5 = cnt_6 = cnt_7 = cnt_8 = cnt_9 = cnt_10 = 0;
	sum_temp1 = sum_temp2 = sum_temp3 = sum_temp4 = sum_temp5 = 0.0;
	height_1 = height_2 = height_3 = height_4 = height_5 = height_6 = height_7 = height_8 = height_9 = height_10 = 0.0;
	W = 100;
	std::cout << "Initialization Complete." << std::endl;
}

Convert::~Convert()
{

}
void callback(const sensor_msgs::LaserScanConstPtr &scan, const sensor_msgs::ImuConstPtr &imu, Convert* &convertob_)
{
	convertob_->CombinedCallback(*scan, *imu);   
}

void Convert::CombinedCallback(const sensor_msgs::LaserScan& scan, const sensor_msgs::Imu& imu)
{
	q0 = imu.orientation.w;
	q1 = imu.orientation.x;
	q2 = imu.orientation.y;
	q3 = imu.orientation.z;
	//Yaw += 0.00667;
	projector.transformLaserScanToPointCloud("/base_link", scan, pointcloud, listener);
	for(int k = 0;k < point_num; ++k)
	{	
		pointcloud.points[k].x -= 0.1;
		if((pointcloud.points[k].z > 1.0) && (pointcloud.points[k].z < 1.2) && (pointcloud.points[k].y > 0))
		{
			sum_temp1 += pointcloud.points[k].y;
			++cnt_wl; 
		}
		else if((pointcloud.points[k].z > 1.0) && (pointcloud.points[k].z < 1.2) && (pointcloud.points[k].y < 0))
		{
			sum_temp2 += pointcloud.points[k].y;
			++cnt_wr; 	
		} 
	}
	width_l = sum_temp1 / cnt_wl;
	width_r = sum_temp2 / cnt_wr;
	cnt_wl = cnt_wr = sum_temp1 = sum_temp2 = 0;
	if(width_l - width_r < W)
	{
		W = width_l - width_r;
		q_temp.w = imu.orientation.w;
		q_temp.x = -imu.orientation.x;
		q_temp.y = -imu.orientation.y;
		q_temp.z = -imu.orientation.z;
		width_ls = width_l;
		width_rs = width_r;
	}

	//std::cout << width_l << "  " << width_r << std::endl;
	std::cout << "W = "  << W << std::endl;
	
	if (current_size < pointcloud_size-719) //719
	{
		for(int i = 0; i < point_num; ++i)
		{ 
		/*------------Calc the actrual rotation center of pointcloud-------------*/	
			
			cloud_output.points[i + current_size].x = (1-2*q2*q2-2*q3*q3)*pointcloud.points[i].x + \
													  (2*q1*q2-2*q0*q3)*pointcloud.points[i].y + \
													  (2*q1*q3+2*q0*q2)*pointcloud.points[i].z ;
			cloud_output.points[i + current_size].y = (2*q1*q2+2*q0*q3)*pointcloud.points[i].x + \
													  (1-2*q1*q1-2*q3*q3)*pointcloud.points[i].y + \
													  (2*q2*q3-2*q0*q1)*pointcloud.points[i].z ;
			cloud_output.points[i + current_size].z = (2*q1*q3 - 2*q0*q2) * pointcloud.points[i].x + \
						    			 		      (2*q2*q3 + 2*q0*q1) * pointcloud.points[i].y + \
									  			      (1 - 2*q1*q1 - 2*q2*q2) * pointcloud.points[i].z;	
		}
		
		current_size += point_num;	
		iterate_time++;
	}
	else 
	{	
		std::cout << "Reached maximum point number" << std::endl;
		/*----------Rotate the pointclouds to the orthogonal axis----------*/
		
		for(int n = 0;n <= current_size; ++n)
		{
			cloud_output_r.points[n].x = (1 - 2*q_temp.y*q_temp.y - 2*q_temp.z*q_temp.z) * cloud_output.points[n].x + \
									     (2*q_temp.x*q_temp.y - 2*q_temp.w*q_temp.z) * cloud_output.points[n].y + \
									     (2*q_temp.x*q_temp.z + 2*q_temp.w*q_temp.y) * cloud_output.points[n].z ;
			cloud_output_r.points[n].y = (2*q_temp.x*q_temp.y + 2*q_temp.w*q_temp.z) * cloud_output.points[n].x + \
									     (1 - 2*q_temp.x*q_temp.x - 2*q_temp.z*q_temp.z) * cloud_output.points[n].y + \
										 (2*q_temp.y*q_temp.z - 2*q_temp.w*q_temp.x) * cloud_output.points[n].z ;
			cloud_output_r.points[n].z = (2*q_temp.x*q_temp.z - 2*q_temp.w*q_temp.y) * cloud_output.points[n].x + \
						    			 (2*q_temp.y*q_temp.z + 2*q_temp.w*q_temp.x) * cloud_output.points[n].y + \
									     (1 - 2*q_temp.x*q_temp.x - 2*q_temp.y*q_temp.y) * cloud_output.points[n].z;
		}
		double w = W / 11;
		for(int n = 0;n <= current_size; ++n)
		{	/*----------Calculate the height of 9 points----------*/

			if(cloud_output_r.points[n].y >= width_rs+w-0.1 && cloud_output_r.points[n].y <= width_rs+w+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{	
					height_1 += cloud_output_r.points[n].z;
					++ cnt_1;
				}
			}
			else if(cloud_output_r.points[n].y >= width_rs+2*w-0.1 && cloud_output_r.points[n].y <= width_rs+2*w+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_2 += cloud_output_r.points[n].z;
					++ cnt_2;
				}
			}
			else if(cloud_output_r.points[n].y >= 3*w+width_rs-0.1 && cloud_output_r.points[n].y <= 3*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_3 += cloud_output_r.points[n].z;
					++ cnt_3;
				}
			}
			else if(cloud_output_r.points[n].y >= 4*w+width_rs-0.1 && cloud_output_r.points[n].y <= 4*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_4 += cloud_output_r.points[n].z;
					++ cnt_4;
				}
			}
			else if(cloud_output_r.points[n].y >= 5*w+width_rs-0.1 && cloud_output_r.points[n].y <= 5*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_5 += cloud_output_r.points[n].z;
					++ cnt_5;
				}
			}
			else if(cloud_output_r.points[n].y >= 6*w+width_rs-0.1 && cloud_output_r.points[n].y <= 6*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_6 += cloud_output_r.points[n].z;
					++ cnt_6;
				}
			}
			else if(cloud_output_r.points[n].y >= 7*w+width_rs-0.1 && cloud_output_r.points[n].y <= 7*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_7 += cloud_output_r.points[n].z;
					++ cnt_7;
				}
			}
			else if(cloud_output_r.points[n].y >= 8*w+width_rs-0.1 && cloud_output_r.points[n].y <= 8*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
						height_8 += cloud_output_r.points[n].z;
					++ cnt_8;
				}
			}
			else if(cloud_output_r.points[n].y >= 9*w+width_rs-0.1 && cloud_output_r.points[n].y <= 9*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_9 += cloud_output_r.points[n].z;
					++ cnt_9;
				}
			}
			else if(cloud_output_r.points[n].y >= 10*w+width_rs-0.1 && cloud_output_r.points[n].y <= 10*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_10 += cloud_output_r.points[n].z;
					++ cnt_10;
				}
			}
		}
		height_1 = height_1 / cnt_1;
		height_2 = height_2 / cnt_2;
		height_3 = height_3 / cnt_3;
		height_4 = height_4 / cnt_4;
		height_5 = height_5 / cnt_5;
		height_6 = height_6 / cnt_6;
		height_7 = height_7 / cnt_7;
		height_8 = height_8 / cnt_8;
		height_9 = height_9 / cnt_9;
		height_10 = height_10 / cnt_10;
		W = width_ls - width_rs;
/*----------Output to screen & Save to txt file----------*/
		std::cout << "Left width is " <<width_ls << "  Right width is " << width_rs << std::endl;
		std::cout << "Width of the tunnel is " << W << std::endl;
 		std::cout << height_1 << " " << height_2 << " " << height_3 << " " << height_4 << " " << height_5 << " " 
				  << height_6 << " " << height_7 << " " << height_8 << " " << height_9 << " " << height_10 << std::endl;
		std::ofstream out;
		out.open("/home/pibot/Wuchenxi_data/Measurement.txt",std::ios::in|std::ios::out|std::ios::binary);
		if (out.is_open())
		{
			out << width_r << "   " << width_l << " m.\n";
			out << "Height_1 of the tunnel is " << height_1 << " m.\n";
			out << "Height_2 of the tunnel is " << height_2 << " m.\n";
			out << "Height_3 of the tunnel is " << height_3 << " m.\n";
			out << "Height_4 of the tunnel is " << height_4 << " m.\n";
			out << "Height_5 of the tunnel is " << height_5 << " m.\n";
			out << "Height_6 of the tunnel is " << height_6 << " m.\n";
			out << "Height_7 of the tunnel is " << height_7 << " m.\n";
			out << "Height_8 of the tunnel is " << height_8 << " m.\n";
			out << "Height_9 of the tunnel is " << height_9 << " m.\n";
			out << "Height_10 of the tunnel is " << height_10 << " m.\n";
		}
		
		sensor_msgs::convertPointCloudToPointCloud2(cloud_output_r, pc2);
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl::fromROSMsg(pc2, pcl_cloud);
		pcl::io::savePCDFileASCII ("/home/pibot/Wuchenxi_data/cloud.pcd", pcl_cloud);
		ros::shutdown();
	}	
	pub.publish(cloud_output);
}

bool Convert::MotionFilter(geometry_msgs::Quaternion q_in)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(q_in, quat);
	double R_new, P_new, Y_new;
	tf::Matrix3x3(quat).getRPY(R_new, P_new, Y_new);
	if (abs(Y_new - Yaw) <= 0.00)
	{
		return false;
	}
	else 
	{
		Roll = R_new;
		Pitch = P_new;
		Yaw = Y_new;
		return true;
	}
}

void Convert::finish()
{
	std::cout << "Process stopped mannually." << std::endl;
		/*----------Rotate the pointclouds to the orthogonal axis----------*/
		
		for(int n = 0;n <= current_size; ++n)
		{
			cloud_output_r.points[n].x = (1 - 2*q_temp.y*q_temp.y - 2*q_temp.z*q_temp.z) * cloud_output.points[n].x + \
									     (2*q_temp.x*q_temp.y - 2*q_temp.w*q_temp.z) * cloud_output.points[n].y + \
									     (2*q_temp.x*q_temp.z + 2*q_temp.w*q_temp.y) * cloud_output.points[n].z ;
			cloud_output_r.points[n].y = (2*q_temp.x*q_temp.y + 2*q_temp.w*q_temp.z) * cloud_output.points[n].x + \
									     (1 - 2*q_temp.x*q_temp.x - 2*q_temp.z*q_temp.z) * cloud_output.points[n].y + \
										 (2*q_temp.y*q_temp.z - 2*q_temp.w*q_temp.x) * cloud_output.points[n].z ;
			cloud_output_r.points[n].z = (2*q_temp.x*q_temp.z - 2*q_temp.w*q_temp.y) * cloud_output.points[n].x + \
						    			 (2*q_temp.y*q_temp.z + 2*q_temp.w*q_temp.x) * cloud_output.points[n].y + \
									     (1 - 2*q_temp.x*q_temp.x - 2*q_temp.y*q_temp.y) * cloud_output.points[n].z;
		}
		double w = W / 11;
		for(int n = 0;n <= current_size; ++n)
		{	/*----------Calculate the height of 9 points----------*/

			if(cloud_output_r.points[n].y >= width_rs+w-0.1 && cloud_output_r.points[n].y <= width_rs+w+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{	
					height_1 += cloud_output_r.points[n].z;
					++ cnt_1;
				}
			}
			else if(cloud_output_r.points[n].y >= width_rs+2*w-0.1 && cloud_output_r.points[n].y <= width_rs+2*w+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_2 += cloud_output_r.points[n].z;
					++ cnt_2;
				}
			}
			else if(cloud_output_r.points[n].y >= 3*w+width_rs-0.1 && cloud_output_r.points[n].y <= 3*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_3 += cloud_output_r.points[n].z;
					++ cnt_3;
				}
			}
			else if(cloud_output_r.points[n].y >= 4*w+width_rs-0.1 && cloud_output_r.points[n].y <= 4*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_4 += cloud_output_r.points[n].z;
					++ cnt_4;
				}
			}
			else if(cloud_output_r.points[n].y >= 5*w+width_rs-0.1 && cloud_output_r.points[n].y <= 5*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_5 += cloud_output_r.points[n].z;
					++ cnt_5;
				}
			}
			else if(cloud_output_r.points[n].y >= 6*w+width_rs-0.1 && cloud_output_r.points[n].y <= 6*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_6 += cloud_output_r.points[n].z;
					++ cnt_6;
				}
			}
			else if(cloud_output_r.points[n].y >= 7*w+width_rs-0.1 && cloud_output_r.points[n].y <= 7*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_7 += cloud_output_r.points[n].z;
					++ cnt_7;
				}
			}
			else if(cloud_output_r.points[n].y >= 8*w+width_rs-0.1 && cloud_output_r.points[n].y <= 8*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
						height_8 += cloud_output_r.points[n].z;
					++ cnt_8;
				}
			}
			else if(cloud_output_r.points[n].y >= 9*w+width_rs-0.1 && cloud_output_r.points[n].y <= 9*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_9 += cloud_output_r.points[n].z;
					++ cnt_9;
				}
			}
			else if(cloud_output_r.points[n].y >= 10*w+width_rs-0.1 && cloud_output_r.points[n].y <= 10*w+width_rs+0.1 && cloud_output_r.points[n].z > 1.0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_10 += cloud_output_r.points[n].z;
					++ cnt_10;
				}
			}
		}
		height_1 = height_1 / cnt_1;
		height_2 = height_2 / cnt_2;
		height_3 = height_3 / cnt_3;
		height_4 = height_4 / cnt_4;
		height_5 = height_5 / cnt_5;
		height_6 = height_6 / cnt_6;
		height_7 = height_7 / cnt_7;
		height_8 = height_8 / cnt_8;
		height_9 = height_9 / cnt_9;
		height_10 = height_10 / cnt_10;
		W = width_ls - width_rs;
/*----------Output to screen & Save to txt file----------*/
		std::cout << "Left width is " <<width_ls << "  Right width is " << width_rs << std::endl;
		std::cout << "Width of the tunnel is " << W << std::endl;
 		std::cout << height_1 << " " << height_2 << " " << height_3 << " " << height_4 << " " << height_5 << " " 
				  << height_6 << " " << height_7 << " " << height_8 << " " << height_9 << " " << height_10 << std::endl;
		std::ofstream out;
		out.open("/home/pibot/Wuchenxi_data/Measurement.txt",std::ios::in|std::ios::out|std::ios::binary);
		if (out.is_open())
		{
			out << "Width of the Tunnel is " << W << " m.\n";
			out << "Height_1 of the tunnel is " << height_1 << " m.\n";
			out << "Height_2 of the tunnel is " << height_2 << " m.\n";
			out << "Height_3 of the tunnel is " << height_3 << " m.\n";
			out << "Height_4 of the tunnel is " << height_4 << " m.\n";
			out << "Height_5 of the tunnel is " << height_5 << " m.\n";
			out << "Height_6 of the tunnel is " << height_6 << " m.\n";
			out << "Height_7 of the tunnel is " << height_7 << " m.\n";
			out << "Height_8 of the tunnel is " << height_8 << " m.\n";
			out << "Height_9 of the tunnel is " << height_9 << " m.\n";
			out << "Height_10 of the tunnel is " << height_10 << " m.\n";
		}
		
		sensor_msgs::convertPointCloudToPointCloud2(cloud_output_r, pc2);
		pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
		pcl::fromROSMsg(pc2, pcl_cloud);
		pcl::io::savePCDFileASCII ("/home/pibot/Wuchenxi_data/cloud.pcd", pcl_cloud);
		ros::shutdown();
}

