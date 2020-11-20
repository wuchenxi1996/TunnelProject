void Convert::CombinedCallback(const sensor_msgs::LaserScan& scan, const sensor_msgs::Imu& imu)
{
	imu_get.orientation = imu.orientation;
	imu_get.header.stamp = imu.header.stamp;
	q0 = imu_get.orientation.w;
	q1 = imu_get.orientation.x;
	q2 = imu_get.orientation.y;
	q3 = imu_get.orientation.z;
	Yaw = atan2(2*q1*q2 + 2*q0*q3, q0*q0 + q1*q1 - q2*q2 - q3*q3);


	projector.transformLaserScanToPointCloud("/base_link", scan, pointcloud, listener);

/*	for(int k = 0;k < point_num; ++k)
	{	
		//pointcloud.points[k].x -= 0.12;
		if((pointcloud.points[k].z >  0.4) && (pointcloud.points[k].z < 0.6) && (pointcloud.points[k].y > 0))
		{
			sum_temp1 += pointcloud.points[k].y;
			++cnt_wl; 
		}
		else if((pointcloud.points[k].z >  0.4) && (pointcloud.points[k].z < 0.6) && (pointcloud.points[k].y < 0))
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
		q_temp.w = q0;
		q_temp.x = -q1;
		q_temp.y = -q2;
		q_temp.z = -q3;
		width_ls = width_l;
		width_rs = width_r;
	}

	*/
	std::cout << "Yaw"  << Yaw << std::endl;
	if (iterate_time <449)
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
			cloud_output.points[i + current_size].z = (2*q1*q3-2*q0*q2)*pointcloud.points[i].x + \
													  (2*q2*q3+2*q0*q1)*pointcloud.points[i].y + \
													  (1-2*q1*q1-2*q2*q2)*pointcloud.points[i].z;
		}
		current_size += point_num;	
		iterate_time++;
	}
	else 
	{	
		std::cout << "Reached maximum point number" << std::endl;
		/*----------Rotate the pointclouds to the orthogonal axis----------*/
	/*	for(int n = 0;n <= current_size; ++n)
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

	/*		if(cloud_output_r.points[n].y >= width_rs+w-0.1 && cloud_output_r.points[n].y <= width_rs+w+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_1 += cloud_output_r.points[n].z;
				++ cnt_1;
			}
			else if(cloud_output_r.points[n].y >= width_rs+2*w-0.1 && cloud_output_r.points[n].y <= width_rs+2*w+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_2 += cloud_output_r.points[n].z;
				++ cnt_2;
			}
			else if(cloud_output_r.points[n].y >= 3*w+width_rs-0.1 && cloud_output_r.points[n].y <= 3*w+width_rs+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_3 += cloud_output_r.points[n].z;
				++ cnt_3;
			}
			else if(cloud_output_r.points[n].y >= 4*w+width_rs-0.1 && cloud_output_r.points[n].y <= 4*w+width_rs+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_4 += cloud_output_r.points[n].z;
				++ cnt_4;
			}
			else if(cloud_output_r.points[n].y >= 5*w+width_rs-0.1 && cloud_output_r.points[n].y <= 5*w+width_rs+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_5 += cloud_output_r.points[n].z;
				++ cnt_5;
			}
			else if(cloud_output_r.points[n].y >= 6*w+width_rs-0.1 && cloud_output_r.points[n].y <= 6*w+width_rs+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_6 += cloud_output_r.points[n].z;
				++ cnt_6;

			}
			else if(cloud_output_r.points[n].y >= 7*w+width_rs-0.1 && cloud_output_r.points[n].y <= 7*w+width_rs+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_7 += cloud_output_r.points[n].z;
				++ cnt_7;
			}
			else if(cloud_output_r.points[n].y >= 8*w+width_rs-0.1 && cloud_output_r.points[n].y <= 8*w+width_rs+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_8 += cloud_output_r.points[n].z;
				++ cnt_8;
			}
			else if(cloud_output_r.points[n].y >= 9*w+width_rs-0.1 && cloud_output_r.points[n].y <= 9*w+width_rs+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_9 += cloud_output_r.points[n].z;
				++ cnt_9;
			}
			else if(cloud_output_r.points[n].y >= 10*w+width_rs-0.1 && cloud_output_r.points[n].y <= 10*w+width_rs+0.1 && cloud_output_r.points[n].z > 0)
			{
				height_10 += cloud_output_r.points[n].z;
				++ cnt_10;
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
*/
/*----------Output to screen----------*/
		std::cout << "Left width is " <<width_ls << "  Right width is " << width_rs << std::endl;
		std::cout << "Width of the tunnel is " << W << std::endl;
 		std::cout << height_1 << " " << height_2 << " " << height_3 << " " << height_4 << " " << height_5 << " " 
				  << height_6 << " " << height_7 << " " << height_8 << " " << height_9 << " " << height_10 << std::endl;
		sensor_msgs::convertPointCloudToPointCloud2(cloud_output, pc2);
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
/*
