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
	listener.waitForTransform("/base_link", "/laser_link", ros::Time::now(), ros::Duration(3.0));
	listener.lookupTransform("/base_link", "/laser_link", ros::Time::now(), transform);
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
	if(std::cin.fail()){
		rotation_time = 20;
		ROS_INFO("Input is not a valid number, Program will run under default time: 20s!");
	}
	pointcloud_size = 14380 * rotation_time;//14380
	pub = node.advertise<sensor_msgs::PointCloud>("/Cloud", 100);
	point_num = 719;//719
	floor_sum = 0.0;
	cloud_output.points.resize(pointcloud_size);
	cloud_output_r.points.resize(pointcloud_size);
	cloud_rviz.points.resize(pointcloud_size);
	cloud_last.points.resize(720);
	cloud_output.header.frame_id = "base_link";
	cloud_output_r.header.frame_id = "base_link";
	cloud_rviz.header.frame_id = "base_link";
	cloud_last.header.frame_id = "base_link";
	current_size = 0;
	iterate_time = 0;
	width_l = width_r = 0.0;
	Roll = Pitch = Yaw = 0.0;
	cnt_wl = cnt_wr = cnt_1 = cnt_2 = cnt_3 = cnt_4 = cnt_5 = cnt_6 = cnt_7 = cnt_8 = cnt_9 = cnt_10 = 0;
	sum_temp1 = sum_temp2 = sum_temp3 = sum_temp4 = sum_temp5 = 0.0;
	height_1 = height_2 = height_3 = height_4 = height_5 = height_6 = height_7 = height_8 = height_9 = height_10 = 0.0;
	W = 100;

	iteration =  20 * rotation_time;
	_length = new double[iteration]; 							//store width of each scan
	_qua = new geometry_msgs::Quaternion[iteration];  		//store quaternion of each scan
	_delta_q = new geometry_msgs::Quaternion[iteration]; 
	_theta = new double[iteration];							//store angles for widths
 	_width_sum = 0;
	_width_final = 0;
	_width_calc = new double[iteration];
	_width_rs = new double[iteration];
	_width_ls = new double[iteration];
	_roll = new double[iteration];
	_pitch = new double[iteration];
	std::cout << "Initialization Complete." << std::endl;
}

Convert::~Convert()
{
	delete []_qua;
	delete []_delta_q;
	delete []_width_rs ;
	delete []_width_ls;
}

void callback(const sensor_msgs::LaserScanConstPtr &scan, const sensor_msgs::ImuConstPtr &imu, Convert* &convertob_)
{
	convertob_->CombinedCallback( *scan, *imu);   
}


void Convert::get_floor(const sensor_msgs::PointCloud& pc_){
	std::map<double, int> z_map;
	std::map<double, int>::iterator it;
	it = z_map.begin(); 
	int biggest = 0;
	double z_value[720];
	for(int i=0;i<719;i++){
			z_value[i] = round(pc_.points[i].z * 1000);
		while(it != z_map.end()){
			if(z_value[i] == it->first && z_value[i] <= 0){
				it->second = it->second + 1;
				it = z_map.begin();
				break;
			}
			else{
				it++;
			}
			
		}
		if(it == z_map.end()){
			z_map.insert(it, std::pair<double, int>(z_value[i], 1));
			it = z_map.begin();
		}
	}
	it = z_map.begin();
	for(it = z_map.begin();it!=z_map.end();it++)
		if(it->second >= biggest){
			biggest = it->second;
			floor = it->first;
			break;
		}
		else{
			it++;
		}
	floor = floor / 1000;
}



void Convert::CombinedCallback(const sensor_msgs::LaserScan& scan, const sensor_msgs::Imu& imu)
{	
	/*double ww = imu.orientation.w;																
	double xx = imu.orientation.x;
	double yy = imu.orientation.y;
	double zz = imu.orientation.z;
	q0=qs_w*ww-xx*qs_x-qs_y*yy-qs_z*zz;
	q1=qs_x*ww+qs_w*xx-qs_z*yy+qs_y*zz;
	q2=qs_y*ww+qs_z*xx+qs_w*yy-qs_x*zz;
	q3=qs_w*ww-qs_y*xx+qs_x*yy+qs_w*zz;*/
	q0 = imu.orientation.w;														//get quaternion from IMU						
	q1 = imu.orientation.x;
	q2 = imu.orientation.y;
	q3 = imu.orientation.z;
	_qua[iterate_time].w = imu.orientation.w;
	_qua[iterate_time].x = -imu.orientation.x;
	_qua[iterate_time].y = -imu.orientation.y;
	_qua[iterate_time].z = -imu.orientation.z;
	
	double p=-asin(2*q1*q3-2*q0*q2);//PITCH
	double y=atan2(2.0*q1*q2+2.0*q0*q3,q0*q0+q1*q1-q2*q2-q3*q3);//YAW
	double r=atan2(2.0*q2*q3+2*q0*q1,q3*q3+q0*q0-q1*q1-q2*q2);
	_roll[iterate_time] = atan2(2.0*q2*q3+2*q0*q1,q3*q3+q0*q0-q1*q1-q2*q2);
	_pitch[iterate_time] = -asin(2*q1*q3-2*q0*q2);
	projector.transformLaserScanToPointCloud("/base_link", scan, pointcloud, listener);			//porject scan to pointcloud
	get_floor(pointcloud);	//calc floor position
/*----------- Filter the Tripod ----------*/
	for(int i = 0;i < point_num; ++i){
		if(pointcloud.points[i].z > (floor - 0.05) && pointcloud.points[i].z < +0.05 && pointcloud.points[i].y <= 0.4 && pointcloud.points[i].y >= -0.4){
			pointcloud.points[i].x = pointcloud.points[i].y = pointcloud.points[i].z = 0.0;
		}
	}
/*----------- calculate width of each scan ----------*/
	for(int k = 0;k < point_num; ++k)							
	{	
		
		if((pointcloud.points[k].z > -0.1) && (pointcloud.points[k].z < 0.1) && (pointcloud.points[k].y > 0)) //1.2 11_10
		{
			sum_temp1 += pointcloud.points[k].y;
			++cnt_wl; 
		}
		else if((pointcloud.points[k].z > -0.1) && (pointcloud.points[k].z < 0.1) && (pointcloud.points[k].y < 0))
		{
			sum_temp2 += pointcloud.points[k].y;
			++cnt_wr; 	
		} 
	}
	width_l = sum_temp1 / cnt_wl;
	width_r = sum_temp2 / cnt_wr;
	cnt_wl = cnt_wr = sum_temp1 = sum_temp2 = 0;
	_width_ls[iterate_time] = width_l;
	_width_rs[iterate_time] = width_r;
	_length[iterate_time] = width_l - width_r;
/*---------- store the shortest position ----------*/
		if(W > _length[iterate_time]){
		W = _length[iterate_time];
		q_mid.w = imu.orientation.w;
		q_mid.x = -imu.orientation.x;
		q_mid.y = -imu.orientation.y;
		q_mid.z = -imu.orientation.z;
		}
	std::cout << "W="  << W << "     " << "floor= " << floor << std::endl;
	std::cout << "Time: " << iterate_time * 0.05 << std::endl; 
/*---------- transform of each scan ----------*/
	
	double X_1 = q0*q0+q1*q1-q2*q2-q3*q3;
	double X_2 = 2*q1*q2-2*q0*q3;
	double X_3 = 2*q1*q3+2*q0*q2;
	double Y_1 = 2*q1*q2+2*q0*q3;
	double Y_2 = q0*q0+q2*q2-q1*q1-q3*q3;
	double Y_3 = 2*q2*q3-2*q0*q1;
	double Z_1 = 2*q1*q3-2*q0*q2;
	double Z_2 = 2*q2*q3+2*q0*q1;
	double Z_3 = q0*q0+q3*q3-q1*q1-q2*q2;
	if (current_size < pointcloud_size - 720) //719
	{	
		for(int i = 0; i < point_num; ++i)
		{ /*
			cloud_output.points[i + current_size].x = X_1*pointcloud.points[i].x + \
													  X_2*pointcloud.points[i].y + \
													  X_3*pointcloud.points[i].z ;
			cloud_output.points[i + current_size].y = Y_1*pointcloud.points[i].x + \
													  Y_2*pointcloud.points[i].y + \
													  Y_3*pointcloud.points[i].z ;
			cloud_output.points[i + current_size].z = Z_1*pointcloud.points[i].x + \
						    			 		      Z_2*pointcloud.points[i].y + \
									  			      Z_3*pointcloud.points[i].z;	*/
			//cloud_output.points[i].x -= 0.1*cos(_pitch[iterate_time]);//-0.1

			cloud_output.points[i + current_size].x = cos(y)*pointcloud.points[i].x - \
													  sin(y)*pointcloud.points[i].y ;
			cloud_output.points[i + current_size].y = -sin(y)*pointcloud.points[i].x + \
													  cos(y)*pointcloud.points[i].y ;
			cloud_output.points[i + current_size].z = pointcloud.points[i].z;
	
		}
		cloud_rviz.header.stamp = ros::Time::now();
		current_size += point_num;	
		floor_sum = floor_sum + floor;
		iterate_time++;
		pub.publish(cloud_output);
	}
	else 
	{	floor = floor_sum / iterate_time ;
		std::cout << "Reached maximum point number" << std::endl;
		double roll_ave = std::accumulate(_roll, _roll+iterate_time,0) / iterate_time;
		double pitch_ave = std::accumulate(_pitch, _pitch+iterate_time,0) / iterate_time;
		double yaw_mid=atan2(2.0*q_mid.x*q_mid.y+2.0*q_mid.w*q_mid.z,q_mid.w*q_mid.w+q_mid.x*q_mid.x-q_mid.y*q_mid.y-q_mid.z*q_mid.z);
		
		/*----------Rotate the pointclouds to the orthogonal axis----------*/
		
		double X_1 = (1 - 2*q_mid.y*q_mid.y - 2*q_mid.z*q_mid.z) ;
		double X_2 = (2*q_mid.x*q_mid.y - 2*q_mid.w*q_mid.z) ;
		double X_3 = (2*q_mid.x*q_mid.z + 2*q_mid.w*q_mid.y);
		double Y_1 = (2*q_mid.x*q_mid.y + 2*q_mid.w*q_mid.z);
		double Y_2 = (1 - 2*q_mid.x*q_mid.x - 2*q_mid.z*q_mid.z);
		double Y_3 = (2*q_mid.y*q_mid.z - 2*q_mid.w*q_mid.x);
		double Z_1 = (2*q_mid.x*q_mid.z - 2*q_mid.w*q_mid.y);
		double Z_2 = (2*q_mid.y*q_mid.z + 2*q_mid.w*q_mid.x);
		double Z_3 = (1 - 2*q_mid.x*q_mid.x - 2*q_mid.y*q_mid.y);


		for(int n = 0;n <= current_size; ++n)
		{
			cloud_output_r.points[n].x = cos(yaw_mid)*cloud_output.points[n].x - sin(yaw_mid)*cloud_output.points[n].y ;
			cloud_output_r.points[n].y = -sin(yaw_mid)*cloud_output.points[n].x + cos(yaw_mid)*cloud_output.points[n].y ;
			cloud_output_r.points[n].z = cloud_output.points[n].z;

		}
		q_mid.x = - q_mid.x;
		q_mid.y = - q_mid.y;
		q_mid.z = - q_mid.z;
		
		for(int i = 0;i <= iterate_time; i++){
			_delta_q[i].w = _qua[i].w*q_mid.w - _qua[i].x*q_mid.x - _qua[i].y*q_mid.y - _qua[i].z*q_mid.z;
			_delta_q[i].x = _qua[i].w*q_mid.x + _qua[i].x*q_mid.w + _qua[i].z*q_mid.y - _qua[i].y*q_mid.z;
			_delta_q[i].y = _qua[i].w*q_mid.y + _qua[i].y*q_mid.w + _qua[i].x*q_mid.z - _qua[i].z*q_mid.x;
			_delta_q[i].z = _qua[i].w*q_mid.z + _qua[i].z*q_mid.w + _qua[i].y*q_mid.x - _qua[i].x*q_mid.y;

			_theta[i] = atan2(2 * (_delta_q[i].w*_delta_q[i].z + _delta_q[i].x*_delta_q[i].y ), 1 - 2 * (_delta_q[i].y *_delta_q[i].y - _delta_q[i].z*_delta_q[i].z));
			_width_l_sum += _width_ls[i] * cos(_theta[i]);
			_width_r_sum += _width_rs[i] * cos(_theta[i]);
			_width_calc[i] = _length[i] * cos(_theta[i]);

		}
		_width_sum = std::accumulate(_width_calc,_width_calc + iterate_time,0);
		_width_l_final = _width_l_sum /iterate_time;
		_width_r_final = _width_r_sum /iterate_time;
		_width_final = _width_sum / iterate_time;
		double w = _width_final / 11;
			std::cout << "width final= "<<_width_final << "  width right= " << _width_r_final << "  width left= " << _width_l_final<<"w= " << w <<  std::endl;
		/*----------Calculate the height ----------*/
		for(int n = 0;n <= current_size; ++n)
		{	

			if(cloud_output_r.points[n].y >= _width_r_final+w-0.1 && cloud_output_r.points[n].y <= _width_r_final+w+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{	
					height_1 += cloud_output_r.points[n].z;
					++ cnt_1;
				}
			}
			else if(cloud_output_r.points[n].y >= _width_r_final+2*w-0.1 && cloud_output_r.points[n].y <= _width_r_final+2*w+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_2 += cloud_output_r.points[n].z;
					++ cnt_2;
				}
			}
			else if(cloud_output_r.points[n].y >= 3*w+_width_r_final-0.1 && cloud_output_r.points[n].y <= 3*w+_width_r_final+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_3 += cloud_output_r.points[n].z;
					++ cnt_3;
				}
			}
			else if(cloud_output_r.points[n].y >= 4*w+_width_r_final-0.1 && cloud_output_r.points[n].y <= 4*w+_width_r_final+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_4 += cloud_output_r.points[n].z;
					++ cnt_4;
				}
			}
			else if(cloud_output_r.points[n].y >= 5*w+_width_r_final-0.1 && cloud_output_r.points[n].y <= 5*w+_width_r_final+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_5 += cloud_output_r.points[n].z;
					++ cnt_5;
				}
			}
			else if(cloud_output_r.points[n].y >= 6*w+_width_r_final-0.1 && cloud_output_r.points[n].y <= 6*w+_width_r_final+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_6 += cloud_output_r.points[n].z;
					++ cnt_6;
				}
			}
			else if(cloud_output_r.points[n].y >= 7*w+_width_r_final-0.1 && cloud_output_r.points[n].y <= 7*w+_width_r_final+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_7 += cloud_output_r.points[n].z;
					++ cnt_7;
				}
			}
			else if(cloud_output_r.points[n].y >= 8*w+_width_r_final-0.1 && cloud_output_r.points[n].y <= 8*w+_width_r_final+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
						height_8 += cloud_output_r.points[n].z;
					++ cnt_8;
				}
			}
			else if(cloud_output_r.points[n].y >= 9*w+_width_r_final-0.1 && cloud_output_r.points[n].y <= 9*w+_width_r_final+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_9 += cloud_output_r.points[n].z;
					++ cnt_9;
				}
			}
			else if(cloud_output_r.points[n].y >= 10*w+_width_r_final-0.1 && cloud_output_r.points[n].y <= 10*w+_width_r_final+0.1 && cloud_output_r.points[n].z > 0)
			{
				if(cloud_output_r.points[n].x >= -2.5 && cloud_output_r.points[n].x <=2.5)
				{
					height_10 += cloud_output_r.points[n].z;
					++ cnt_10;
				}
			}
		}
		std::cout << "cnt1= " << cnt_1 << std::endl;
		height_1 = height_1 / cnt_1 - floor;
		height_2 = height_2 / cnt_2 - floor;
		height_3 = height_3 / cnt_3 - floor;
		height_4 = height_4 / cnt_4 - floor;
		height_5 = height_5 / cnt_5 - floor;
		height_6 = height_6 / cnt_6 - floor;
		height_7 = height_7 / cnt_7 - floor;
		height_8 = height_8 / cnt_8 - floor;
		height_9 = height_9 / cnt_9 - floor;
		height_10 = height_10 / cnt_10 - floor;
/*----------Output to screen & Save to txt file----------*/
		std::cout << "Width is " << _width_final << " m." << std::endl;
		std::cout << "Floor is " << floor << " m." << std::endl;
		std::cout << "Height_1 of the tunnel is " << height_1 << " m." << std::endl;
		std::cout << "Height_2 of the tunnel is " << height_2 << " m." << std::endl;
		std::cout << "Height_3 of the tunnel is " << height_3 << " m." << std::endl;
		std::cout << "Height_4 of the tunnel is " << height_4 << " m." << std::endl;
		std::cout << "Height_5 of the tunnel is " << height_5 << " m." << std::endl;
		std::cout << "Height_6 of the tunnel is " << height_6 << " m." << std::endl;
		std::cout << "Height_7 of the tunnel is " << height_7 << " m." << std::endl;
		std::cout << "Height_8 of the tunnel is " << height_8 << " m." << std::endl;
		std::cout << "Height_9 of the tunnel is " << height_9 << " m." << std::endl;
		std::cout << "Height_10 of the tunnel is " << height_10 << " m." << std::endl;
		std::ofstream out;
		out.open("/home/pibot/Wuchenxi_data/Measurement.txt",std::ios::in|std::ios::out|std::ios::binary);
		if (out.is_open())
		{
			out << "Width of the tunnel is " << W << " m." << std::endl;
			out << "Floor is " << floor << std::endl;
			out << "Height_1 of the tunnel is " << height_1 << " m." << std::endl;
			out << "Height_2 of the tunnel is " << height_2 << " m." << std::endl;
			out << "Height_3 of the tunnel is " << height_3 << " m." << std::endl;
			out << "Height_4 of the tunnel is " << height_4 << " m." << std::endl;
			out << "Height_5 of the tunnel is " << height_5 << " m." << std::endl;
			out << "Height_6 of the tunnel is " << height_6 << " m." << std::endl;
			out << "Height_7 of the tunnel is " << height_7 << " m." << std::endl;
			out << "Height_8 of the tunnel is " << height_8 << " m." << std::endl;
			out << "Height_9 of the tunnel is " << height_9 << " m." << std::endl;
			out << "Height_10 of the tunnel is " << height_10 << " m." << std::endl;
		}

		sensor_msgs::convertPointCloudToPointCloud2(cloud_output_r, pc2);
		sensor_msgs::convertPointCloudToPointCloud2(cloud_last, pc2_last);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(pc2, *pcl_cloud);
		pcl::io::savePCDFileASCII ("/home/pibot/Wuchenxi_data/cloud.pcd", *pcl_cloud);
		pcl::KdTreeFLANN<pcl::PointXYZ> cloud_kdTree;
		cloud_kdTree.setInputCloud(pcl_cloud);
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

void signalHandler(int signum)
{
	std::cout << "Process stopped manually." << std::endl;
	ros::shutdown();

}

