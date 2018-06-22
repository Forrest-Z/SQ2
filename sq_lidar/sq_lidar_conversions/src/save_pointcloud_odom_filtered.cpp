#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr save_pc_ (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr old_pc_ (new pcl::PointCloud<pcl::PointXYZI>);

nav_msgs::Odometry odom_;
nav_msgs::Odometry init_odom_;

ros::Publisher pub_pc;

Eigen::Matrix4f transform_matrix;
Eigen::Matrix4f inverse_transform_matrix;

int count_ = 0;
int save_num = 60;

bool init_lcl_flag = true;

// float z_threshold = 5.0;
float z_threshold = 6.0;

string POINTCLOUD_TOPIC;
string OUTPUT_TOPIC;
string ODOM_TOPIC;
string ODOM_PARENT_FRAME;
string ODOM_CHILD_FRAME;

Eigen::Matrix4f create_matrix(nav_msgs::Odometry odom_now){
	double roll_now, pitch_now, yaw_now;
	
	tf::Quaternion q_now(odom_now.pose.pose.orientation.x, odom_now.pose.pose.orientation.y, odom_now.pose.pose.orientation.z, odom_now.pose.pose.orientation.w);
	tf::Matrix3x3(q_now).getRPY(roll_now, pitch_now, yaw_now);
	// cout<<"roll_now : "<<roll_now<<", pitch_now : "<<pitch_now<<", yaw_now : "<<yaw_now<<endl;
	
	Eigen::Translation3f init_translation(odom_now.pose.pose.position.x, odom_now.pose.pose.position.y, odom_now.pose.pose.position.z);
	Eigen::AngleAxisf init_rotation_x(roll_now, Eigen::Vector3f::UnitX());
	Eigen::AngleAxisf init_rotation_y(pitch_now, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf init_rotation_z(yaw_now, Eigen::Vector3f::UnitZ());
	
	Eigen::Matrix4f init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

	// Eigen::Matrix4f init_guess(Eigen::Matrix4f::Identity());
	return init_guess;
}

void lcl_callback(nav_msgs::Odometry msg){
	if(init_lcl_flag){
		init_odom_ = msg;
		init_lcl_flag = false;
	}
	odom_ = msg;
	// odom_.pose.pose.position.x -= init_odom_.pose.pose.position.x;
	// odom_.pose.pose.position.y -= init_odom_.pose.pose.position.y;
	// odom_.pose.pose.position.z -= init_odom_.pose.pose.position.z;
	// odom_.pose.pose.orientation.x -= init_odom_.pose.pose.orientation.x;
	// odom_.pose.pose.orientation.y -= init_odom_.pose.pose.orientation.y;
	// odom_.pose.pose.orientation.z -= init_odom_.pose.pose.orientation.z;
	// odom_.pose.pose.orientation.w -= init_odom_.pose.pose.orientation.w;
	// cout<<"odom_.pose.pose.position.x : "<<odom_.pose.pose.position.x<<endl;
	// cout<<"odom_.pose.pose.position.y : "<<odom_.pose.pose.position.y<<endl;
	// cout<<"odom_.pose.pose.position.z : "<<odom_.pose.pose.position.z<<endl;
	// cout<<"odom_.pose.pose.orientation.x : "<<odom_.pose.pose.orientation.x<<endl;
	// cout<<"odom_.pose.pose.orientation.y : "<<odom_.pose.pose.orientation.y<<endl;
	// cout<<"odom_.pose.pose.orientation.x : "<<odom_.pose.pose.orientation.z<<endl;
	// cout<<"odom_.pose.pose.orientation.w : "<<odom_.pose.pose.orientation.w<<endl;

	transform_matrix = create_matrix(odom_);
	// cout<<"transform_matrix : "<<endl<<transform_matrix<<endl;
}

void pc_callback(sensor_msgs::PointCloud2 msg){
	// clock_t start=clock();

	// cout<<"-----------------------"<<endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr single_pc_ (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(msg, *single_pc_);
	// cout<<"single_pc_ : "<<single_pc_->points.size()<<endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc (new pcl::PointCloud<pcl::PointXYZI>);
	// pcl::transformPointCloud(*single_pc_, *output_pc, inverse_transform_matrix);
	pcl::transformPointCloud(*single_pc_, *output_pc, transform_matrix);

	pcl::PointCloud<pcl::PointXYZI>::Ptr output_pc_after (new pcl::PointCloud<pcl::PointXYZI>);
	size_t single_pc_size = single_pc_->points.size();
	for(size_t i=0; i<single_pc_size;i++){
		// if(single_pc_->points[i].z <= z_threshold){
		if((single_pc_->points[i].z <= z_threshold) && (output_pc->points[i].intensity > 600)){
			pcl::PointXYZI temp;
			temp.x = output_pc->points[i].x;
			temp.y = output_pc->points[i].y;
			temp.z = output_pc->points[i].z;
			temp.intensity = output_pc->points[i].intensity;
			output_pc_after->points.push_back(temp);
		}
	}
		
	if(count_ < save_num){
		*save_pc_ += *output_pc_after;
		old_pc_ = output_pc_after;
	}
	else{
		int old_pc_size = (int)old_pc_->points.size();
		save_pc_->points.erase(save_pc_->points.begin(), save_pc_->points.begin()+old_pc_size);	
		*save_pc_ += *output_pc_after;
		old_pc_ = output_pc_after;
	}
	// cout<<"save_pc_ : "<<save_pc_->points.size()<<endl;
	// cout<<"save_pc_ : "<<save_pc_->width<<endl;
	// cout<<"save_pc_ : "<<save_pc_->height<<endl;
	pcl::PointCloud<pcl::PointXYZI>::Ptr output_save_pc (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc (new pcl::PointCloud<pcl::PointXYZI>);
	Eigen::Matrix4f inverse_transform_matrix = transform_matrix.inverse();
	pcl::transformPointCloud(*save_pc_, *output_save_pc, inverse_transform_matrix);
////1cmのvoxelで間引く///Add by Onda 2017/09/27////////////////////////////////////////////////////////

	pcl::VoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
	approximate_voxel_filter.setInputCloud(output_save_pc);
	approximate_voxel_filter.setLeafSize(0.07, 0.07, 0.07);
	approximate_voxel_filter.filter(*filtered_pc);



///////////////////////////////////////////////////////////////////////////
	sensor_msgs::PointCloud2 pc_;
	// toROSMsg(*output_save_pc, pc_);
	toROSMsg(*filtered_pc, pc_);
	pc_.header = msg.header;

	pub_pc.publish(pc_);
	count_++;

	// cout<<(double)(clock()-start)/CLOCKS_PER_SEC<<endl<<endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_lidar_point_lcl");
	ros::NodeHandle n;

	n.getParam("/sq_lidar_conversions/pointcloud_topic", POINTCLOUD_TOPIC);
	n.getParam("/sq_lidar_conversions/output_topic", OUTPUT_TOPIC);
	n.getParam("/sq_lidar_conversions/odom_topic", ODOM_TOPIC);
	n.getParam("/sq_lidar_conversions/odom_parent_frame", ODOM_PARENT_FRAME);
	n.getParam("/sq_lidar_conversions/odom_child_frame", ODOM_CHILD_FRAME);

	// ros::Subscriber sub_pc = n.subscribe(POINTCLOUD_TOPIC, 1, pc_callback);
	// ros::Subscriber sub_lcl = n.subscribe(ODOM_TOPIC, 1, lcl_callback);
	ros::Subscriber sub_pc = n.subscribe(POINTCLOUD_TOPIC, 10, pc_callback);
	ros::Subscriber sub_lcl = n.subscribe(ODOM_TOPIC, 10, lcl_callback);

	ros::Rate loop_rate(40);
	
	pub_pc = n.advertise<sensor_msgs::PointCloud2>(OUTPUT_TOPIC, 1);
	
	nav_msgs::Odometry init_odom;

	init_odom.header.frame_id = ODOM_PARENT_FRAME;
	init_odom.child_frame_id = ODOM_CHILD_FRAME;
	init_odom.pose.pose.position.x = 0.0;
	init_odom.pose.pose.position.y = 0.0;
	init_odom.pose.pose.position.z = 0.0;
	init_odom.pose.pose.orientation.x = 0.0;
	init_odom.pose.pose.orientation.y = 0.0;
	init_odom.pose.pose.orientation.z = 0.0;
	init_odom.pose.pose.orientation.w = 0.0;

	odom_ = init_odom;
	
	cout<<"start!!"<<endl;

	// ros::spin();
	
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
