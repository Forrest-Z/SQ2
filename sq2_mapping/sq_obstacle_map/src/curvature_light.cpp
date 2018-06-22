//curvatureの算出のみを行う
//rvizで参照可
//パラメータ調整で点群数が変わる


#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include </opt/ros/fuerte/include/pcl-1.6/pcl/ros/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <velodyne_msgs/point_types.h>
#include <boost/thread.hpp>


using namespace std;

//Global Variables
pcl::PointCloud<pcl::PointXYZINormal> pc_in;

boost::mutex pc_in_mutex_;

#define curv	0.10	//実験値0.2
#define curv_	1.0/curv //10/29 足回り取り替え前までのパラメータ
// #define curv_	0.4/curv
//#define curv 0.15	//実験値

void pc_callback(sensor_msgs::PointCloud2::ConstPtr msg){
	boost::mutex::scoped_lock(pc_in_mutex_);               //同時アクセスの制限
	pcl::fromROSMsg(*msg , pc_in);
//	cout<<pc_in.points.size()<<endl;
}

int
main (int argc, char** argv)
{

	ros::init(argc, argv, "curvature");
	ros::NodeHandle n;
	ros::Rate roop(10);

//	ros::Subscriber sub = n.subscribe("perfect_estimation",1,pc_callback);
//	ros::Subscriber sub = n.subscribe("/human_detection/forS",1,pc_callback);
	// ros::Subscriber sub = n.subscribe("/rm_cluster/removed_points", 10, pc_callback);
	ros::Subscriber sub = n.subscribe("/perfect_velodyne/normal", 10, pc_callback);
	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("curvature", 10);

	sensor_msgs::PointCloud ros_pc;
	ros_pc.header.frame_id = "/velodyne";
	sensor_msgs::PointCloud2 ros_pc2;
	
	size_t i;
	size_t point_size;
	double cost_c, dist;
	pcl::PointXYZI p;
	pcl::PointCloud<pcl::PointXYZI> pcl_pc;
	pcl::PointCloud<pcl::PointXYZINormal> cloud_tmp;             //xyz,輝度,法線

	cout<<"Here we go!!!"<<endl;

	while(ros::ok()){
		//pcl::PointCloud<pcl::PointXYZI> pcl_pc;
////////////////////////////////////////////////////////////////pc_in		
		//pcl::PointCloud<pcl::PointXYZINormal> cloud_tmp;
		{
			boost::mutex::scoped_lock(pc_in_mutex_);
			cloud_tmp= pc_in;                        //輝度以外sub
		}
////////////////////////////////////////////////////////////////curv
		point_size = cloud_tmp.points.size();
		//for(i=0; i<cloud_tmp.points.size(); i++){	
		for(i=0; i<point_size; ++i){	
			//cost_c = cloud_tmp.points[i].curvature/curv;
			cost_c = cloud_tmp.points[i].curvature*curv_;

			if(cost_c >= 1.0){
				//pcl::PointXYZI p;
				p.x = cloud_tmp.points[i].x;
				p.y = cloud_tmp.points[i].y;
				p.z = cloud_tmp.points[i].z;
				p.intensity = cost_c;
				//dist = sqrt(pow(p.x,2) + pow(p.y, 2));
				//dist = pow(p.x, 2) + pow(p.y, 2);
				dist = p.x*p.x + p.y*p.y;
				//if((dist > 0.3) && (p.z < 0.5)){
				//if((dist > 2.5) && (p.z < 0.5)){
				if((dist > 6.25) && (p.z < 0.25)){	// 上を二乗
					pcl_pc.points.push_back(p);
				}
			}
		}
////////////////////////////////////////////////////////////////to_ros_msgs
        pcl::toROSMsg(pcl_pc, ros_pc2);
        sensor_msgs::convertPointCloud2ToPointCloud(ros_pc2,ros_pc);
		ros_pc.header.frame_id = "/velodyne";
        ros_pc.header.stamp = ros::Time::now();
        pub.publish(ros_pc);

		pcl_pc.points.clear();

////////////////////////////////////////////////////////////////
		ros::spinOnce();
		roop.sleep();
	}
	return 0;
}
