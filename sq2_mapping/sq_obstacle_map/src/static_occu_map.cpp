//forsq2

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <math.h>

using namespace std;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_data (new pcl::PointCloud<pcl::PointXYZ>);            //pclの型
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_dash (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud3 (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_3d (new pcl::PointCloud<pcl::PointXYZ>);


sensor_msgs::PointCloud2 cloud2;                      //rosの型
sensor_msgs::PointCloud2 cloud2_dash;

sensor_msgs::PointCloud2 local_map_3d_2;

nav_msgs::OccupancyGrid obstacle_map;
nav_msgs::GridCells obstacle_cell;

nav_msgs::OccupancyGrid local_map;
nav_msgs::OccupancyGrid local_map_expand;
nav_msgs::OccupancyGrid local_map_tf;

nav_msgs::Odometry odom;
nav_msgs::Odometry ndt_odom;

sensor_msgs::PointCloud obstacle_points;

double min_height = 0.2;
double max_height = 1.6;

// const float W = 800;
// const float H = 800;
const float W = 1500;
const float H = 1500;
// const float W = 10000;
// const float H = 10000;

const float min_x = 0.0;
const float min_y = 0.0;

const float R = 0.05;                //障害物判定の範囲を指定

float local_x = 0.0;
float local_y = 0.0;
float local_theta = 0.0 / 180.0 * M_PI;
const float local_W = 20.0;
const float local_H = 20.0;
const float local_dW = local_W / 2.0;
const float local_dH = local_H / 2.0;

const float RADIUS = 0.3;

// const int cell_filter = 10;		//for ikuta
// const int color_filter = 100; 	//for ikuta

const int cell_filter = 1;		//for tsukuba
const int color_filter = 100;	//for tsukuba
const float diff_height = 0.5;

bool sub_flag = false;
bool ndt_sub_flag = false;
bool pub_flag = true;

float x_init = 0.0;                              //初期の位置を決定 localmapに関係
float y_init = -0.0;
// float theta_init = (90.0 + 3.0) / 180.0 *M_PI;
// float theta_init = (-0.5 + 180.0) / 180.0 *M_PI;
float theta_init =(0.0) / 180.0 *M_PI;

void ndt_odom_callback(nav_msgs::Odometry msg){
	ndt_odom = msg;
	cout<<"ndt_odom : "<<ndt_odom.pose.pose.orientation.x<<endl;
	cout<<"ndt_odom : "<<ndt_odom.pose.pose.orientation.y<<endl;
	cout<<"ndt_odom : "<<ndt_odom.pose.pose.orientation.z<<endl;
	cout<<"ndt_odom : "<<ndt_odom.pose.pose.orientation.w<<endl;
	ndt_sub_flag = true;
}



void PointXYZ_to_PointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr *input, sensor_msgs::PointCloud2 *output){             //ros用のデータに変換
	pcl::PCLPointCloud2 pcl_cloud2;
	pcl::toPCLPointCloud2(**input, pcl_cloud2);
	pcl_conversions::fromPCL(pcl_cloud2, *output);
}

void PointCloud2_to_PointXYZ(sensor_msgs::PointCloud2 *input, pcl::PointCloud<pcl::PointXYZ>::Ptr *output){              //pcl用のデータに変換
	pcl::PCLPointCloud2 pcl_cloud2;
	pcl_conversions::toPCL(*input, pcl_cloud2);
	pcl::fromPCLPointCloud2(pcl_cloud2, **output);
}

void remove_point_by_heght(pcl::PointCloud<pcl::PointSurfel>::Ptr cloud, nav_msgs::OccupancyGrid *map){
		// pcl::PCLPointCloud2 pcl_cloud2;
		// pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);

		vector<float> data(W/R*H/R, 100.0);
		
		// cout<<"cloud : "<<cloud->points[0]<<endl;
		// cout<<"cloud->points[0].x : "<<cloud->points[0].x<<endl;
		// cout<<"cloud->points[0].y : "<<cloud->points[0].y<<endl;
		// cout<<"cloud->points[0].z : "<<cloud->points[0].z<<endl;
		// cout<<"cloud->points[0].normal_x : "<<cloud->points[0].normal_x<<endl;
		// cout<<"cloud->points[0].normal_y : "<<cloud->points[0].normal_y<<endl;
		// cout<<"cloud->points[0].normal_z : "<<cloud->points[0].normal_z<<endl;
		// cout<<"cloud->points[0].r : "<<(int)cloud->points[0].r<<endl;
		// cout<<"cloud->points[0].g : "<<(int)cloud->points[0].g<<endl;
		// cout<<"cloud->points[0].b : "<<(int)cloud->points[0].b<<endl;
		

		// cout<<"cloud->points.x : "<<cloud->points[0].x<<endl;
		// cout<<"cloud->points.y : "<<cloud->points[0].y<<endl;
		// cout<<"cloud->points.z : "<<cloud->points[0].z<<endl;
		// cout<<"couud->points.size() : "<<cloud->points.size()<<endl;


		for(size_t i = 0; i < cloud->points.size(); i++){
			int x = int((cloud->points[i].x - map->info.origin.position.x) / R);
			// cout<<"x : "<<x<<endl;
			int y = int((cloud->points[i].y - map->info.origin.position.y) / R);
			// cout<<"y : "<<y<<endl;
			int num = x + y * map->info.width;
			
			if(cloud->points[i].z < data[num]){
				data[num] = cloud->points[i].z;
			}
			
			// if(num == 32020123){
				// cout<<"data["<<num<<"] : "<<data[num]<<endl;
				// cout<<"temp_cloud->points["<<i<<"].z : "<<temp_cloud->points[i].z<<endl;
			// }
		}
		for(size_t i = 0; i < cloud->points.size(); i++){
			int x = int((cloud->points[i].x - map->info.origin.position.x) / R);
			// cout<<"x : "<<x<<endl;
			int y = int((cloud->points[i].y - map->info.origin.position.y) / R);
			// cout<<"y : "<<y<<endl;
			int num = x + y * map->info.width;
			if((data[num] + min_height) < cloud->points[i].z && cloud->points[i].z <= (data[num] + max_height)){
				// cout<<"data["<<num<<"] : "<<data[num]<<endl;
				// cout<<"temp_cloud->points["<<i<<"].x : "<<temp_cloud->points[i].x<<endl;
				// cout<<"temp_cloud->points["<<i<<"].y : "<<temp_cloud->points[i].y<<endl;
				// cout<<"temp_cloud->points["<<i<<"].z : "<<temp_cloud->points[i].z<<endl;
				// cout<<endl;
				pcl::PointXYZ temp_point;
				pcl::PointXYZ temp_point_dash;

				temp_point.x = cloud->points[i].x;
				temp_point.y = cloud->points[i].y;
				temp_point.z = cloud->points[i].z;
				// temp_point.r = cloud->points[i].r;
				// temp_point.g = cloud->points[i].g;
				// temp_point.b = cloud->points[i].b;

				temp_point_dash.x = cloud->points[i].x;
				temp_point_dash.y = cloud->points[i].y;
				temp_point_dash.z = cloud->points[i].z;
				// cout<<"temp_point.x : "<<temp_point.x<<endl;
				// cout<<"temp_point.y : "<<temp_point.y<<endl;
				// cout<<"temp_point.z : "<<temp_point.z<<endl;
				temp_cloud->push_back(temp_point);
				temp_cloud3->push_back(temp_point);
				temp_cloud_dash->push_back(temp_point_dash);
			}
		}
		
		for(size_t i=0; i < cloud->points.size(); i++){
			int x = int((cloud->points[i].x - map->info.origin.position.x) / R);
			// cout<<"x : "<<x<<endl;
			int y = int((cloud->points[i].y - map->info.origin.position.y) / R);
			// cout<<"y : "<<y<<endl;
			int num = x + y * map->info.width;
			if(cloud->points[i].z <= (data[num] + diff_height)){
				if((int)cloud->points[i].r <= color_filter && (int)cloud->points[i].g <= color_filter && (int)cloud->points[i].b <= color_filter){
					pcl::PointXYZ temp_point;

					temp_point.x = cloud->points[i].x;
					temp_point.y = cloud->points[i].y;
					temp_point.z = cloud->points[i].z;
					
					// cout<<"temp_point.x : "<<temp_point.x<<endl;
					// cout<<"temp_point.y : "<<temp_point.y<<endl;
					// cout<<"temp_point.z : "<<temp_point.z<<endl;
					temp_cloud2->push_back(temp_point);
					temp_cloud3->push_back(temp_point);
				}
			}
		}
			

		for(size_t i = 0; i < temp_cloud3->points.size(); i++){
			// cout<<"temp_cloud->points["<<i<<"].x : "<<temp_cloud->points[i].x<<endl;
			// cout<<"temp_cloud->points["<<i<<"].y : "<<temp_cloud->points[i].y<<endl;
			// cout<<"temp_cloud->points["<<i<<"].z : "<<temp_cloud->points[i].z<<endl;
			temp_cloud3->points[i].z = 0.0;
			// cout<<"temp_cloud->points["<<i<<"].x : "<<temp_cloud->points[i].x<<endl;
			// cout<<"temp_cloud->points["<<i<<"].y : "<<temp_cloud->points[i].y<<endl;
			// cout<<"temp_cloud->points["<<i<<"].z : "<<temp_cloud->points[i].z<<endl;
			
		}
}


void create_obstacle_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid *map){            //全体のマップを作成
	geometry_msgs::Point obstacle_point;

	vector<int>	count((long(W/R) * long(H/R)), 0);
	// vector<int>::iterator a;
	// for(a = count.begin(); a != count.end(); a++){
		// cout<<"a : "<<*a<<endl;
	// }
	// for(int i = 0;i<count.size(); i++){
		// cout<<count[i]<<endl;
	// }
	// vector<int8_t>::iterator mit;
	// for(mit=map->data.begin(); mit != map->data.end();mit++){
		// *mit=0;
	// }
	for(int i = 0; i != map->data.size(); i++){
		map->data[i] = 0;
		// cout<<"map->data["<<i<<"] : "<<map->data[i]<<endl;
	}
	// cout<<"cloud->points.size() : "<<cloud->points.size()<<endl;	
	for(size_t i = 0; i < cloud->points.size(); i++){
		// cout<<"i : "<<i<<endl;
		int x = int((cloud->points[i].x - map->info.origin.position.x) / R);
		// cout<<"x : "<<x<<endl;
		int y = int((cloud->points[i].y - map->info.origin.position.y) / R);
		// cout<<"y : "<<y<<endl;
		// cout<<"map->width : "<<map->info.width<<endl;
		if((0 <= x && x < W/R) && (0 <= y && y < H/R)){
			long num = x + y * map->info.width;
			// cout<<"map->data.size() : "<<map->data.size()<<endl;
			// cout<<"num : "<<num<<endl;
			// map->data[num] = 100;
			count[num] += 1; 
			// obstacle_point.x = cloud->points[i].x;
			// obstacle_point.y = cloud->points[i].y;
			// obstacle_point.z = 0;
		}
	}



	// for(int i = 0; i != map->data.size(); i++){
		// if(map->data[i] == 100){
			// cout<<"map->data["<<i<<"] : "<<map->data[i]<<endl;
		// }
	// }
	for(int i = 0;i<count.size(); i++){
		if(count[i] > cell_filter){
			// cout<<"count["<<i<<"] : "<<count[i]<<endl;
			map->data[i] = 100;
			obstacle_point.x = (i % map->info.width) * R + map->info.origin.position.x;
			obstacle_point.y = int(i / map->info.width) * R + map->info.origin.position.y;
			obstacle_point.z = 0;
			// obstacle_point.x = cloud->points[i].x;
			// obstacle_point.y = cloud->points[i].y;
			// obstacle_point.z = 0;
			
			obstacle_cell.cells.push_back(obstacle_point);
			printf("////////////////////////////////////////cell\n");              /////////////////////////////
		}
	}

}

void expand_circle(int x0, int y0, int radius, nav_msgs::OccupancyGrid *map){

	int x = radius;
	int y = 0;
	int err = 0;
	
		// cout<<endl;
		// cout<<"x0 : "<<x0<<endl;
		// cout<<"W/R : "<<W/R<<endl;
		// cout<<"y0 : "<<y0<<endl;
		// cout<<"H/R : "<<H/R<<endl;
		// cout<<"radius : "<<radius<<endl;
		while(x >= y){
			int num1 = (x0+x)+(y0+y)*map->info.width;
			int num2 = (x0+y)+(y0+x)*map->info.width;
			int num3 = (x0-y)+(y0+x)*map->info.width;
			int num4 = (x0-x)+(y0+y)*map->info.width;
			int num5 = (x0-x)+(y0-y)*map->info.width;
			int num6 = (x0-y)+(y0-x)*map->info.width;
			int num7 = (x0+y)+(y0-x)*map->info.width;
			int num8 = (x0+x)+(y0-y)*map->info.width;
			
			if(0<=num1 && num1<map->data.size()){
				map->data[num1] = 100;
			}
			if(0<=num2 && num2<map->data.size()){
				map->data[num2] = 100;
			}
			if(0<=num3 && num3<map->data.size()){
				map->data[num3] = 100;
			}
			if(0<=num4 && num4<map->data.size()){
				map->data[num4] = 100;
			}
			if(0<=num5 && num5<map->data.size()){
				map->data[num5] = 100;
			}
			if(0<=num6 && num6<map->data.size()){
				map->data[num6] = 100;
			}
			if(0<=num7 && num7<map->data.size()){
				map->data[num7] = 100;
			}
			if(0<=num8 && num8<map->data.size()){
				map->data[num8] = 100;
			}

//				// cout<<"(x0+x)+(y0+y)*map->info.width : "<<(x0+x)+(y0+y)*map->info.width<<endl;
//				// cout<<"(x0+x) : "<<x0+x<<", (y0+y) : "<<y0+y<<endl;
//				map->data[(x0+x)+(y0+y)*map->info.width] = 100;
//				// cout<<"(x0+y)+(y0+x)*map->info.width : "<<(x0+y)+(y0+x)*map->info.width<<endl;
//				// cout<<"(x0+y) : "<<x0+y<<", (y0+x) : "<<y0+x<<endl;
//				map->data[(x0+y)+(y0+x)*map->info.width] = 100;
//				// cout<<"(x0-y)+(y0+x)*map->info.width : "<<(x0-y)+(y0+x)*map->info.width<<endl;
//				// cout<<"(x0-y) : "<<x0-y<<", (y0+x) : "<<y0+x<<endl;
//				map->data[(x0-y)+(y0+x)*map->info.width] = 100;
//				// cout<<"(x0-x)+(y0+y)*map->info.width : "<<(x0-x)+(y0+y)*map->info.width<<endl;
//				// cout<<"(x0-x) : "<<x0-x<<", (y0+y) : "<<y0+y<<endl;
//				map->data[(x0-x)+(y0+y)*map->info.width] = 100;
//				// cout<<"(x0-x)+(y0-y)*map->info.width : "<<(x0-x)+(y0-y)*map->info.width<<endl;
//				// cout<<"(x0-x) : "<<x0-x<<", (y0-y) : "<<y0-y<<endl;
//				map->data[(x0-x)+(y0-y)*map->info.width] = 100;
//				// cout<<"(x0-y)+(y0-x)*map->info.width : "<<(x0-y)+(y0-x)*map->info.width<<endl;
//				// cout<<"(x0-y) : "<<x0-y<<", (y0-x) : "<<y0-x<<endl;
//				map->data[(x0-y)+(y0-x)*map->info.width] = 100;
//				// cout<<"(x0+y)+(y0-x)*map->info.width : "<<(x0+y)+(y0-x)*map->info.width<<endl;
//				// cout<<"(x0+y) : "<<x0+y<<", (y0-x) : "<<y0-x<<endl;
//				map->data[(x0+y)+(y0-x)*map->info.width] = 100;
//				// cout<<"(x0+x)+(y0-y)*map->info.width : "<<(x0+x)+(y0-y)*map->info.width<<endl;
//				// cout<<"(x0+x) : "<<x0+x<<", (y0-y) : "<<y0-y<<endl;
//				map->data[(x0+x)+(y0-y)*map->info.width] = 100;
//			}
				
			y+=1;
			err += 1 + 2*y;
			if(2*(err-x)+1>0){
				x -= 1;
				err += 1 -2*x;
			}
		}
}

void expandObstacle(nav_msgs::OccupancyGrid *map_in){
	// cout<<"out.info.height : "<<out.info.height<<endl;
	// cout<<"out.info.width : "<<out.info.width<<endl;
	nav_msgs::OccupancyGrid out;

	out = *map_in;
	
	vector<int> x_list;
	vector<int> y_list;
	for(int xi=0; xi<(int)out.info.height; xi++){
		for(int yi=0; yi<(int)out.info.width; yi++){
			// if the cell is LETHAL
			// cout<<"SADASDADAS"<<endl;
			// cout<<"xi : "<<xi<<endl;
			// cout<<"yi : "<<yi<<endl<<endl;
			if(out.data[xi+out.info.width*yi]!=0){
				// expand the LETHAL cells with respect to the circle radius
				// cout<<"out.["<<xi+out.info.width*yi<<"]"<<endl;
				// cout<<"xi : "<<xi<<endl;
				// cout<<"yi : "<<yi<<endl<<endl;
				x_list.push_back(xi);
				y_list.push_back(yi);
				// expand_circle(xi, yi, 0.20/R, map_in);
				// expand_circle(0, 0, int(0.3/R), map_in);
				// cout<<"out.["<<xi+out.info.width*yi<<"]"<<endl;
				
				// for(litr=expanded_circle.begin(); litr!=expanded_circle.end(); litr++){
					// int x=xi+litr->i, y=yi+litr->j;
					// if(x>=0 && x<(int)local_map.info.height && 	y>=0 && y<(int)local_map.info.width
						// && map_in.data[xi+map_in.info.width*yi]>local_map.data[x+map_in.info.width*y]){
						// local_map.data[x+map_in.info.width*y]=map_in.data[xi+map_in.info.width*yi];
			}
		}
	}
	// for(int i=0;i<x_list.size();i++){
		// if(y_list[i]==0){
			// cout<<"x_list["<<i<<"] : "<<x_list[i]<<endl;
			// cout<<"y_list["<<i<<"] : "<<y_list[i]<<endl;
		// }
	// }
	for(int i=0;i<x_list.size();i++){
		expand_circle(x_list[i], y_list[i], int(RADIUS/R), &out);
	}

	*map_in = out;

}

float rotation_x(float x, float y, float theta){                      //座標に合わせて地図をの座標を動かす
	float x_r;
	// if(theta >= 0){
	x_r = cos(theta) * x - sin(theta) * y;
	// }
	// else{
		// x_r = cos(theta) * x - sin(theta) * y;
	// }
	return x_r;
}

float rotation_y(float x, float y, float theta){
	float y_r;
	// if(theta >= 0){
	y_r = sin(theta) * x + cos(theta) * y;
	// }
	// else{
		// y_r = -1 * sin(theta) * x + cos(theta) * y;
	// }
	return y_r;
}

void create_local_map(float x, float y, float theta, nav_msgs::GridCells obs_grid, nav_msgs::OccupancyGrid* loc_map){                  //どこを走っているかを算出する

	nav_msgs::OccupancyGrid out;

	out = *loc_map;
	
	vector<int8_t>::iterator mit;
	for(mit = out.data.begin(); mit != out.data.end(); mit++){
		*mit = 0;
	}
	for(size_t i = 0; i < obs_grid.cells.size(); i++){
		// cout<<"obs_grid.cells.size() : "<<obs_grid.cells.size()<<endl;
		// cout<<"i : "<<i<<endl;
		//int xi = int((rotation_x(obs_grid.cells[i].x-x, obs_grid.cells[i].y-y, -1*theta) - out.info.origin.position.x) / R);
		// int xi = int((rotation_x(obstacle_points.points[i].x-x, obstacle_points.points[i].y-y, theta) - out.info.origin.position.x) / R);
		// cout<<"xi : "<<xi<<endl;
		//int yi = int((rotation_y(obs_grid.cells[i].x-x, obs_grid.cells[i].y-y, -1*theta) - out.info.origin.position.y) / R);
		// int yi = int((rotation_y(obstacle_points.points[i].x-x, obstacle_points.points[i].y-y, theta) - out.info.origin.position.y) / R);
		// cout<<"yi : "<<yi<<endl;
		int xi = int((rotation_x(obs_grid.cells[i].x-x, obs_grid.cells[i].y-y, -1*theta) - out.info.origin.position.x) / R);
		// int xi = int((rotation_x(obstacle_points.points[i].x-x, obstacle_points.points[i].y-y, theta) - out.info.origin.position.x) / R);
		// cout<<"xi : "<<xi<<endl;
		int yi = int((rotation_y(obs_grid.cells[i].x-x, obs_grid.cells[i].y-y, -1*theta) - out.info.origin.position.y) / R);
		if((0 <= xi && xi < local_W/R) && (0 <= yi && yi < local_H/R)){
			int num = xi + yi * out.info.width;
			// cout<<"num : "<<num<<endl;
			out.data[num] = 100;
		}
		
	}
	
	// cout<<"AASFDASFMAS*NMDSF"<<endl;
	// expandObstacle(&out);

	*loc_map = out;

}

void create_3d_local_map(float x, float y, float theta, pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output){
	cout<<"input->points.size() : "<<input->points.size()<<endl;
	
	
	output->points.clear();

	size_t input_size = input->points.size();
	
	cout <<"x(3d) : "<<x<<endl; 
	cout <<"y(3d) : "<<y<<endl; 
	for(size_t i = 0; i < input_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = rotation_x(input->points[i].x - x, input->points[i].y - y, -1*theta);
		temp_point.y = rotation_y(input->points[i].x - x, input->points[i].y - y, -1*theta);
		temp_point.z = input->points[i].z;
		
		if((-10.0 <= temp_point.x && temp_point.x <= 10.0) && (-10.0 <= temp_point.y && temp_point.y <= 10.0)){
			output->points.push_back(temp_point);
		}
	}
}


void map2matching_base_link(float x, float y, float theta, tf::TransformBroadcaster br){
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, 0.0));
	tf::Quaternion q;
	q.setRPY(0, 0, theta);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/global_obstacle_map", "/matching_base_link2"));
	// cout<<"tf pub!!!"<<endl;	
	// cout<<"x : "<<x<<endl;
	// cout<<"y : "<<y<<endl;
	// cout<<"z : "<<0.0<<endl;
	// cout<<"theta : "<<theta<<endl;
	// geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);

	// geometry_msgs::TransformStamped map_local_tf;

	// map_local_tf.header.stamp = ros::Time::now();
	// map_local_tf.header.frame_id = "/map";
	// map_local_tf.child_frame_id = "/local_map";

	// map_local_tf.transform.translation.x = x;
	// map_local_tf.transform.translation.y = y;
	// map_local_tf.transform.translation.z = 0.0;
	// map_local_tf.transform.rotation = q;
	
	// br.sendTransform(map_local_tf);

}


int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "global_obstacle_map_creater");
	ros::NodeHandle n;

	string filename = "/home/amsl/sq2_d_kan_around_3d_aft_crcl.pcd";            //change
	n.getParam("map2d/obs4path", filename);
	// string filename = "test_velodyne.pcd";
	// string filename = "map_Dkan_test.pcd";
	//string filename = "map_0_hagi.pcd";
	// string filename = "map_ds.pcd";
	// string filename = "map_2.pcd";
	// string filename = "map_0.pcd";
	// string filename = "/home/amsl/ikuchalle.pcd";
	// string filename = "/home/amsl/obs_map_after_crcl.pcd";
	// string filename = "/home/amsl/obs_map_curvature_after_crcl.pcd";
	// string filename = "/home/amsl/obs_map_new_ikuta_after_crcl.pcd";
	// string filename = "/home/amsl/obs_cloud_data_robosym2016_aft_crcl.pcd";
	
	// string filename = "/home/amsl/obs_map_ohshimizu_after_crcl.pcd";
	// string filename = "/home/amsl/obs_map_tsukuba_all_machi.pcd";
	// string filename = "/home/amsl/obs_map_tsukuba_final_after_crcl.pcd";
	// string filename = "/home/amsl/obs_map_tsukuba_final_after_crcl_ark.pcd";
	if(pcl::io::loadPCDFile (filename, *cloud_data) == -1){                     //地図を読み込む
		PCL_ERROR("Couldn't read file '%s'", filename.c_str());
		printf("can't read pcd file name\n");          //change
		return -1;
	}

	// string filename2 = "/home/amsl/map_tsukuba_all2.pcd";
	// if(pcl::io::loadPCDFile (filename2, *global_map_data) == -1){
		// PCL_ERROR("Couldn't read file '%s'", filename2.c_str());
		// return -1;
	// }
	
	// pcl::io::loadPCDFile ("test_velodyne.pcd", *cloud_data);

	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	// viewer = simpleVis(cloud_data);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_ptr(&temp_cloud);
	// viewer = simpleVis(temp_cloud);
    
	// viewer.showCloud(cloud);
	
	// ros::Subscriber sub1 = n.subscribe("/ekf_lcl", 1 , test_odom_callback);
	
	// ros::Subscriber sub12 = n.subscribe("/ekf_ndt", 1 , ndt_odom_callback);
	//ros::Subscriber sub12 = n.subscribe("/ekf_DgaussAndNDT", 1 , ndt_odom_callback);                        //ekf_dgaussandndtをsub         ////////////////////////////////ここを変更
	ros::Subscriber sub12 = n.subscribe("/lcl_sq_ndt", 1 , ndt_odom_callback);                        //ekf_dgaussandndtをsub         ////////////////////////////////ここを変更

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/output", 1);
	ros::Publisher pub_dash = n.advertise<sensor_msgs::PointCloud2>("/output_dash", 1);
	
	ros::Publisher pub2 = n.advertise<nav_msgs::OccupancyGrid>("/ob_map", 1);
	ros::Publisher pub3 = n.advertise<nav_msgs::OccupancyGrid>("/local_map_static", 1);
	ros::Publisher pub3_expand = n.advertise<nav_msgs::OccupancyGrid>("/local_map_static/expand", 1);
	ros::Publisher pub4 = n.advertise<nav_msgs::GridCells>("/obs_cell", 1);
	ros::Publisher pub5 = n.advertise<sensor_msgs::PointCloud2>("/3d_local_map", 1);
	
	// cloud2.header.frame_id = "/world";
	// cloud2_dash.header.frame_id = "/world";

	
	// obstacle_map.header.frame_id = "/global_obstacle_map";
	obstacle_map.header.frame_id = "/map";
	obstacle_map.data.resize(int(W / R) * int(H / R));
	obstacle_map.info.width = int(W / R);
	obstacle_map.info.height = int(H / R);
	obstacle_map.info.resolution = R;
	obstacle_map.info.origin.position.x = (min_x - W) / 2.0;
	obstacle_map.info.origin.position.y = (min_y - H) / 2.0;
	
	// obstacle_cell.header.frame_id = "/obstacle_cell";
	obstacle_cell.header.frame_id = "/map";
	obstacle_cell.cell_width = R;
	obstacle_cell.cell_height = R;


	// local_map.header.frame_id = "/static_local_map";
	local_map.header.frame_id = "/centerlaser2_";
	local_map.data.resize(int(local_W / R) * int(local_H / R));                 //R=0.1で拡張しexpandに
	local_map.info.width = int(local_W / R);
	local_map.info.height = int(local_H / R);
	local_map.info.resolution = R;
	local_map.info.origin.position.x = (min_x - local_W) / 2.0;
	local_map.info.origin.position.y = (min_y - local_H) / 2.0;
	
	local_map_expand = local_map;

	// remove_point_by_heght(cloud_data);
	// remove_point_by_heght(cloud_data, &obstacle_map);
	PointXYZ_to_PointCloud2(&cloud_data, &cloud2);                //pcdデータをcloud2（ros）へ変換

	create_obstacle_map(cloud_data, &obstacle_map);               //全体のmapを作成
	// expandObstacle(&obstacle_map);

	float ndt_x = x_init;
	float ndt_y = y_init;
	// float theta = 83.0 / 180.0 *M_PI;
	float ndt_theta = theta_init;
	
	tf::TransformBroadcaster br;
	
	obstacle_map.header.stamp = ros::Time::now();
	pub2.publish(obstacle_map);
	
	obstacle_cell.header.stamp = ros::Time::now();
	pub4.publish(obstacle_cell);

	ros::Rate loop_rate(20);
	while(ros::ok()){
		// obstacle_cell.header.stamp = ros::Time::now();
		// pub4.publish(obstacle_cell);
		// viewer->spinOnce (100);
		// boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		if(ndt_sub_flag){
			ndt_x = ndt_odom.pose.pose.position.x + x_init;
			// x = odom.pose.pose.position.y + y_init;
			// cout<<"x : "<<x<<endl;
			ndt_y = ndt_odom.pose.pose.position.y + y_init;
			// y = odom.pose.pose.position.x + x_init;
			// cout<<"y : "<<y<<endl;

			// x_global = rotation_x(x, y, theta_init) + x_init;
			// y_global = rotation_x(x, y, theta_init) + y_init;
			
			// tf::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
			// q = q.normalize();
			// cout<<"q : "<<q.z<<endl;
			// theta = tf::getYaw(q) + theta_init;
			ndt_theta = ndt_odom.pose.pose.orientation.z + theta_init;
			//cout<<"theta() : "<<theta<<endl;
			// sub_flag =false;
		}
		cout<<"ndt_x : "<<ndt_x<<endl;
		cout<<"ndt_y : "<<ndt_y<<endl;
		cout<<"ndt_theta : "<<ndt_theta<<endl;
		
		// create_local_map(x, y, theta, obstacle_cell, &local_map);
		create_local_map(ndt_x, ndt_y, ndt_theta, obstacle_cell, &local_map);                 //localmapを作成 現在の座標をつかう
		create_local_map(ndt_x, ndt_y, ndt_theta, obstacle_cell, &local_map_expand);

		// create_3d_local_map(ndt_x, ndt_y, ndt_theta, global_map_data, local_map_3d);
		// PointXYZ_to_PointCloud2(&local_map_3d, &local_map_3d_2);
		// local_map_expand = local_map;
		expandObstacle(&local_map_expand);
		// x += 0.1;
		// y += 0.1;
		// theta +=  -1.0 / 180.0 * M_PI;
		
		// cloud2.header.frame_id = "/velodyne";
		// cloud2.header.stamp = ros::Time::now();
		// pub.publish(cloud2);
		
		// obstacle_map.header.stamp = ros::Time::now();
		// pub2.publish(obstacle_map);
		// visu_obstacle_map.header.stamp = ros::Time::now();
		// pub2_visu.publish(visu_obstacle_map);
		
		local_map.header.stamp = ros::Time::now();
		local_map_expand.header.stamp = ros::Time::now();
		pub3.publish(local_map);
		pub3_expand.publish(local_map_expand);
		
		// local_map_3d_2.header.frame_id = "/velodyne";
		// local_map_3d_2.header.frame_id = "/map";
		// local_map_3d_2.header.stamp = ros::Time::now();
		// pub5.publish(local_map_3d_2);

		// cout<<"obstacle_cell : "<<obstacle_cell<<endl;
		//obstacle_cell.header.stamp = ros::Time::now();
		//pub4.publish(obstacle_cell);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
