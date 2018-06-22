//waypoint到達を判断するnode
//sub tf 

#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
// #include <rwrc16_msgs/GetGoalsId.h>
// #include <rwrc16_msgs/InitialValue.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <math.h>
#include <vector>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include "waypoint_manager/Visualize_lib.h"
// #include "waypoint_manager/GPStoXY.h"
#include "waypoint_manager/init_visu.h"
#include "waypoint_manager/read_wp_file.h"
#include <std_msgs/String.h>

#define ON 1
#define OFF 0

#define SLOW 0
#define NORMAL 1
#define FAST 2

#define NO_MOVE 0
#define LIFT 1
using namespace std;
//wp_mngr.cpp, init_visu.cpp, read_wp_file.cppのheader_frame
//を使用するグローバル座標系の名前に変更すること//
const string header_frame("/map");
const string robot_frame("/centerlaser3_");
// const string ongaku_command("gnome-terminal -e \"/home/amsl/AMSL_ros_pkg/l/script/beep.sh\" --geometry=0x0+0+0");
const float thresh_radius=0.5;
const float thresh_radius2=4.0;
const float thresh_dist=0.5;
const float thresh_parallel_dist=2.0;
const float thresh_normal_dist = 3.0;
bool fin_flag=false;

inline void pubMode(int mode, ros::Publisher& pub)
{
	std_msgs::Int32 msg_mode;
	msg_mode.data = mode;
	pub.publish(msg_mode);
}

geometry_msgs::Pose2D nearest(	geometry_msgs::Pose2D A, 
								geometry_msgs::Pose2D B, 
								//geometry_msgs::Pose2D P )
								geometry_msgs::Pose2D P,
								bool& flag)
{
	geometry_msgs::Pose2D a, b;
	double r;
	//a = (next wp - now wp),b = (robo pos - now_wp)
	a.x = B.x - A.x;
	a.y = B.y - A.y;
	b.x = P.x - A.x;
	b.y = P.y - A.y;
	
	r = (a.x*b.x + a.y*b.y) / (a.x*a.x + a.y*a.y);
	
	if( r<= 0 ){
		return A;
	}else if( r>=1 ){
		flag=true;
		return B;
	}else{
		geometry_msgs::Pose2D result;
		result.x = A.x + r*a.x;
		result.y = A.y + r*a.y;
		return result;
	}
}
float calcNormalDistToWpLine( geometry_msgs::PoseStamped& twp,                  //waypointを結んだ直線と現在地との距離
							  geometry_msgs::PoseStamped& pwp,
							  geometry_msgs::PoseStamped& robo)
{
	float dx=twp.pose.position.x-pwp.pose.position.x;
	float dy=twp.pose.position.y-pwp.pose.position.y;
	if (fabs(dx)<=0.01) return fabs(robo.pose.position.x-twp.pose.position.x);// vertical
	else if (fabs(dy)<=0.01) return fabs(robo.pose.position.y-twp.pose.position.y); //horizontal
	else{ //diagonal
		float slope= dy/dx;
		float intercept=-slope*pwp.pose.position.x+pwp.pose.position.y;
		return fabs(robo.pose.position.y-slope*robo.pose.position.x-intercept)/sqrt(1.0+slope*slope);
	}
}

void setCircleParameter(visualization_msgs::Marker& now, geometry_msgs::PoseStamped& traced_waypoint,
 geometry_msgs::PoseStamped& now_state,
 visualization_msgs::Marker& circle_dr,
 visualization_msgs::Marker& traced_circle_dr)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.8;
	rgba_in.g = 0.8;
	rgba_in.b = 0.0;
	rgba_in.a = 0.8;
	Visualization* marker_circle = new Visualization(rgba_in, ADD, 0.1, "area",header_frame);
    
    now_state.pose.position.x =	now.points[0].x;
    now_state.pose.position.y =	now.points[0].y;
    now_state.pose.position.z = now.points[0].z;
    
    traced_waypoint.header.frame_id=header_frame;
    traced_waypoint.pose.position.x = now.points[0].x;
    traced_waypoint.pose.position.y = now.points[0].y;
    traced_waypoint.pose.position.z = 0.5;
    
    marker_circle->potint2MarkerCircles(traced_circle_dr, now_state, 222, thresh_radius);
	traced_circle_dr.header.frame_id=header_frame;
}

void checkArrivalAtWp(		const geometry_msgs::PoseStamped& robo,       //下で得られたmatchingbaselinkとwaypointを使う
					const nav_msgs::Path& waypoints,
					int& wp_count)
{
	geometry_msgs::Pose2D P, A, B, H;
	//P: robot position, A:next waypoints
	P.x = robo.pose.position.x;
	P.y = robo.pose.position.y;
	A.x = waypoints.poses[wp_count+1].pose.position.x;
	A.y = waypoints.poses[wp_count+1].pose.position.y;
	B.x = waypoints.poses[wp_count].pose.position.x;
	B.y = waypoints.poses[wp_count].pose.position.y;
	//ロボットと次のwaypointの距離
	float dx = P.x - A.x;
	float dy = P.y - A.y;
	float dist = sqrt(dx*dx + dy*dy);

	//トラックラインに対する法線方向の距離
	bool flag=false;
	H = nearest(A, B, P, flag);
	//H = nearest(A, B, P);
	float normal_dist= (H.x-P.x)*(H.x-P.x)+(H.y-P.y)*(H.y-P.y) ;	
	float parallel_dist=(H.x-A.x)*(H.x-A.x)+(H.y-A.y)*(H.y-A.y);
	//次のwapointまでのトラックライン方向の距離
	float dist2 = sqrt(dist*dist*+normal_dist);
	// cout<<"distance to wp ="<<dist<<endl;
	//if (dist<thresh_radius || dist2<thresh_dist){
	if (dist<thresh_radius){
		if(wp_count >= (int)waypoints.poses.size()-2){              //waypoint到達を判定
			cout<<"goal";
			fin_flag=true;
		}else{
			cout<<"change!! : "<<wp_count+1<<endl;
			wp_count++;                                        //wayipointを変更
            // system(ongaku_command.c_str());
		}
	}
	// else if(dist<thresh_radius2 && flag){ //11/7version
		// cout<<"change!!"<<endl;
		// wp_count++;
	// }
	else if(parallel_dist<thresh_parallel_dist && normal_dist<thresh_normal_dist){ //11/7version kai
		cout<<"change!! : "<<wp_count+1<<endl;
		wp_count++;
	}
}

inline bool checkTF(tf::StampedTransform& transform, geometry_msgs::PoseStamped& robo,                     //ここでtfをsubする
					tf::TransformListener& listener)
{
	try{
		//listener.waitForTransform(header_frame, robot_frame, ros::Time(0), ros::Duration(1.0));
	    listener.lookupTransform(header_frame, robot_frame, ros::Time(0), transform);                //mapにmatchingbaselinkを追従させる
        //copy robot position
        robo.pose.position.x=transform.getOrigin().x();                                             //tfと座標の変換 matchingbaselinkの座標を得る
        robo.pose.position.y=transform.getOrigin().y();
        robo.pose.position.z=0;
        float angle = tf::getYaw(transform.getRotation());
        robo.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
		return true;
	}
	catch (tf::TransformException ex){
		// cout<<"waiting for global and robot frame relation"<<endl;
        ROS_ERROR("%s",ex.what());
		return false;
	}
}

int main(int argc, char** argv){
	
	ros::init(argc, argv, "rwrc16_waypoint_manager");
	ros::NodeHandle n;

	cout<<"waypoint server"<<endl;	
	// for visualization//
	ros::Publisher marker_pub        = n.advertise<visualization_msgs::Marker>("localization/waypoints", 1);
	ros::Publisher t_circle_pub      = n.advertise<visualization_msgs::Marker>("localization/traced_waypoint_circle", 1);
	ros::Publisher marker_pub4       = n.advertise<visualization_msgs::MarkerArray>("localization/waypoints/num", 1);
	//-------------------//
	ros::Publisher num_pub           = n.advertise<std_msgs::Int32>("/waypoint/now", 1);
	ros::Publisher target_pub        = n.advertise<geometry_msgs::PoseStamped>("/target/pose", 10);
	ros::Publisher prev_wp_pub       = n.advertise<geometry_msgs::PoseStamped>("/waypoint/prev", 10);
	ros::Publisher normal_dist_pub   = n.advertise<std_msgs::Float32>("/waypoint/normal_dist", 10);
	// ros::Publisher fin_pub           = n.advertise<std_msgs::Bool>("/tinypower/finish_flag", 10);
	ros::Publisher fin_pub           = n.advertise<std_msgs::Bool>("/fin_goal_arrival", 10);
	ros::Publisher pub_trace_mode    = n.advertise<std_msgs::Int32>("/wp_mode", 1);                //基本０？
	ros::Publisher pub_speed_mode    = n.advertise<std_msgs::Int32>("/speed_mode", 1);
	ros::Publisher pub_ekf_mode		 = n.advertise<std_msgs::Int32>("/ekf_mode", 1);
	// ros::Publisher pub_arm_lift_flag = n.advertise<std_msgs::String>("/action_signal", 1);
	ros::Publisher pub_arm_lift_flag = n.advertise<std_msgs::Bool>("/arm1_start", 1);
	tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener listener;                           //tfをsubしている
	
	//--------------initVisualizationMsgs-----------------//
	visualization_msgs::MarkerArray txts;
	visualization_msgs::Marker lines, points, now, fin;
	initVisualizationMsgs1(lines);
	initVisualizationMsgs2(points);
	now=points;
	initVisualizationMsgs3(now);
	initVisualizationMsgs4(fin);
	
	
    bool tf_flag = false;
	int wp_count = 0;
	int id_now = 1;
	nav_msgs::Path waypoints;
	geometry_msgs::PoseStamped traced_waypoint;
	
    //-----------read and set waypoints-----------------//
    string file_name;
    nav_msgs::Path input_latlon;
    file_name = selectWaypointFile(n);
	std::vector<WpMode> wp_mode;
    readWaypointInXY(waypoints,file_name, wp_mode);            //read_wp_file.cppへ
    setMarkers(waypoints, lines, points, fin, txts);//set the variables for visualization
    waypointAngleSet(waypoints);//calc relative angle between wp[i] and wp[i+1]
    marker_pub.publish(points);
	//------------------------------------------------//
    
    ros::Rate loop_rate(10); // ループの周期 
	while(ros::ok()){
		
        //get robot position on the global_coordinate(=map)
        tf::StampedTransform transform;
        geometry_msgs::PoseStamped robo;
		tf_flag=checkTF(transform, robo, listener);
        if(tf_flag){	
            checkArrivalAtWp(robo, waypoints,wp_count);//ここで現在のwaypointの計算をしている
            id_now = wp_count + 1;
            
            std_msgs::Int32 now_num;
            now_num.data = id_now;
            num_pub.publish(now_num);                 //現在のwaypointをpub
                
            //now.points[0]には次のwaypointの座標が入ってる
            // cout<<"pub:"<<id_now<<endl;
            geometry_msgs::Point pt;
            pt.x = waypoints.poses[wp_count+1].pose.position.x;
            pt.y = waypoints.poses[wp_count+1].pose.position.y;	
            pt.z = 0;
            // cout<<"count:"<<wp_count<<"	x:"<<pt.x<<"	y"<<pt.y<<endl;
            now.points.clear();                               //nowとは現在の目標waypoint座標
            now.points.push_back(pt);
			now.header.frame_id=header_frame;
            
            /////////////////Circle drawing!!!//////////////////////////
            geometry_msgs::PoseStamped now_state, prev_wp;
            visualization_msgs::Marker circle_dr, traced_circle_dr;
            setCircleParameter(now, traced_waypoint, now_state, circle_dr, traced_circle_dr);
            
            prev_wp.header.frame_id=header_frame;
            prev_wp.pose.position.x = waypoints.poses[wp_count].pose.position.x;
            prev_wp.pose.position.y = waypoints.poses[wp_count].pose.position.y;
            prev_wp.pose.position.z = 0.5;
            
			pubMode((int)wp_mode[id_now].trace, pub_trace_mode);
			pubMode((int)wp_mode[id_now].speed, pub_speed_mode);
			pubMode((int)wp_mode[id_now].ekf, pub_ekf_mode);
			
			if ((int)wp_mode[id_now].arm == LIFT){
				std_msgs::Bool arm_lift_flag;
				arm_lift_flag.data = true;
				pub_arm_lift_flag.publish(arm_lift_flag);
				// std_msgs::String arm_lift_flag;
				// arm_lift_flag.data = "rwrc15_waypoint_manager";
				// pub_arm_lift_flag.publish(arm_lift_flag);
			}

			std_msgs::Float32 normal_dist;
			normal_dist.data = calcNormalDistToWpLine(traced_waypoint, prev_wp, robo);
			normal_dist_pub.publish(normal_dist);   //waypointを結んだ直線と自己位置との距離
			prev_wp_pub.publish(prev_wp);           //直前に通過したwaypoint
            target_pub.publish(traced_waypoint);    //現在目指しているwaypoint
            
            //////////////publish visualization msg/////////////
            marker_pub.publish(now);
            // marker_pub.publish(circle_dr);
            t_circle_pub.publish(traced_circle_dr);
        }

        // publish msg to stop when infant reaches final waypoint// //2015/02/21 
        if (fin_flag){
			std_msgs::String arm_lift_flag;
			arm_lift_flag.data = "rwrc16_waypoint_manager";
			pub_arm_lift_flag.publish(arm_lift_flag);
            
			std_msgs::Bool fin_flag_msg;
            fin_flag_msg.data=true;
            fin_pub.publish(fin_flag_msg);
            std::cout<<"******************************goal***********************************************************************"<<std::endl;
        }
		
        ////////////publish visualization msg/////////////
        // marker_pub.publish(now);
        marker_pub.publish(points);
        marker_pub.publish(fin);
        marker_pub.publish(lines);
        marker_pub4.publish(txts);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	ros::spin();
	return 0;
}

