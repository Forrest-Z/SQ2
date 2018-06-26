#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include "sq_waypoint_manager/GPStoXY.h"
#include "sq_waypoint_manager/read_wp_file.h"
#include "sq_waypoint_manager/Visualize_lib.h"
using namespace std;

//wp_mngr.cpp, init_visu.cpp, read_wp_file.cppのheader_frameを使用するグローバル座標系の名前に変更すること//
const string header_frame("/map");

//-----------------原点座標入力---------------------//
//海上自衛隊浦郷 2012/12/10 追浜実験
//double start_latlon[2] = {139.6363793,35.3165939};
//旧研究室	ikutachallenge 2014
double start_latlon[2] = {35.613415,139.549145};
//いつものスタートマンホール
//double start_latlon[2] = {35.614784,139.549492};
//左折矢印の根元
//double start_latlon[2] = {35.614756,139.54951};//今までの
//double start_latlon[2] = {35.614745 139.549445};//0211try2
//double start_latlon[2] = {35.614784,139.54949};//0211try1
//駅伝
//double start_latlon[2] = {35.613921, 139.549194};

//tkb 1
//double start_latlon[2] = {36.086091,140.110657};
//tkb start
//double start_latlon[2] = {36.079341,140.115387};
//2014tkb 原点

double tkb_start_latlon[2] = {36.079131,140.115302};
template <class T>
void getParam(ros::NodeHandle &n, string param, T &val, bool* flag)
{
    string str;
    if(!n.hasParam(param)){
        std::cout << param << " don't exist." << std::endl;
        *flag=true;
    }

    if(!n.getParam(param, str)){
        std::cout << "NG" << std::endl;
        *flag=true;
    }
    std::stringstream ss(str);
    T rsl;
    ss >> rsl;
    val = rsl;
    std::cout << param << " = " << str << std::endl;
}

string selectWaypointFile(ros::NodeHandle &n)
{
    string wp_dir_path, wp_txt;
    bool error_flag=false;
    getParam(n, "wp_dir_path", wp_dir_path, &error_flag);
    getParam(n, "wp_txt", wp_txt, &error_flag);
	string waypoint_file_name = wp_dir_path + wp_txt;
	return waypoint_file_name;
}

///////////////////////////////////////////////////////
//////-------Input waypoint data from txt------////////
///////////////////////////////////////////////////////
void inputFromTxt(nav_msgs::Path& waypoints, const string& file_name)
{
	ifstream ifs(file_name.c_str());
	string buf;
	int wp_count=0;
	double x,y,yaw;
	geometry_msgs::Point pt;
	if( !ifs ){
		ROS_FATAL( "Error:Non file!!!!!---------Is this file's name correct?\n------->%s\n",file_name.c_str());
		exit(0);
	}else{
		while(ifs && getline(ifs, buf)) {
			istringstream is( buf );
			is>>x>>y>>yaw;
			pt.x=x;
			pt.y=y;
			pt.z=0;
			
			geometry_msgs::PoseStamped point;
			point.pose.position.x=pt.x;
			point.pose.position.y=pt.y; 
			point.pose.position.z=pt.z;
			
			cout<<"count:"<<wp_count<<"\tx = "<<pt.x<<"\ty = "<<pt.y<<"\tyaw = "<< yaw <<endl;
			waypoints.poses.push_back(point);
			wp_count++;
		}
		cout<<"Input Complete!\t"<<wp_count<<endl;
	}
}

void convertWaypoints(sq_waypoint_manager::InitialValue init_state, 
						nav_msgs::Path waypoint, 
						nav_msgs::Path& converts)
{
	GPStoXY* gps_xy;
	/////////////////////////初期位置(0, 0)///////////////////////////////
	//---------GPStoXY(lon, lat)---------------//
	printf("Lat:%4.8f \tLon:%4.8f\n",init_state.InLat,init_state.InLon);
	if(init_state.LocateFlag)
        gps_xy = new GPStoXY(tkb_start_latlon[1], tkb_start_latlon[0]);	
	else
        gps_xy = new GPStoXY(start_latlon[1], start_latlon[0]);	
	converts.poses.clear();		
	/////////////////////////目標位置(x, y)//////////////////////////
	geometry_msgs::PoseStamped p_g;
	p_g.header.frame_id=header_frame;
					
	for(int i = 0 ; i < (int)waypoint.poses.size() ; i++){
		float xx = waypoint.poses[i].pose.position.x;
		float yy = waypoint.poses[i].pose.position.y;
		gps_xy->Convert( yy, xx);
		
		cout<<"asdfafx = "<<gps_xy->X()<<"\tasdfy = "<<gps_xy->Y()<<endl;
		
		geometry_msgs::Pose pt;
		pt.position.x = gps_xy->X();
		pt.position.y = gps_xy->Y();
		pt.position.z = 0;
		p_g.pose.position = pt.position;
		p_g.pose.orientation.x = 0;
		p_g.pose.orientation.y = 0;
		p_g.pose.orientation.z = 0;
		p_g.pose.orientation.w = 0;
		
		converts.poses.push_back(p_g);
	}
}

void readWaypointInXY(nav_msgs::Path& waypoints, const string& file_name, std::vector<WpMode>& wp_mode)
{
	int wp_count=0;
	// double x,y,mode;
	double x,y;
	geometry_msgs::Point pt;
	WpMode mode;
	string buf;
	ifstream ifs(file_name.c_str());
	
    if( !ifs ){
		ROS_FATAL( "Error:Non file!!!!!---------Is this file's name correct?\n------->%s\n",file_name.c_str());
		exit(0);
	}else{
		while(ifs && getline(ifs, buf)) {
			istringstream is( buf );
			is>>x>>y>>mode.trace>>mode.speed>>mode.arm>>mode.ekf;                 //ここでfileから読み込み xから読み込み
			pt.x=x;
			pt.y=y;
			pt.z=0;//running mode (trace line or avoid)
			
			geometry_msgs::PoseStamped point;
			point.pose.position.x=pt.x;
			point.pose.position.y=pt.y; 
			point.pose.position.z=pt.z;
			
			cout<<"wp_count:"<<wp_count<<"\tx = "<<pt.x<<"\ty = "<<pt.y<<"\tz = "<<pt.z<<endl;
			cout<<"\ttrace:"<<mode.trace<<"\tspeed:"<<mode.speed<<"\tarm:"<<mode.arm<<"\tekf : "<<mode.ekf<<endl;
			waypoints.poses.push_back(point);
			wp_mode.push_back(mode);
			wp_count++;
		}
		cout<<"Read Complete!\t"<<wp_count<<endl<<endl;
	}
}

void waypointAngleSet(nav_msgs::Path& waypoints)
{
	for(unsigned int i=0; i<waypoints.poses.size()-1; i++){
		float angle=atan2(waypoints.poses[i+1].pose.position.y - waypoints.poses[i].pose.position.y,
				waypoints.poses[i+1].pose.position.x - waypoints.poses[i].pose.position.x);
		geometry_msgs::Quaternion angle_quat = tf::createQuaternionMsgFromYaw(angle);
		waypoints.poses[i].pose.orientation=angle_quat;
	}
}

string IntToString(int number)
{
	stringstream ss;
	ss << number;
	return ss.str();
}

void setMarkers(nav_msgs::Path waypoints,
				visualization_msgs::Marker& lines,
				visualization_msgs::Marker& points,
				visualization_msgs::Marker& fin,
				visualization_msgs::MarkerArray& txts)
{
	std_msgs::ColorRGBA rgba_in;
	rgba_in.r = 0.0;
	rgba_in.g = 0.8;
	rgba_in.b = 0.9;
	rgba_in.a = 0.8;
	Visualization* marker_txt = new Visualization(rgba_in, ADD, 2.0, "txtSet",header_frame);
	float yaw = 0;
	geometry_msgs::Point pt;
	int waypoint_n = waypoints.poses.size();
	for(int i=0; i<waypoint_n; i++) {
		pt.x = waypoints.poses[i].pose.position.x;
		pt.y = waypoints.poses[i].pose.position.y;
		pt.z = 0;
		
		geometry_msgs::PoseStamped point;
		point.pose.position.x = pt.x;
		point.pose.position.y = pt.y; 
		point.pose.position.z = pt.z;
		
		visualization_msgs::Marker txt1;
		marker_txt->txtMarker(IntToString(i), point, txt1, i);
		
		txts.markers.push_back(txt1);
		waypoints.poses.push_back(point);
		lines.points.push_back(pt);
		points.points.push_back(pt);
	}
	geometry_msgs::Quaternion pose_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);
	fin.pose.position = pt;
	fin.pose.orientation = pose_quat;
}
