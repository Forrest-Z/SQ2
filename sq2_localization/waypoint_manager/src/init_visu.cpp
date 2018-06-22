#include "waypoint_manager/init_visu.h"
#include <string>

//wp_mngr.cpp, init_visu.cpp, read_wp_file.cppのheader_frameを使用するグローバル座標系の名前に変更すること//
const std::string header_frame("/map");

void initVisualizationMsgs1(visualization_msgs::Marker& lines)
{
	lines.header.frame_id = header_frame;
	lines.header.stamp = ros::Time::now();
	lines.ns = "lines";
	//lines.action = visualization_msgs::Marker::ADD;
	lines.id = 0; // set each ID number of lines
	lines.type = visualization_msgs::Marker::LINE_STRIP;
		
	lines.pose.position.x=0;
	lines.pose.position.y=0;
	lines.pose.position.z=0;
	lines.pose.orientation.x = 0.0;
	lines.pose.orientation.y = 0.0;
	lines.pose.orientation.z = 0.0;
	lines.pose.orientation.w = 0.0;

	lines.scale.x=0.2;
	lines.scale.y=0.2;
	lines.scale.z=0.2;

	lines.color.g=0.0;
	lines.color.r=0.0;
	lines.color.b=1;
/*	lines.color.g=1.0;
	lines.color.r=1.0;
	lines.color.b=0.9;
*/	lines.color.a=0.9;
}

void initVisualizationMsgs2(visualization_msgs::Marker& points)
{
	points.header.frame_id=header_frame;
	points.header.stamp=ros::Time::now();
	points.ns="points";
	//points.action = visualization_msgs::Marker::ADD;
	points.id=1; // set each ID number of lines
	points.type=visualization_msgs::Marker::POINTS;
		
	points.pose.position.x=0;
	points.pose.position.y=0;
	points.pose.position.z=0.7;
	points.pose.orientation.x = 0.0;
	points.pose.orientation.y = 0.0;
	points.pose.orientation.z = 0.0;
	points.pose.orientation.w = 0.0;

	points.scale.x=0.7;
	points.scale.y=0.7;
	points.scale.z=0.7;

	//points.color.g=0.0;
	//points.color.r=0.0;
	//points.color.b=1;
	//points.color.a=0.8;
	points.color.g=0.0;
	points.color.r=1.0;
	points.color.b=1.0;
	points.color.a=0.7;
}

void initVisualizationMsgs3(visualization_msgs::Marker& now)
{
	now.header.frame_id=header_frame;
	now.header.stamp=ros::Time::now();
	now.ns="now";
	//now.action = visualization_msgs::Marker::ADD;
	now.id=2; // set each ID number of lines
	now.type=visualization_msgs::Marker::CUBE_LIST;
		
	now.pose.position.x=0;
	now.pose.position.y=0;
	now.pose.position.z=0.7;
	now.pose.orientation.x = 0.0;
	now.pose.orientation.y = 0.0;
	now.pose.orientation.z = 0.0;
	now.pose.orientation.w = 0.0;

	now.scale.x=0.4;
	now.scale.y=0.4;
	now.scale.z=0.7;

	now.color.a=0.8;
	now.color.g=0.8;
	now.color.r=1;
	now.color.b=0;
}

void initVisualizationMsgs4(visualization_msgs::Marker& points)
{
	points.header.frame_id=header_frame;
	points.header.stamp=ros::Time::now();
	points.ns="goal";
	//points.action = visualization_msgs::Marker::ADD;
	points.id=2; // set each ID number of lines
	points.type=visualization_msgs::Marker::ARROW;
	points.scale.x=1.2;
	points.scale.y=0.2;
	points.scale.z=0.2;

	points.color.a=0.8;
	points.color.g=0.2;
	points.color.r=0.9;
	points.color.b=0.0;
}
