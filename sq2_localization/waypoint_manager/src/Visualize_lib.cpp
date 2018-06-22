#include "waypoint_manager/Visualize_lib.h"
#include <iostream>

using namespace std;

Visualization::Visualization(std_msgs::ColorRGBA rgba_in,
							 uint action_in,
							 float scale_in,
							 const string& name_in,
							 const string& frame_id_in)
{
	marker.color = rgba_in;
	marker.action = action_in;
	marker.scale.x=scale_in;
	marker.scale.y=scale_in;
	marker.scale.z=scale_in;
	marker.ns = name_in;
	marker.header.frame_id = frame_id_in;
}
///////////////////////////////////////////////////////
/////-----------Visualize function-------------////////
///////////////////////////////////////////////////////
void Visualization::markerCircle(visualization_msgs::Marker& circle,
								float r, geometry_msgs::PoseStamped state)
{
	
	geometry_msgs::Point pt;
	int n_t = 30;
	float radian = 2 * M_PI / n_t;
	for (int j = 0; j < n_t; j++ ) {
		pt.x = cos(radian * j)*r + state.pose.position.x;
		pt.y = sin(radian * j)*r + state.pose.position.y;
		pt.z = state.pose.position.z-0.1;
		circle.points.push_back(pt);
	}
	pt.x = cos(radian * 0)*r + state.pose.position.x;
	pt.y = sin(radian * 0)*r + state.pose.position.y;
	pt.z = state.pose.position.z-0.1;
	circle.points.push_back(pt);
}


void Visualization::convertPath2MarkerLine(	nav_msgs::Path path,
								visualization_msgs::Marker& line, 
								int id)
{
	line = marker;
	line.header.stamp=ros::Time::now();
	line.id = id; // set each ID number of lines
	line.type=visualization_msgs::Marker::LINE_STRIP;

	line.pose.position.x=0.0;
	line.pose.position.y=0.0;
	line.pose.position.z=0.0;
	line.pose.orientation.x = 0.0;
	line.pose.orientation.y = 0.0;
	line.pose.orientation.z = 0.0;
	line.pose.orientation.w = 0.0;

	
	
	geometry_msgs::Point pt;
	for(uint i=0; i<path.poses.size(); i++){
		pt.x = path.poses[i].pose.position.x;
		pt.y = path.poses[i].pose.position.y;
		pt.z = path.poses[i].pose.position.z-0.1;
		line.points.push_back(pt);
	}
}

void Visualization::potint2MarkerCircles(visualization_msgs::Marker& circle, 
										geometry_msgs::PoseStamped state,
										int id, float r)
{
	circle = marker;
	circle.header.stamp=ros::Time::now();
	circle.id = id; // set each ID number of lines
	circle.type=visualization_msgs::Marker::LINE_STRIP;

	circle.pose.position.x=0.0;
	circle.pose.position.y=0.0;
	circle.pose.position.z=0.0;
	circle.pose.orientation.x = 0.0;
	circle.pose.orientation.y = 0.0;
	circle.pose.orientation.z = 0.0;
	circle.pose.orientation.w = 0.0;
	
	markerCircle(circle, r, state);
}

void Visualization::convertPath2MarkerCircles(nav_msgs::Path path,
											visualization_msgs::MarkerArray& circles, 
											int id, float r)
{
	visualization_msgs::Marker circle;
	circle = marker;
	circle.header.stamp=ros::Time::now();
	circle.id = id; // set each ID number of lines
	circle.type=visualization_msgs::Marker::LINE_STRIP;

	circle.pose.position.x=0.0;
	circle.pose.position.y=0.0;
	circle.pose.position.z=0.0;
	circle.pose.orientation.x = 0.0;
	circle.pose.orientation.y = 0.0;
	circle.pose.orientation.z = 0.0;
	circle.pose.orientation.w = 0.0;

	
	for(uint i=0; i<40; i++){
		circle.points.clear();
		if(path.poses.size() > i){
			geometry_msgs::PoseStamped state;
			state.pose = path.poses[i].pose;
			markerCircle(circle, r, state);
		}
		
		circle.id = id+i;
		circles.markers.push_back(circle);
	}
}

void Visualization::convertPath2MarkerLineOffset(nav_msgs::Path path,
													visualization_msgs::Marker& line, 
													int id, float offset)
{
	line = marker;
	line.header.stamp=ros::Time::now();
	line.id = id; // set each ID number of lines
	line.type=visualization_msgs::Marker::LINE_STRIP;

	line.pose.position.x=0.0;
	line.pose.position.y=0.0;
	line.pose.position.z=0.0;
	line.pose.orientation.x = 0.0;
	line.pose.orientation.y = 0.0;
	line.pose.orientation.z = 0.0;
	line.pose.orientation.w = 0.0;

	
	
	geometry_msgs::Point pt;
	for(uint i=0; i<path.poses.size(); i++){
		pt.x = path.poses[i].pose.position.x;
		pt.y = path.poses[i].pose.position.y;
		pt.z = path.poses[i].pose.position.z + offset;
		line.points.push_back(pt);
	}
}

void Visualization::txtMarker(	const string& txt_in,
								geometry_msgs::PoseStamped goal,
								visualization_msgs::Marker& mk, 
								int id)
{
	mk = marker;
	mk.header.stamp=ros::Time::now();

	mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    mk.id = id;
	mk.text = txt_in;
	mk.pose = goal.pose;
	mk.pose.position.z += 1;
	mk.scale.x = 1.2;
	mk.scale.y = 1.2;
	mk.scale.z = 1.2;
	mk.color.r = 1.0;
	mk.color.g = 1.0;
	mk.color.b = 1.0;
	mk.color.a = 1.0;
	mk.lifetime = ros::Duration();
}
