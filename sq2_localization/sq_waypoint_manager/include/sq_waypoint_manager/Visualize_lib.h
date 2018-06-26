//
//
//

#ifndef VISUALIZATION_CLASS_H
#define VISUALIZATION_CLASS_H

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <boost/thread.hpp>
#include <string>
#define ADD		0
#define DELETE	2

using namespace std;

class Visualization{
private:
	void markerCircle(visualization_msgs::Marker& circle,
								float r, geometry_msgs::PoseStamped state);
	Visualization(Visualization&);
	
public:
	Visualization(std_msgs::ColorRGBA rgba_in, uint action_in,float scale_in,const string& name_in, const string& frame_id_in);
	void convertPath2MarkerLine(nav_msgs::Path path,
								visualization_msgs::Marker& line, 
								int id);
	
	void potint2MarkerCircles(visualization_msgs::Marker& circle, 
										geometry_msgs::PoseStamped state,
										int id, float r);
	
	void convertPath2MarkerCircles(nav_msgs::Path path,
									visualization_msgs::MarkerArray& circles, 
									int id, float r);
	
	void convertPath2MarkerLineOffset(nav_msgs::Path path,
										visualization_msgs::Marker& line, 
										int id, float offset);
	
	void txtMarker(	const string& txt_in,
					geometry_msgs::PoseStamped goal,
					visualization_msgs::Marker& mk, 
					int id);
	
	// 管理しているMarker
	visualization_msgs::Marker marker;
};


#endif //LOCAL_MAP_ORGANAIZER

