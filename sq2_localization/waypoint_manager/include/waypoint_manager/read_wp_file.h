#include <ros/ros.h>
#include <waypoint_manager/InitialValue.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

struct WpMode{
	double trace;
	double speed;
	double arm;
	double ekf;
};

template <class T>
void getParam(ros::NodeHandle &n, std::string param, T &val, bool* flag);
std::string selectWaypointFile(ros::NodeHandle &n);
void inputFromTxt(nav_msgs::Path& waypoints, const std::string& file_name);
void convertWaypoints(waypoint_manager::InitialValue init_state, nav_msgs::Path waypoint, nav_msgs::Path& converts);
void readWaypointInXY(nav_msgs::Path& waypoints, const std::string& file_name, std::vector<WpMode>& wp_mode);
void waypointAngleSet(nav_msgs::Path& waypoints);
void setMarkers(nav_msgs::Path waypoints,
				visualization_msgs::Marker& lines,
				visualization_msgs::Marker& points,
				visualization_msgs::Marker& fin,
				visualization_msgs::MarkerArray& txts);
