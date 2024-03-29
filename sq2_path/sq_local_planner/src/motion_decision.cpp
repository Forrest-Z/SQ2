#include "ros/ros.h"
#include <sq_trajectory_generation/TrajectoryGeneration.h>
#include <sq_trajectory_generation/Velocity.h>
#include <sq_trajectory_generation/VelocityArray.h>
#include <cstdlib>
#include <tf/transform_listener.h>
#include <sq_local_planner/Velocity.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <knm_tiny_msgs/Velocity.h>

#define HALF_PI M_PI*0.5
#define STOP 0
#define TURN 2
#define STOP_TIME 2*10 //wait sec * loop rate

using namespace std;

const string header_frame("/map");
const string robot_frame("/centerlaser2_");
const float Vmax=0.9;                              //vmaxの設定
const float ANGULARmax=1.0; 
const float ANGULARmin=0.1;
const int MaxStop=40; //4[s]
const float Max_ndist=3.0;
float THRESH_ANG=0.25*M_PI;//HALF_PI

ros::Publisher vel_pub;
int time_count=0;
bool v_a_flag=false, cheat_flag=false, joy_flag=false, t_wp_flag=false, em_flag=false, rp_flag=false;
geometry_msgs::Twist cheat_vel;
boost::mutex v_array_mutex_;
sq_trajectory_generation::VelocityArray g_v_array; 
geometry_msgs::PoseStamped t_wp;
int wp_mode=TURN;
float n_dist=0;
bool stop_Tflag=false; //stop flag when a obstacle is on track line

float calcRelativeAngle(geometry_msgs::PoseStamped& wp, geometry_msgs::PoseStamped& robo)
{
	float dx=wp.pose.position.x-robo.pose.position.x;
	float dy=wp.pose.position.x-robo.pose.position.y;
	float wp_ang;
	if (fabs(dx)<=0.01 && dy>=0) wp_ang = HALF_PI; //vertical
	else if (fabs(dx)<=0.01 && dy<0) wp_ang = -HALF_PI;
	else{
		wp_ang=atan(dy/dx);
		if (dx<0 && dy>0) wp_ang += M_PI;
		else if (dx<0 && dy<0) wp_ang += -M_PI;
	} 

	//cout<<"wp-robo:"<<wp_ang*180.0/M_PI<<endl;
	//cout<<"relative:"<<(wp_ang-robo.pose.orientation.z)*180.0/M_PI<<endl;
	return wp_ang-robo.pose.orientation.z;
}

//void setStopCommand(local_planner::Velocity& cmd_vel)
void setStopCommand(geometry_msgs::Twist& cmd_vel)
{
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
}

void setTurnCommand(sq_local_planner::Velocity& cmd_vel,
		geometry_msgs::PoseStamped& wp,
		geometry_msgs::PoseStamped& robo)
{
	float turn_angular = 0.3;
	//	float turn_angular = 0.2;
	float turn_dir = 1;
	if (calcRelativeAngle(wp, robo)<0) turn_dir=-1;
	cmd_vel.op_linear = 0;
	cmd_vel.op_angular = turn_dir * turn_angular;
	cout<<"now turn"<<endl;
}

// void commandDecision(	trajectory_generation::VelocityArray& v_a,
// 						local_planner::Velocity& cmd_vel)
void commandDecision(	sq_trajectory_generation::VelocityArray& v_a,
		geometry_msgs::Twist& cmd_vel)
{
	// float hoge = 0.6*10.0/0.4;
	float hoge = 15;
	int ind = hoge+time_count;
	if((int)v_a.vel.size() > ind){
		//cmd_vel.header.stamp = ros::Time::now();
		// cmd_vel.op_linear = v_a.vel[ind].op_linear;
		// cmd_vel.op_angular = -v_a.vel[ind].op_angular;
		cmd_vel.linear.x = v_a.vel[ind].op_linear;

		//cmd_vel.angular.z = -v_a.vel[ind].op_angular; 
		//cmd_vel.angular.z = v_a.vel[ind].op_angular;            ///paramchange 
		//cmd_vel.angular.z = v_a.vel[ind].op_angular * 0.55;            ///paramchange 
		cmd_vel.angular.z = v_a.vel[ind].op_angular * 0.45;            ///paramchange 
	}else{
		//cmd_vel.op_linear = 0;
		//cmd_vel.op_angular = 0;
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = 0;
	}
	//cout<<"lin1 = "<<cmd_vel.op_linear<<"\tang1 = "<<cmd_vel.op_angular<<endl;
}

inline bool checkTF(tf::StampedTransform& transform, geometry_msgs::PoseStamped& robo, tf::TransformListener& listener)          //mapにおけるrobotの座標を算出
{
	try{
		//listener.waitForTransform(header_frame, robot_frame, ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform(header_frame, robot_frame, ros::Time(0), transform);
		robo.pose.position.x=transform.getOrigin().x();
		robo.pose.position.y=transform.getOrigin().y();
		robo.pose.position.z=0;
		robo.pose.orientation.z = tf::getYaw(transform.getRotation());
		// float angle = tf::getYaw(transform.getRotation());
		// robo.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,angle);
		return true;
	}
	catch (tf::TransformException ex){
		cout<<"waiting for global and robot frame relation"<<endl;
		return false;
	}
}

void JoyCallback(const sensor_msgs::JoyConstPtr& msg){
	joy_flag = true;
}

void CheatCallback(const geometry_msgs::TwistPtr& msg){
	cheat_vel.linear.x = msg->linear.x;
	cheat_vel.angular.z = msg->angular.z;
	cheat_flag = true;
}

void StopTraceFlagCallback(const std_msgs::BoolConstPtr& msg)
{
	stop_Tflag=msg->data;
}

void EmergencyCallback(const std_msgs::BoolConstPtr& msg)
{
	em_flag = msg->data;
	rp_flag = true;
}

void WpModeCallback(const std_msgs::Int32ConstPtr& msg){
	wp_mode = msg->data;
}

void NormalDistCallback(const std_msgs::Float32ConstPtr& msg)
{
	n_dist=msg->data;
}

void TargetWpCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	t_wp=*msg;
	t_wp_flag=true;
}

void vArrayCallback(const sq_trajectory_generation::VelocityArrayConstPtr& msg)
{
	boost::mutex::scoped_lock(v_array_mutex_);
	g_v_array=*msg;
	v_a_flag=true;
}

bool _fin_goal_arrival=false;
void FinGoalArrivalCallback(const std_msgs::BoolConstPtr& msg)
{
	_fin_goal_arrival = msg->data;
}

void MotionDecision()
{
	ros::NodeHandle n;
	ros::Subscriber cntl_sub       = n.subscribe("/joy", 10, JoyCallback);
	ros::Subscriber cheat_sub       = n.subscribe("/t_frog/cheat_velocity", 10, CheatCallback);
	ros::Subscriber v_array_sub    = n.subscribe("/plan/velocity_array", 10, vArrayCallback);              //pathの赤線
	ros::Subscriber twp_sub        = n.subscribe("/target/pose", 10, TargetWpCallback);                    //目指しているwaypoint
	ros::Subscriber norm_dist_sub  = n.subscribe("/waypoint/normal_dist", 10, NormalDistCallback);         //waypoint間の直線との距離
	ros::Subscriber wp_mode_sub    = n.subscribe("/wp_mode", 10, WpModeCallback);
	ros::Subscriber emrgcy_sub     = n.subscribe("/emergency_stop", 1, EmergencyCallback);
	ros::Subscriber stop_Tflag_sub = n.subscribe("/stop_for_trace", 1, StopTraceFlagCallback);
	ros::Subscriber sub_stop_flag = n.subscribe("/fin_goal_arrival", 1, FinGoalArrivalCallback);           //最後のwaypointに到達

	// vel_pub = n.advertise<local_planner::Velocity>("tinypower/command_velocity", 10);
	vel_pub = n.advertise<geometry_msgs::Twist>("/t_frog/cmd_vel", 10);
	//for debug
	ros::Publisher mode_pub = n.advertise<std_msgs::Int32>("running_mode", 1);

	// int stop_count=0, em_stop_count=0;
	int stop_count=0;
	bool turn_flag=false;
	bool normal_flag=true;
	bool stop_flag=false;
	// float relative_ang=0;
	//local_planner::Velocity cmd_vel;
	geometry_msgs::Twist cmd_vel;
	tf::StampedTransform transform;
	tf::TransformListener listener;
	geometry_msgs::PoseStamped robo;
	std_msgs::Int32 mode;
	mode.data=0;//0:stop ,1: normal, 2: turn

	ros::Rate loop_rate(10);
	while(ros::ok()){
		//get robot position
		if(rp_flag){
			bool tf_flag = checkTF(transform, robo, listener);
			// if (t_wp_flag && tf_flag){ //normal state
			if (t_wp_flag && tf_flag && !cheat_flag){ //normal state
				// if (t_wp_flag && tf_flag && !joy_flag){ //normal state
				sq_trajectory_generation::VelocityArray v_array; 
				//check whether robot would return the track line
				// relative_ang = calcRelativeAngle(t_wp, robo);
				// cout<<"relative_ang:"<<relative_ang<<endl;
				// if (n_dist>Max_ndist && fabs(relative_ang)>THRESH_ANG){
				// turn_flag=true;
				// }
				if (!turn_flag){                 //基本false
					if (v_a_flag){               //pathを受信していたらtrue
						boost::mutex::scoped_lock(v_array_mutex_);
						v_array = g_v_array;
					}
					if (v_array.vel.size()){// path is generated
						normal_flag=true;
						cout<<"run"<<endl;
					}
					else { // path is not generated
						//when path is not generated for stop_count [s],
						//robot turns to generate path
						// if (stop_count>MaxStop && wp_mode==TURN){
						// turn_flag=true;
						// cout<<"stop_count_turn"<<endl;
						// }
						// else{
						stop_flag=true;
						cout<<"no path stop"<<endl;
						// }
						stop_count++;
					}
				}

				// set velocity and angular //
				cout<<"stop_flag: "<<stop_flag<<"turn_flag: "<<turn_flag<<"em_flag: "<<em_flag<<endl;
				if (_fin_goal_arrival){                //ゴールに着いた場合
					cout<<"stop"<<endl;
					setStopCommand(cmd_vel);           //速度を0にする
					mode.data=4;
					_fin_goal_arrival = false;
				}
				else if (em_flag){                         //緊急停止信号受信の場合
					cout<<"emergency stop"<<endl<<endl;
					setStopCommand(cmd_vel);
					mode.data=3;
					// em_stop_count++;
					// if (em_stop_count>STOP_TIME){           //しばらくしたら走行可能に
					// 	em_flag=false;
					// 	em_stop_count=0;
					// }
				}
				else if (stop_flag){              //pathが出ていない場合
					cout<<"stop"<<endl;
					setStopCommand(cmd_vel);
					mode.data=4;
					stop_flag=false;
				}
				else if (stop_Tflag){
					cout<<"stop to trace line"<<endl;
					setStopCommand(cmd_vel);
					mode.data=5;
				}
				// else if (turn_flag){
				// cout<<"turn"<<endl;
				// setTurnCommand(cmd_vel, t_wp, robo);
				// mode.data=2;
				// stop_count=0;
				// turn_flag=false;
				// }
				else if (normal_flag){                   //pathで速度が決められた場合
					commandDecision(v_array, cmd_vel);
					mode.data=1;
					stop_count=0;
					time_count=0;
					normal_flag=false;
				}
				else {                      //それ以外
					cout<<"unknown"<<endl;
					mode.data=2;
					setStopCommand(cmd_vel);
				}

				// safety //
				//if (cmd_vel.op_linear>Vmax) cmd_vel.op_linear=Vmax;                     //一定以上の速度、角度を制限
				//if (cmd_vel.op_angular>ANGULARmax) cmd_vel.op_angular=ANGULARmax;
				//else if (cmd_vel.op_angular<-ANGULARmax) cmd_vel.op_angular=-ANGULARmax;
				if (cmd_vel.linear.x>Vmax) cmd_vel.linear.x=Vmax;                     //一定以上の速度、角度を制限
				if (cmd_vel.angular.z>ANGULARmax) cmd_vel.angular.z=ANGULARmax;
				else if (cmd_vel.angular.z<-ANGULARmax) cmd_vel.angular.z=-ANGULARmax;

				// publish //
				vel_pub.publish(cmd_vel);                 //速度、角度をpub

				cout<<"mode: "<<mode.data<<endl;
				cout<<"lin = "<<cmd_vel.linear.x<<"\tang = "<<cmd_vel.angular.z<<endl<<endl;

				//v_a_flag=false;
				//t_wp_flag=false;
				tf_flag=false;
				stop_Tflag=false;
			}
				else if (cheat_flag){                   //手動の信号が受信された場合
					vel_pub.publish(cheat_vel);
					cheat_flag=false;
					// joy_flag=false;
				}
				else { //waiting for callback
					cout<<"-----------"<<endl;
					cout<<"v_arr:"<<v_a_flag<<endl;
					cout<<"t_wp:"<<t_wp_flag<<endl;
					cout<<"tf_flag:"<<tf_flag<<endl;
					cout<<"joy_flag:"<<joy_flag<<endl;
					cout<<"-----------"<<endl;
					cmd_vel.linear.x=0;
					cmd_vel.angular.z=0;
					cout<<"v:"<<cmd_vel.linear.x<<", ang: "<<cmd_vel.angular.z<<endl<<endl;
					vel_pub.publish(cmd_vel);
					mode.data=0;
				}
			}
			else{
				cmd_vel.linear.x = 0.0;
				cmd_vel.angular.z = 0.0;
				vel_pub.publish(cmd_vel);
			}

			mode_pub.publish(mode);
			time_count++;
			// rp_flag = false;
			ros::spinOnce();
			loop_rate.sleep();
		}
		}

		int main(int argc, char **argv)
		{
			ros::init(argc, argv, "motion_decision");
			MotionDecision();

			ROS_INFO("Killing now!!!!!");
			return 0;
		}
