#include <ros/ros.h>
#include "local_planner/mpc.h"

#include <thread>         // std::this_thread::sleep_for
#include <chrono> 
#include <cstdio>
#include <iostream>
#include <stdio.h>
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"  
#include <visualization_msgs/MarkerArray.h>
#include "tf2_msgs/TFMessage.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;

ros::Subscriber _obs_sub;
ros::Publisher cmd_vel_pub, path_cur_pub, pose_pub, path_predict_pub, point_pub, obs_vis_pub, obs_predict_pub, point_predict_pub, obst_collision_pub, sysdataPub_ ;

vector<double> control_output;
vector<double> predict_states;
vector<double> control_output_last;
vector<double> predict_states_last;
Eigen::VectorXd current_state(6);
vector<double> desired_state;
vector<double> obs_state(31*2, 10); // static risk point
vector<double> dyn_obs_state; // Dynamic obstacle prediction trajectory
double odom_flag, obs_flag_;
unique_ptr<Mpc> mpc_ptr;
bool mpc_success = true;
vector<double> obst_collision_;

class communicate_tool
{
private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_subscribe;
  ros::Subscriber static_obs_sub;
  ros::Subscriber obs_predict_sub;
 
  double msg_x, msg_y, msg_z, msg_vx, msg_vy, msg_vz;
  geometry_msgs::PoseStamped current_pose_stamped;
  nav_msgs::Path path_current;

public:
  communicate_tool(const ros::NodeHandle& nh);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
  void static_obs_point_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void predict_obs_point_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  ~communicate_tool();
};

communicate_tool::communicate_tool(const ros::NodeHandle &nh) : nh_(nh)
{
    odom_subscribe = nh_.subscribe("/base_pose_ground_truth", 10, &communicate_tool::odom_callback, this);

    static_obs_sub = nh_.subscribe("/dynamic_map/static_obs_point", 10, &communicate_tool::static_obs_point_callback, this);

    obs_predict_sub = nh_.subscribe("/obs_predict_point", 10, &communicate_tool::predict_obs_point_callback, this);

    path_cur_pub = nh_.advertise<nav_msgs::Path>("/path_cur", 10);

    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/pose_cur", 10);

    path_predict_pub = nh_.advertise<nav_msgs::Path>("/path_predict", 10);

    point_pub = nh_.advertise<geometry_msgs::PointStamped>("/point", 10);

    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    obs_vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("/obs_vis", 1);

    obs_predict_pub = nh_.advertise<nav_msgs::Path>("/obs_predict", 10); 

    point_predict_pub = nh_.advertise<sensor_msgs::PointCloud2>("/point_predict", 10);

    obst_collision_pub = nh_.advertise<sensor_msgs::PointCloud2>("/obst_collision", 10);

    sysdataPub_ = nh_.advertise<std_msgs::Float32MultiArray>("/sys_data", 10);
}

communicate_tool::~communicate_tool()
{
}

void communicate_tool::odom_callback(const nav_msgs::Odometry::ConstPtr&  msg){
	msg_x=msg->pose.pose.position.x;//position
  msg_y=msg->pose.pose.position.y;
  msg_z=msg->pose.pose.position.z;
  msg_vx=msg->twist.twist.linear.x;//Vx
  msg_vy=msg->twist.twist.linear.y;//Vy
  msg_vz=msg->twist.twist.linear.z;

  tf2::Quaternion odom_quat( msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3 m(odom_quat);
  m.getRPY(roll, pitch, yaw);

  current_state << msg_x, msg_y, yaw, msg_vx, msg_vy, msg_vz; // x, y, phi, u, v, r

	// trajectory vis
  current_pose_stamped.pose.position.x = msg_x;
  current_pose_stamped.pose.position.y = msg_y;
  current_pose_stamped.pose.position.z = msg_z;
  current_pose_stamped.pose.orientation = msg->pose.pose.orientation;
  current_pose_stamped.header.stamp=ros::Time::now();
  current_pose_stamped.header.frame_id="world";
  path_current.poses.push_back(current_pose_stamped);
  path_current.header.frame_id = "world";
  path_current.header.stamp = ros::Time::now();

  pose_pub.publish(current_pose_stamped);
  path_cur_pub.publish(path_current);
	// printf("msg_vx:%f, msg_vy:%f, msg_vz:%f \n", msg_vx , msg_vy , msg_vz );
	odom_flag = 1 ;
}

void communicate_tool::static_obs_point_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);  
  pcl::fromROSMsg(*msg, *cloud_);  
  if (cloud_->points.size()){
    for (int i = 0; i < cloud_->points.size() ; i++) {
      obs_state[2*i] = cloud_->points[i].x;
      obs_state[2*i+1] = cloud_->points[i].y;
    }
    obs_flag_ = 1;
  }
}

void communicate_tool::predict_obs_point_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);  
  dyn_obs_state.clear();
  pcl::fromROSMsg(*msg, *cloud_);  
  if (cloud_->points.size()){
    for (int i = 0; i < cloud_->points.size() ; i++) {
      dyn_obs_state.push_back(cloud_->points[i].x);
      dyn_obs_state.push_back(cloud_->points[i].y);
    }
  }
}

void obst_collision_vis(vector<double> obst_collision, const ros::Publisher& pub){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建一个PCL点云  

  cloud->width = obst_collision.size()/2+1;  
  cloud->height = 1;  
  cloud->is_dense = true;  
  cloud->points.resize(cloud->width * cloud->height); 

  if (obst_collision.size()){
    for(int j = 0; j < obst_collision.size()/2; ++j){
      if(obst_collision[2*j] >= 12){
        obst_collision[2*j] = 100;
        obst_collision[2*j+1] = 100;
      }
      if (obst_collision[2*j] != 100){
        cloud->points[j].x = obst_collision[2*j];
        cloud->points[j].y = obst_collision[2*j+1];
      }
    }
  }
  cloud->points[obst_collision.size()/2].x = obs_state[2];
  cloud->points[obst_collision.size()/2].y = obs_state[3];

  sensor_msgs::PointCloud2 obst_point_;
  pcl::toROSMsg(*cloud, obst_point_);  
  obst_point_.header.frame_id = "world";  
  obst_point_.header.stamp = ros::Time::now();
  pub.publish(obst_point_);
}

void vis_Obs(ros::Publisher &_Ellipses_vis_pub, vector<double> obst_state)
{
  // ellipse
  visualization_msgs::MarkerArray ellipse_vis;

  visualization_msgs::Marker shape_vis;
  shape_vis.header.frame_id = "world";
  shape_vis.header.stamp = ros::Time::now();
  shape_vis.ns = "obs";
  shape_vis.type = visualization_msgs::Marker::CYLINDER;
  shape_vis.action = visualization_msgs::Marker::ADD;
  shape_vis.lifetime = ros::Duration(0.15);

  // number
  visualization_msgs::Marker text_vis;
  text_vis.header.frame_id = "world";
  text_vis.header.stamp = ros::Time::now();
  text_vis.ns = "text";
  text_vis.action = visualization_msgs::Marker::ADD;
  text_vis.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_vis.lifetime = ros::Duration(0.1);
  text_vis.scale.z = 0.3;
  text_vis.color.r = 1;
  text_vis.color.g = 1;
  text_vis.color.b = 1;
  text_vis.color.a = 1;

  shape_vis.color.a = 0.9;
  shape_vis.color.r = 1;
  shape_vis.color.g = 0;
  shape_vis.color.b = 0;

  shape_vis.id = 1;
  shape_vis.pose.orientation.x = 0.0;
  shape_vis.pose.orientation.y = 0.0;
  shape_vis.pose.orientation.z = 0.0;
  shape_vis.pose.orientation.w = 1.0;
  shape_vis.pose.position.x = obst_state[2];
  shape_vis.pose.position.y = obst_state[3];
  shape_vis.pose.position.z = -0.0;
  shape_vis.scale.x = 0.342*2;
  shape_vis.scale.y = 0.342*2;
  shape_vis.scale.z = 0.075;

  text_vis.id = 1;

  // publish
  ellipse_vis.markers.push_back(shape_vis);
  ellipse_vis.markers.push_back(text_vis);

  _Ellipses_vis_pub.publish(ellipse_vis);

  nav_msgs::Path obs_predict;
  geometry_msgs::PoseStamped obs_pose_stamped;
  for (int i = 0; i < (obst_state.size()-2)/2; i++) {
    obs_pose_stamped.pose.position.x = obst_state[2+2*i];
    obs_pose_stamped.pose.position.y = obst_state[3+2*i];
    obs_predict.poses.push_back(obs_pose_stamped);
  }
  obs_predict.header.frame_id = "world";
  obs_predict.header.stamp = ros::Time::now();
  obs_predict_pub.publish(obs_predict); 
  obs_predict.poses.clear();
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh;

    odom_flag = 0;
    obs_flag_ = 0;
    communicate_tool tool(nh);

    mpc_ptr.reset(new Mpc());

    ROS_INFO("local_planner start.");
    geometry_msgs::Twist cmd_vel;


    ros::Rate rate(10); // 10 Hz
    while(!(odom_flag && obs_flag_)){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("waiting !");
    }
    ROS_INFO("strat working !");

    desired_state = {15, 0, 0};
    geometry_msgs::PointStamped this_point_stamped;
    this_point_stamped.header.stamp = ros::Time::now();
    this_point_stamped.header.frame_id = "world";
    this_point_stamped.point.x = desired_state[0];
    this_point_stamped.point.y = desired_state[1];
    this_point_stamped.point.z = 0;
    point_pub.publish(this_point_stamped);

    this_point_stamped.header.stamp = ros::Time::now();
    this_point_stamped.header.frame_id = "world";
    this_point_stamped.point.x = current_state[0];
    this_point_stamped.point.y = current_state[1];
    this_point_stamped.point.z = 0;
    point_pub.publish(this_point_stamped);

    while (ros::ok()) {
        auto solve_start = ros::Time::now();
        mpc_ptr->getObst(obs_state, dyn_obs_state);

        mpc_success = mpc_ptr->solve(current_state, desired_state);

        auto solve_end = ros::Time::now();

        if(mpc_success){
            control_output = mpc_ptr->getAllU();
            predict_states = mpc_ptr->getPredictX();
            control_output_last = control_output;
        }else{
            for (int i=0;i<control_output_last.size()-3;++i){
                control_output_last[i] = control_output_last[i+3];
            }

            control_output[0] = control_output_last[0];
            control_output[2] = control_output_last[2];
        }
        // Check target
        bool goal = mpc_ptr->is_target(current_state, desired_state);
        if(goal)
        {
            cmd_vel.linear.x = 0 ;
            cmd_vel.angular.z = 0 ;
            cout << "Work Done!" << endl;
        }
        else
        {
            cmd_vel.linear.x = control_output[0];
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = control_output[2];
        }
        cmd_vel_pub.publish(cmd_vel);

        nav_msgs::Path path_predict;
        geometry_msgs::PoseStamped predict_pose_stamped;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        cloud->width = 31;  
        cloud->height = 1;  
        cloud->is_dense = true;  
        cloud->points.resize(cloud->width * cloud->height); 

        for (int i = 0; i < predict_states.size()/2; i += 1) {
            predict_pose_stamped.pose.position.x = predict_states[2*i];
            predict_pose_stamped.pose.position.y = predict_states[2*i + 1];
            path_predict.poses.push_back(predict_pose_stamped);

            cloud->points[i].x = predict_states[2*i];
            cloud->points[i].y = predict_states[2*i + 1];
            cloud->points[i].z = 0.25 ;
        }
        path_predict.header.frame_id = "world";
        path_predict.header.stamp = ros::Time::now();
        path_predict_pub.publish(path_predict); 
        path_predict.poses.clear();

        sensor_msgs::PointCloud2 point_predict_;
        pcl::toROSMsg(*cloud, point_predict_);  
        point_predict_.header.frame_id = "world";  
        point_predict_.header.stamp = ros::Time::now();
        point_predict_pub.publish(point_predict_);

        vector<double> obst_collision_add = mpc_ptr->getObst_collision();
        for(int j =0; j < obst_collision_add.size(); j++){
          obst_collision_.push_back(obst_collision_add[j]);
        }
        obst_collision_vis(obst_collision_, obst_collision_pub);
        dyn_obs_state.clear();

        // sys_data
        std_msgs::Float32MultiArray sys_data;
        sys_data.data.push_back((solve_end - solve_start).toSec());
        sys_data.data.push_back(current_state[0]);
        sys_data.data.push_back(current_state[1]);
        sysdataPub_.publish(sys_data);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}