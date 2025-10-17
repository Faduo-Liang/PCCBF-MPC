/**
  ******************************************************************************
  * @file    pseudo_odom.cc
  * @author  Alex Liu 
  * @version V1.0.0
  * @date    2024/01/07
  * @brief   twist cmd test tool for MobiRo @ tib_k331
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include <cmath>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>

namespace robot_test {
//
ros::Publisher _pseudo_odom_pub;
geometry_msgs::Pose _robot_pose;
geometry_msgs::Twist _robot_twist;
ros::Time _last_update_stamp;
bool _has_robot_state = false;

double Unwrap(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

void ModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr &msg) {
  for (int i = 0; (int)i < msg->name.size(); i++) {
    if (msg->name.at(i) != "jackal") continue;
    if (!_has_robot_state) {
      _last_update_stamp = ros::Time::now();
      _robot_pose = msg->pose.at(i);
      _has_robot_state = true;
      return;
    }
    // update time stamp
    double dt = (ros::Time::now() - _last_update_stamp).toSec();
    _last_update_stamp = ros::Time::now();
    // calculate velocity by differential
    double dx = msg->pose.at(i).position.x - _robot_pose.position.x;
    double dy = msg->pose.at(i).position.y - _robot_pose.position.y;

    double last_yaw = tf::getYaw(_robot_pose.orientation);
    double yaw = tf::getYaw(msg->pose.at(i).orientation);
    double delta_yaw = Unwrap(yaw - last_yaw);

    Eigen::Vector2d map_vel = {dx / ((dt + 1e-8)), dy / (dt + 1e-8)};  // add small value to avoid singularity
    Eigen::Matrix2d rot_mat;
    rot_mat << cos(yaw), sin(yaw), -sin(yaw), cos(yaw);
    Eigen::Vector2d body_vel = rot_mat * map_vel;

    double yaw_rate = delta_yaw / (dt + 1e-8);
    // update pose and velocity
    _robot_pose = msg->pose.at(i);
    _robot_twist.linear.x = body_vel(0);
    _robot_twist.linear.y = body_vel(1);
    _robot_twist.angular.z = yaw_rate;
  } 
}

void OdomPubCallback(const ros::TimerEvent &/* event */) {
  if (!_has_robot_state) return;
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = _last_update_stamp;
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose = _robot_pose;
  odom_msg.twist.twist = _robot_twist;
  _pseudo_odom_pub.publish(odom_msg);
}

}  // namespace robot_test

int main(int argc, char **argv) {
	ros::init(argc, argv, "localization_node");
	ros::NodeHandle nh;

  auto model_states_sub = nh.subscribe(
      "/gazebo/model_states", 5, &robot_test::ModelStatesCallback);

  auto odom_pub_timer = nh.createTimer(
      ros::Duration(0.1), &robot_test::OdomPubCallback);

  robot_test::_pseudo_odom_pub = nh.advertise<nav_msgs::Odometry>(
      "base_pose_ground_truth", 5, true);

  ros::spin();
  return 0;
}
