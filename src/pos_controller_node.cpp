/**********************************************************************************************************************
 *     Copyright (C)  2022  Toshiharu Tabuchi                                                                         *
 *                                                                                                                    *
 *     This file is part of MAVROS Offboard Multicopter Position Controller demo project.                             *
 *                                                                                                                    *
 *     MAVROS Offboard Multicopter Position Controller demo project is free software:                                 *
 *     you can redistribute it and/or modify                                                                          *
 *     it under the terms of the GNU General Public License as published by                                           *
 *     the Free Software Foundation, either version 3 of the License, or                                              *
 *     (at your option) any later version.                                                                            *
 *                                                                                                                    *
 *     MAVROS Offboard Multicopter Position Controller demo is distributed in the hope that it will be useful,        *
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of                                                 *
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                                  *
 *     GNU General Public License for more details.                                                                   *
 *                                                                                                                    *
 *     You should have received a copy of the GNU General Public License  along with                                  *
 *     MAVROS Offboard Multicopter Position Controller demo.                                                          *
 *     If not, see <http://www.gnu.org/licenses/>.                                                                    *
**********************************************************************************************************************/

#include "pos_controller_node.h"
#include <ros/ros.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Thrust.h>
#include <nav_msgs/Odometry.h>

#include "pos_controller/pid.h"
#include "std_srvs/SetBool.h"
#include <cmath>

bool hasWaypointBeenReached(Waypoint current_waypoint_, Waypoint target_waypoint_){
  auto pos_error = std::sqrt(std::pow(current_waypoint_.position[0]-target_waypoint_.position[0], 2) +
                            std::pow(current_waypoint_.position[1]-target_waypoint_.position[1], 2) +
                            std::pow(current_waypoint_.position[2]-target_waypoint_.position[2], 2));

  auto yaw_error = std::abs(current_waypoint_.yaw - target_waypoint_.yaw);

  ROS_DEBUG("pos_error: %f", pos_error);
  ROS_DEBUG("yaw_error: %f", yaw_error);

  return (pos_error < POSITION_ERROR_TOL) && (yaw_error < YAW_ERROR_TOL);
}

bool tunePid(pos_controller::pid::Request &req,
             pos_controller::pid::Response &res) {

  pos_parameters.kp = req.kp;
  pos_parameters.ki = req.ki;
  pos_parameters.kd = req.kd;

  pos_parameters.min_error_limit = req.min_error_limit;
  pos_parameters.max_error_limit = req.max_error_limit;

  pos_parameters.min_control_value = req.min_control_value;
  pos_parameters.max_control_value = req.max_control_value;

  posX_controller.updateParameters(pos_parameters);
  posY_controller.updateParameters(pos_parameters);
  ROS_INFO("PID parameters updated");
  return true;
}

bool start_test_cb(std_srvs::SetBool::Request &req,
                std_srvs::SetBool::Response &res){

  start_test = req.data;
  res.success = true;

  if(start_test){
    ROS_INFO("Starting position controller test");
    res.message = "Starting position controller test";
  }
  return true;

}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
  current_state = *msg;
}

void velocity_body_cb(const geometry_msgs::TwistStamped::ConstPtr & msg){
  auto thrust_error = vertical_speed_controller.compute_control_value(speed_target_vector.getZ(), msg->twist.linear.z);

  auto pitch = speed_vx_controller.compute_control_value(speed_target_vector.getX(), msg->twist.linear.x);
  auto roll = -speed_vy_controller.compute_control_value(speed_target_vector.getY(), msg->twist.linear.y);

  tf2::Quaternion quat;
  quat.setRPY(roll,pitch,target_waypoint.yaw);

  attitude_target_msg.header.stamp = ros::Time::now();
  attitude_target_msg.thrust = (hover_thrust + thrust_error);
  attitude_target_msg.type_mask = 0;
  attitude_target_msg.body_rate.x=0;
  attitude_target_msg.body_rate.y=0;
  attitude_target_msg.body_rate.z = 0;
  attitude_target_msg.orientation.x = quat.getX();
  attitude_target_msg.orientation.y = quat.getY();
  attitude_target_msg.orientation.z = quat.getZ();
  attitude_target_msg.orientation.w = quat.getW();

  ROS_DEBUG("Linear velocity: %f, %f, %f", msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
  ROS_DEBUG("Angular velocity: %f, %f, %f", msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){

  speed_target_vector.setX(posX_controller.compute_control_value(target_waypoint.position[0], msg->pose.pose.position.x));
  speed_target_vector.setY(posY_controller.compute_control_value(target_waypoint.position[1], msg->pose.pose.position.y));
  speed_target_vector.setZ(height_controller.compute_control_value(target_waypoint.position[2], msg->pose.pose.position.z));

  tf2::Quaternion quat;
  quat.setW(msg->pose.pose.orientation.w);
  quat.setX(msg->pose.pose.orientation.x);
  quat.setY(msg->pose.pose.orientation.y);
  quat.setZ(msg->pose.pose.orientation.z);

  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  current_waypoint.position[0] = msg->pose.pose.position.x;
  current_waypoint.position[1] = msg->pose.pose.position.y;
  current_waypoint.position[2] = msg->pose.pose.position.z;
  current_waypoint.yaw = yaw;

  ROS_DEBUG("Position: %f, %f, %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_DEBUG("Orientation: %f, %f, %f", roll, pitch, yaw);

}

void att_target_cb(const mavros_msgs::AttitudeTarget ::ConstPtr & msg){
  tf2::Quaternion quat;
  quat.setW(msg->orientation.w);
  quat.setX(msg->orientation.x);
  quat.setY(msg->orientation.y);
  quat.setZ(msg->orientation.z);

  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

//  ROS_INFO("Thrust: %f", msg->thrust);
  ROS_DEBUG("Target Orientation: %f | %f | %f ", roll, pitch, yaw);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_controller_node");
  ros::NodeHandle nh;

  ros::ServiceServer pid_tuning_service = nh.advertiseService("pos_controller/pid_tuner", tunePid);
  ros::ServiceServer start_test_service = nh.advertiseService("pos_controller/start_test", start_test_cb);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::Subscriber velocity_body_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 10, velocity_body_cb);
  ros::Subscriber local_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);
  ros::Publisher att_target_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
  ros::Subscriber att_target_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude", 10, att_target_cb);
  ros::Publisher thrust_pub = nh.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust", 10);

  // Waypoints
  uint32_t waypoint_index{0};

  waypoint_list[0].position[0] = 0;
  waypoint_list[0].position[1] = 0;
  waypoint_list[0].position[2] = 10;
  waypoint_list[0].yaw = 0;

  waypoint_list[1].position[0] = 10;
  waypoint_list[1].position[1] = 10;
  waypoint_list[1].position[2] = 20;
  waypoint_list[1].yaw = CONSTANT_PI/4;

  waypoint_list[2].position[0] = -20;
  waypoint_list[2].position[1] = -20;
  waypoint_list[2].position[2] = 5;
  waypoint_list[2].yaw = -CONSTANT_PI/4;

  // Load first waypoint
  target_waypoint = waypoint_list[waypoint_index];
  ROS_INFO("Navigation to Waypoint %d", waypoint_index);

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(controller_freq);

  // wait for FCU connection
  while(ros::ok() && !current_state.connected){
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();


  while(ros::ok()){
    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(1.0))){
      if( set_mode_client.call(offb_set_mode) &&
          offb_set_mode.response.mode_sent){
        ROS_INFO_ONCE("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if( !current_state.armed &&
          (ros::Time::now() - last_request > ros::Duration(1.0))){
        if( arming_client.call(arm_cmd) &&
            arm_cmd.response.success){
          ROS_INFO_ONCE("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    if(start_test){
      att_target_pub.publish(attitude_target_msg);

      if(hasWaypointBeenReached(current_waypoint, target_waypoint)){
        ROS_INFO("Waypoint %d reached!", waypoint_index);
        waypoint_index++;
        if(waypoint_index < NUMBER_OF_WAYPOINTS){
          ROS_INFO("Navigation to Waypoint %d", waypoint_index);
        }
        else{
          waypoint_index = 0;
//          start_test = false;
        }
        target_waypoint = waypoint_list[waypoint_index];
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
