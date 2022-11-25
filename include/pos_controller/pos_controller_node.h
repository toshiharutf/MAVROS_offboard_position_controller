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

#pragma once

#include "PID.h"
#include "tf2/LinearMath/Vector3.h"

#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

static constexpr int controller_freq{100};
static constexpr float hover_thrust{0.70};
static constexpr float POSITION_ERROR_TOL{0.5f};
static constexpr float YAW_ERROR_TOL{0.1f};

tf2::Vector3 speed_target_vector(0, 0, 0);
bool start_test{false};

mavros_msgs::AttitudeTarget attitude_target_msg;
mavros_msgs::State current_state;

struct Waypoint{
  float position[3];
  float yaw;
};

Waypoint current_waypoint;
Waypoint target_waypoint{.position = {0,0,0}, .yaw = 0.0f};
static constexpr uint32_t NUMBER_OF_WAYPOINTS{3};
Waypoint waypoint_list[NUMBER_OF_WAYPOINTS];


struct pidParameters height_controller_parameters = {
    .kp =  0.8f,
    .ki =  0.1f,
    .kd =  0.0f,
    .min_error_limit = -500.0f,
    .max_error_limit = 500.0f,
    .min_integral = -100.0f,
    .max_integral = 100.0f,
    .min_control_value =  -5.0f,
    .max_control_value = 5.0f,
    .cutoff_frequency = 10.0f
};

PID height_controller(1/controller_freq, height_controller_parameters);

struct pidParameters vertical_speed_controller_parameters = {
    .kp =  0.6f,
    .ki =  0.1f,
    .kd =  0.0f,
    .min_error_limit = -5.0f,
    .max_error_limit = 5.0f,
    .min_integral = -1000.0f,
    .max_integral = 1000.0f,
    .min_control_value =  -0.2f,
    .max_control_value = 0.3f,
    .cutoff_frequency = 10.0f
};

PID vertical_speed_controller(1/controller_freq, vertical_speed_controller_parameters);

struct pidParameters speed_vx_parameters = {
    .kp =  0.1f,
    .ki =  0.05f,
    .kd =  0.0f,
    .min_error_limit = -10.0f,
    .max_error_limit = 10.0f,
    .min_integral = -100.0f,
    .max_integral = 100.0f,
    .min_control_value =  -0.3f,
    .max_control_value = 0.3f,
    .cutoff_frequency = 10.0f
};

PID speed_vx_controller(1/controller_freq, speed_vx_parameters);

struct pidParameters speed_vy_parameters = {
    .kp =  0.1f,
    .ki =  0.05f,
    .kd =  0.0f,
    .min_error_limit = -10.0f,
    .max_error_limit = 10.0f,
    .min_integral = -100.0f,
    .max_integral = 100.0f,
    .min_control_value =  -0.3f,
    .max_control_value = 0.3f,
    .cutoff_frequency = 10.0f
};

PID speed_vy_controller(1/controller_freq, speed_vy_parameters);

struct pidParameters pos_parameters = {
    .kp =  0.6f,
    .ki =  0.2f,
    .kd =  0.0f,
    .min_error_limit = -10.0f,
    .max_error_limit = 10.0f,
    .min_integral = -50.0f,
    .max_integral = 50.0f,
    .min_control_value =  -5.0f,
    .max_control_value = 5.0f,
    .cutoff_frequency = 10.0f
};

PID posY_controller(1/controller_freq, pos_parameters);
PID posX_controller(1/controller_freq, pos_parameters);