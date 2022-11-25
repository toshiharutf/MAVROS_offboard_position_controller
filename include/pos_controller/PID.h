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

static constexpr float CONSTANT_PI{3.141592F};

struct pidParameters{
  float kp, ki, kd{0.0f};
  float min_error_limit{-99.9f};
  float max_error_limit{99.99f};
  float min_integral{-99.9f};
  float max_integral{99.99f};
  float min_control_value{-99.9f};
  float max_control_value{99.99f};
  float cutoff_frequency{10.0f};
};

class PID {

 private:
  const float dt;
  pidParameters pid_parameters;
  float past_error{0.0f};
  float past_derivative_error{0.0f};

  float constrain(float value, const float minVal, const float maxVal);
  const float tau{1.0f/ (2*CONSTANT_PI*pid_parameters.cutoff_frequency)};

 public:
  PID(const float &dt, const pidParameters &pid_parameters_);
  float compute_control_value(float target_state, float current_state);
  void updateParameters(const pidParameters &pid_parameters);


};
