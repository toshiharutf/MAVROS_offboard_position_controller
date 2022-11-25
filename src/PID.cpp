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

#include "PID.h"
#include <math.h>

PID::PID(const float &dt, const pidParameters &pid_parameters_) : dt(dt), pid_parameters(pid_parameters_){

}

float PID::compute_control_value(float target_state, float current_state) {
  float error, derivative_error, integral_error, control_value;

  error = constrain(target_state - current_state, pid_parameters.min_error_limit, pid_parameters.max_error_limit);

  if(dt > 1.0E-6F){
    derivative_error = (error - past_error)/dt;

    // low pass filter
    derivative_error = past_derivative_error + ((dt/(dt+tau))) * (derivative_error-past_derivative_error);
    past_derivative_error = derivative_error;
  }
  integral_error = constrain((past_error + error/2)*dt, pid_parameters.min_integral, pid_parameters.max_integral);

  control_value = pid_parameters.kp*error + pid_parameters.ki*integral_error + pid_parameters.kd*derivative_error;
  control_value = constrain(control_value, pid_parameters.min_control_value, pid_parameters.max_control_value);

  past_error = error;
  return control_value;
}

float PID::constrain(float value, const float minVal, const float maxVal) {
  return fminf(maxVal, fmaxf(minVal, value));
}

void PID::updateParameters(const pidParameters &pid_parameters_) {
  pid_parameters = pid_parameters_;
}
