
/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "wf_simulator/wfsim_model_time_delay_steer.hpp"

WFSimModelTimeDelaySteer::WFSimModelTimeDelaySteer(double vel_lim, double steer_lim, double wheelbase, double dt, double vel_delay,
                                                   double vel_time_constant, double steer_delay, double steer_time_constant)
    : WFSimModelInterface(5 /* dim x */, 2 /* dim u */), vel_lim_(vel_lim), steer_lim_(steer_lim), wheelbase_(wheelbase), vel_delay_(vel_delay),
      vel_time_constant_(vel_time_constant), steer_delay_(steer_delay), steer_time_constant_(steer_time_constant)
{
    initializeInputQueue(dt);
};

double WFSimModelTimeDelaySteer::getX() { return state_(IDX::X); };
double WFSimModelTimeDelaySteer::getY() { return state_(IDX::Y); };
double WFSimModelTimeDelaySteer::getYaw() { return state_(IDX::YAW); };
double WFSimModelTimeDelaySteer::getVx() { return state_(IDX::VX); };
double WFSimModelTimeDelaySteer::getWz() { return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_; };
double WFSimModelTimeDelaySteer::getSteer() { return state_(IDX::STEER); };

void WFSimModelTimeDelaySteer::initializeInputQueue(const double &dt)
{
    size_t vel_input_queue_size = static_cast<size_t>(round(vel_delay_ / dt));
    for (size_t i = 0; i < vel_input_queue_size; i++)
    {
        vel_input_queue_.push_back(0.0);
    }
    size_t steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
    for (size_t i = 0; i < steer_input_queue_size; i++)
    {
        steer_input_queue_.push_back(0.0);
    }
}

Eigen::VectorXd WFSimModelTimeDelaySteer::calcModel(const Eigen::VectorXd &state, Eigen::VectorXd &input)
{
    const double delay_input_vel = vel_input_queue_.front();
    vel_input_queue_.pop_front();
    vel_input_queue_.push_back(input_(IDX_U::VX_DES));
    const double delay_input_steer = steer_input_queue_.front();
    steer_input_queue_.pop_front();
    steer_input_queue_.push_back(input_(IDX_U::STEER_DES));

    const double vel = state(IDX::VX);
    const double yaw = state(IDX::YAW);
    const double steer = state(IDX::STEER);
    const double delay_vx_des = std::max(std::min(delay_input_vel, vel_lim_), -vel_lim_);
    const double delay_steer_des = std::max(std::min(delay_input_steer, steer_lim_), -steer_lim_);
    double vx_rate = 0.0;
    double steer_rate = 0.0;
    vx_rate = -(vel - delay_vx_des) / vel_time_constant_;
    steer_rate = -(steer - delay_steer_des) / steer_time_constant_;

    Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
    d_state(IDX::X) = vel * cos(yaw);
    d_state(IDX::Y) = vel * sin(yaw);
    d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
    d_state(IDX::VX) = vx_rate;
    d_state(IDX::STEER) = steer_rate;

    return d_state;
};