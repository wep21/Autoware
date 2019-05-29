/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "vehicle_model.hpp"

VehicleModel::VehicleModel() : dim_x_(5), dim_u_(2)
{
    state_ = Eigen::VectorXd::Zero(dim_x_);
    input_ = Eigen::VectorXd::Zero(dim_u_);

    // set as a default value
    steer_lim_ = 0.0;
    vel_lim_ = 0.0;
    accel_rate_ = 0.0;
    steer_vel_ = 0.0;
    wheelbase_ = 1.0;
};

void VehicleModel::updateEuler(const double &dt)
{
    state_ += calcConstantAccelModel(state_, input_) * dt;

    printf("[wf_simulator Euler] state_ = X: %4.4f,  Y:%4.4f,  YAW:%4.4f,  VX: %4.4f,  STEER: %4.4f,  dt = %3.3f\n", state_(0), state_(1), state_(2), state_(3), state_(4), dt);
}

void VehicleModel::updateRungeKutta(const double &dt)
{
    Eigen::VectorXd k1 = calcConstantAccelModel(state_, input_);
    Eigen::VectorXd k2 = calcConstantAccelModel(state_ + k1 * 0.5 * dt, input_);
    Eigen::VectorXd k3 = calcConstantAccelModel(state_ + k2 * 0.5 * dt, input_);
    Eigen::VectorXd k4 = calcConstantAccelModel(state_ + k3 * dt, input_);
    state_ += 1.0/6 * (k1 + 2*k2 + 2*k3 + k4) * dt;

    printf("[wf_simulator RungeKutta] state_ = X: %4.4f,  Y:%4.4f,  YAW:%4.4f,  VX: %4.4f,  STEER: %4.4f,  dt = %3.3f\n", state_(0), state_(1), state_(2), state_(3), state_(4), dt);
}


Eigen::VectorXd VehicleModel::calcConstantAccelModel(const Eigen::VectorXd &state, Eigen::VectorXd &input)
{
    const double vel = state(IDX::VX);
    const double yaw = state(IDX::YAW);
    const double steer = state(IDX::STEER);
    const double vx_des = std::max(std::min(input(IDX_U::VX_DES), vel_lim_), -vel_lim_);
    const double steer_des = std::max(std::min(input(IDX_U::STEER_DES), steer_lim_), -steer_lim_);
    double vx_accel = 0.0;
    double steer_vel = 0.0;
    if (vx_des > vel)
    {
        vx_accel = accel_rate_;
    }
    else if (vx_des < vel)
    {
        vx_accel = -accel_rate_;
    }

    if (steer_des > steer)
    {
        steer_vel = steer_vel_;
    }
    else if (steer_des < steer)
    {
        steer_vel = -steer_vel_;
    }

    Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
    d_state(IDX::X) = vel * cos(yaw);
    d_state(IDX::Y) = vel * sin(yaw);
    d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
    d_state(IDX::VX) = vx_accel;
    d_state(IDX::STEER) = steer_vel;

    return d_state;
};

Eigen::VectorXd calcInputDelayModel(const Eigen::VectorXd &state, Eigen::VectorXd &input)
{
    /* write me */

    // TODO
    // time delay for input
    // change velocity & steering dynamics to 1d-model
    // add dead-zone for vel & steer control
    // add static & dynamics friction for steering
}
