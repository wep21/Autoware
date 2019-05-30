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

VehicleModel::VehicleModel() : dim_x_(5), dim_u_(2),
                               velT_(1.0), velL_(0.2),
                               steerT_(1.0), steerL_(0.2)
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

VehicleModel::~VehicleModel()
{
    vel_input_queue_.clear();
    steer_input_queue_.clear();
}

void VehicleModel::updateEuler(const double &dt)
{
    delay_input_vel = vel_input_queue_.front();
    vel_input_queue_.pop_front();
    vel_input_queue_.push_back(input_(IDX_U::VX_DES));
    delay_input_steer = steer_input_queue_.front();
    steer_input_queue_.pop_front();
    steer_input_queue_.push_back(input_(IDX_U::STEER_DES));

    state_ += calcInputDelayModel(state_, delay_input_vel, delay_input_steer) * dt;

    // state_ += calcConstantAccelModel(state_, input_) * dt;

    printf("[wf_simulator Euler] state_ = X: %4.4f,  Y:%4.4f,  YAW:%4.4f,  VX: %4.4f,  STEER: %4.4f,  dt = %3.3f\n", state_(0), state_(1), state_(2), state_(3), state_(4), dt);
}

void VehicleModel::updateRungeKutta(const double &dt)
{
    delay_input_vel = vel_input_queue_.front();
    vel_input_queue_.pop_front();
    vel_input_queue_.push_back(input_(IDX_U::VX_DES));
    delay_input_steer = steer_input_queue_.front();
    steer_input_queue_.pop_front();
    steer_input_queue_.push_back(input_(IDX_U::STEER_DES));

    Eigen::VectorXd k1 = calcInputDelayModel(state_, delay_input_vel, delay_input_steer);
    Eigen::VectorXd k2 = calcInputDelayModel(state_ + k1 * 0.5 * dt, delay_input_vel, delay_input_steer);
    Eigen::VectorXd k3 = calcInputDelayModel(state_ + k2 * 0.5 * dt, delay_input_vel, delay_input_steer);
    Eigen::VectorXd k4 = calcInputDelayModel(state_ + k3 * dt, delay_input_vel, delay_input_steer);

    // Eigen::VectorXd k1 = calcConstantAccelModel(state_, input_);
    // Eigen::VectorXd k2 = calcConstantAccelModel(state_ + k1 * 0.5 * dt, input_);
    // Eigen::VectorXd k3 = calcConstantAccelModel(state_ + k2 * 0.5 * dt, input_);
    // Eigen::VectorXd k4 = calcConstantAccelModel(state_ + k3 * dt, input_);
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

Eigen::VectorXd VehicleModel::calcInputDelayModel(const Eigen::VectorXd &state, const double &delay_input_vel, const double &delay_input_steer)
{
    // TODO
    // time delay for input -> Done
    // change velocity & steering dynamics to 1d-model -> Done
    // add dead-zone for vel & steer control
    // add static & dynamics friction for steering
    const double vel = state(IDX::VX);
    const double yaw = state(IDX::YAW);
    const double steer = state(IDX::STEER);
    const double delay_vx_des = std::max(std::min(delay_input_vel, vel_lim_), -vel_lim_);
    const double delay_steer_des = std::max(std::min(delay_input_steer, steer_lim_), -steer_lim_);
    double vx_accel = 0.0;
    double steer_vel = 0.0;
    vx_accel = - 1.0/velT_ * (vel - delay_vx_des);
    steer_vel = - 1.0/steerT_ * (steer - delay_steer_des);

    Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
    d_state(IDX::X) = vel * cos(yaw);
    d_state(IDX::Y) = vel * sin(yaw);
    d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
    d_state(IDX::VX) = vx_accel;
    d_state(IDX::STEER) = steer_vel;

    return d_state;
}
