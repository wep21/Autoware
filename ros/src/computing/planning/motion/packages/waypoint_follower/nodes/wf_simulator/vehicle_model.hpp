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

#pragma once
#include <ros/ros.h>
#include <iostream>
#include <queue>
#include <deque>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

class VehicleModel
{
public:
    VehicleModel();
    ~VehicleModel();
    void getState(Eigen::VectorXd &state) { state = state_; };
    double getX() { return state_(IDX::X); };
    double getY() { return state_(IDX::Y); };
    double getYaw() { return state_(IDX::YAW); };
    double getVx() { return state_(IDX::VX); };
    double getWz() { return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_; };
    double getSteer() { return state_(IDX::STEER); };
    void setDeltaT(const double &dt) { deltaT_ = dt; } ;
    void initialInputQueue() {
        vel_delay_ = static_cast<size_t>(round(velL_/deltaT_));
        for (size_t i = 0; i < vel_delay_; i++) {
            vel_input_queue_.push_back(0.0);
        }
        steer_delay_ = static_cast<size_t>(round(steerL_/deltaT_));
        for (size_t i = 0; i < steer_delay_; i++) {
            steer_input_queue_.push_back(0.0);
        }
    }
    void setState(const Eigen::VectorXd &state) { state_ = state; std::cout << "state : " << state << "state_:" << state_ << std::endl;};
    void setInput(const Eigen::VectorXd &input) { input_ = input; };
    void setSteerLim(const double &steer_lim) { steer_lim_ = steer_lim; };
    void setVelLim(const double &vel_lim) { vel_lim_ = vel_lim; };
    void setAccelRate(const double &accel_rate) { accel_rate_ = accel_rate; };
    void setSteerVel(const double &steer_vel) { steer_vel_ = steer_vel; };
    void setWheelbase(const double &wheelbase) { wheelbase_ = wheelbase; };
    void setVelTimeDelay(const double &velL) { velL_ = velL; };
    void setVelTimeConstant(const double &velT) { velT_ = velT; };
    void setSteerTimeDelay(const double &steerL) { steerL_ = steerL; };
    void setSteerTimeConstant(const double &steerT) { steerT_ = steerT; };
    void updateEuler(const double &dt);
    void updateRungeKutta(const double &dt);

private:
    enum IDX
    {
        X = 0,
        Y,
        YAW,
        VX,
        STEER,
    };
    enum IDX_U
    {
        VX_DES = 0,
        STEER_DES,
    };

    double deltaT_;
    double dim_x_;
    double dim_u_;
    Eigen::VectorXd state_;
    Eigen::VectorXd input_;
    double delay_input_vel, delay_input_steer;
    std::deque<double> vel_input_queue_, steer_input_queue_;
    double velT_, velL_, steerT_, steerL_;
    /*
      T_: First Order System Time constant
      L_: Dead time Constant
      delay_ = static_cast<size_t>(round(L_/deltaT_));
    */
    size_t vel_delay_, steer_delay_;

    double steer_lim_;
    double vel_lim_;
    double accel_rate_;
    double steer_vel_;
    double wheelbase_;

    Eigen::VectorXd calcConstantAccelModel(const Eigen::VectorXd &state, Eigen::VectorXd &input);
    Eigen::VectorXd calcInputDelayModel(const Eigen::VectorXd &state, const double &vel_input, const double &steer_input);
};
