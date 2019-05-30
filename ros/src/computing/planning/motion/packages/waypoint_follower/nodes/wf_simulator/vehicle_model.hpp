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
        delay_ = static_cast<size_t>(round(L_/deltaT_));
        for (size_t i = 0; i < delay_; i++) {
            input_queue_.push_back(Eigen::VectorXd::Zero(dim_u_));
        }
    }
    void setState(const Eigen::VectorXd &state) { state_ = state; std::cout << "state : " << state << "state_:" << state_ << std::endl;};
    void setInput(const Eigen::VectorXd &input) { input_ = input; };
    void setSteerLim(const double &steer_lim) { steer_lim_ = steer_lim; };
    void setVelLim(const double &vel_lim) { vel_lim_ = vel_lim; };
    void setAccelRate(const double &accel_rate) { accel_rate_ = accel_rate; };
    void setSteerVel(const double &steer_vel) { steer_vel_ = steer_vel; };
    void setWheelbase(const double &wheelbase) { wheelbase_ = wheelbase; };
    // void setVelTimeDelay(){};
    // void setVelTimeConstant(){};
    // void setSteerTimeDelay(){};
    // void setSteerTimeConstant(){};
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
    Eigen::VectorXd delay_input_;
    std::deque<Eigen::VectorXd> input_queue_;
    double T_, L_;
    /*
      T_: First Order Lag Time constant
      L_: Dead time Constant
      delay_ = static_cast<size_t>(round(L_/deltaT_));
    */
    size_t delay_;

    double steer_lim_;
    double vel_lim_;
    double accel_rate_;
    double steer_vel_;
    double wheelbase_;

    Eigen::VectorXd calcConstantAccelModel(const Eigen::VectorXd &state, Eigen::VectorXd &input);
    Eigen::VectorXd calcInputDelayModel(const Eigen::VectorXd &state, Eigen::VectorXd &input);
};
