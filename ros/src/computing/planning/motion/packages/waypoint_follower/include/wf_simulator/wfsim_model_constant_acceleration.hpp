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
#include "wf_simulator/wfsim_model_interface.hpp"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

class WFSimModelConstantAccelTwist : public WFSimModelInterface
{
public:
    WFSimModelConstantAccelTwist(double vel_lim, double angvel_lim, double vel_rate, double angvel_rate);
    ~WFSimModelConstantAccelTwist() = default;

private:
    enum IDX
    {
        X = 0,
        Y,
        YAW,
        VX,
        WZ,
    };
    enum IDX_U
    {
        VX_DES = 0,
        WZ_DES,
    };

    double vel_lim_;
    double angvel_lim_;
    double vel_rate_;
    double angvel_rate_;

    double getX() override;
    double getY() override;
    double getYaw() override;
    double getVx() override;
    double getWz() override;
    double getSteer() override;
    Eigen::VectorXd calcModel(const Eigen::VectorXd &state, Eigen::VectorXd &input) override;
};
