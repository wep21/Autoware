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

#pragma once
#include <eigen3/Eigen/Core>

class WFSimModelInterface
{
protected:
  const int dim_x_; //!< @brief dimension of state x
  const int dim_u_; //!< @brief dimension of input u
  Eigen::VectorXd state_;
  Eigen::VectorXd input_;

public:
  WFSimModelInterface(int dim_x, int dim_u);
  ~WFSimModelInterface() = default;

  void getState(Eigen::VectorXd &state);
  void getInput(Eigen::VectorXd &input);
  void setState(const Eigen::VectorXd &state);
  void setInput(const Eigen::VectorXd &input);

  void updateRungeKutta(const double &dt);
  void updateEuler(const double &dt);

  virtual double getX() = 0;
  virtual double getY() = 0;
  virtual double getYaw() = 0;
  virtual double getVx() = 0;
  virtual double getWz() = 0;
  virtual double getSteer() = 0;
  virtual Eigen::VectorXd calcModel(const Eigen::VectorXd &state, Eigen::VectorXd &input) = 0;
};
