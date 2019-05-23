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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include "autoware_msgs/VehicleCmd.h"
#include "autoware_msgs/VehicleStatus.h"
#include "autoware_msgs/Lane.h"

#include "vehicle_model.hpp"

class WFSimulator
{
public:
    WFSimulator();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_vehicle_status_;
    ros::Subscriber sub_vehicle_cmd_;
    ros::Subscriber sub_waypoints_;
    ros::Subscriber sub_initialpose_;
    ros::Subscriber sub_closest_waypoint_;
    ros::Timer timer_simulation_;
    ros::Timer timer_tf_;

    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_twist_;
    std::shared_ptr<autoware_msgs::VehicleCmd> current_vehicle_cmd_ptr_;
    std::shared_ptr<autoware_msgs::Lane> current_waypoints_ptr_;
    std::shared_ptr<std_msgs::Int32> current_closest_waypoint_ptr_;
    bool is_initialized_;
    bool is_first_simulation_;

    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    std::string simulation_frame_id_;
    std::string map_frame_id_;
    std::string lidar_frame_id_;
    ros::Time prev_update_time_;

    double lidar_height_;
    double steer_lim_;
    double vel_lim_;
    double accel_rate_;
    double steer_vel_;
    double wheelbase_;

    enum class VehicleModelType
    {
        ANGVEL = 0,
        STEERING = 1,
    } vehicle_model_type_;

    VehicleModel vehicle_model_;

    void callbackVehicleCmd(const autoware_msgs::VehicleCmdConstPtr &msg);
    void callbackWaypoints(const autoware_msgs::LaneConstPtr &msg);
    void callbackClosestWaypoint(const std_msgs::Int32ConstPtr &msg);
    void callbackInitialPoseWithCov(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
    void callbackInitialPoseStamped(const geometry_msgs::PoseStampedConstPtr &msg);
    void getTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
    void timerCallbackSimulation(const ros::TimerEvent &e);
    void timerCallbackPublishTF(const ros::TimerEvent &e);

    void setInitialState(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist);
    void setInitialStateWithPoseTransform(const geometry_msgs::PoseStamped &pose, const geometry_msgs::Twist &twist);
    void setInitialStateWithPoseTransform(const geometry_msgs::PoseWithCovarianceStamped &pose, const geometry_msgs::Twist &twist);
    void publishPoseTwist(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist);
    void publishTF(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist);
    geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw);
};
