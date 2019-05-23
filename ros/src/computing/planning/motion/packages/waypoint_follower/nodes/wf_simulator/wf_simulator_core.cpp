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

#include "wf_simulator_core.hpp"

#define DEBUG_INFO(...) { ROS_INFO(__VA_ARGS__); } 

WFSimulator::WFSimulator() : nh_(""), pnh_("~"), is_initialized_(false)
{
    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("sim_pose", 1);
    pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("sim_velocity", 1);
    pub_vehicle_status_ = nh_.advertise<autoware_msgs::VehicleStatus>("sim_vehicle_status", 1);
    sub_vehicle_cmd_ = nh_.subscribe("vehicle_cmd", 1, &WFSimulator::callbackVehicleCmd, this);
    sub_waypoints_ = nh_.subscribe("base_waypoints", 1, &WFSimulator::callbackWaypoints, this);
    sub_closest_waypoint_ = nh_.subscribe("closest_waypoint", 1, &WFSimulator::callbackClosestWaypoint, this);
    timer_simulation_ = nh_.createTimer(ros::Duration(0.01), &WFSimulator::timerCallbackSimulation, this);
    timer_tf_ = nh_.createTimer(ros::Duration(0.1), &WFSimulator::timerCallbackPublishTF, this);

    pnh_.param("lidar_height", lidar_height_, double(1.0));
    pnh_.param("steer_lim", steer_lim_, double(3.14 / 3.0));
    pnh_.param("vel_lim", vel_lim_, double(10.0));
    pnh_.param("accel_rate", accel_rate_, double(1.0));
    pnh_.param("steer_vel", steer_vel_, double(0.3));
    nh_.param("vehicle_info/wheel_base", wheelbase_, double(2.7));

    pnh_.param("simulation_frame_id", simulation_frame_id_, std::string("base_link"));
    pnh_.param("map_frame_id", map_frame_id_, std::string("map"));
    pnh_.param("lidar_frame_id", lidar_frame_id_, std::string("lidar"));

    std::string vehicle_model_type_str;
    pnh_.param("vehicle_model_type", vehicle_model_type_str, std::string("STEERING"));
    if (vehicle_model_type_str == "ANGVEL")
    {
        vehicle_model_type_ = VehicleModelType::ANGVEL;
    }
    else if (vehicle_model_type_str == "STEERING")
    {
        vehicle_model_type_ = VehicleModelType::STEERING;
    }
    else
    {
        ROS_ERROR("Invalid vehicle_model_type. Initialization failed.");
    }

    std::string initialize_source;
    pnh_.param("initialize_source", initialize_source, std::string("Origin"));
    ROS_INFO_STREAM("initialize_source : " << initialize_source);
    if (initialize_source == "Rviz")
    {
        sub_initialpose_ = nh_.subscribe("initialpose", 1, &WFSimulator::callbackInitialPoseWithCov, this);
    }
    else if (initialize_source == "lidar_localizer")
    {
        sub_initialpose_ = nh_.subscribe("ndt_pose", 1, &WFSimulator::callbackInitialPoseStamped, this);
    }
    else if (initialize_source == "GNSS")
    {
        sub_initialpose_ = nh_.subscribe("gnss_pose", 1, &WFSimulator::callbackInitialPoseStamped, this);
    }
    else if (initialize_source == "Origin")
    {
        geometry_msgs::Pose p;
        p.orientation.w = 1.0; // yaw = 0
        geometry_msgs::Twist t;
        setInitialState(p, t); // initialize with 0 for all variables
    }
    else
    {
        ROS_WARN("initialize_source is undesired, setting error!!");
    }


    vehicle_model_.setSteerLim(steer_lim_);
    vehicle_model_.setVelLim(vel_lim_);
    vehicle_model_.setAccelRate(accel_rate_);
    vehicle_model_.setSteerVel(steer_vel_);
    vehicle_model_.setWheelbase(wheelbase_);

    prev_update_time_ = ros::Time::now();
    is_first_simulation_ = true;
    current_pose_.orientation.w = 1.0;
}

void WFSimulator::timerCallbackSimulation(const ros::TimerEvent &e)
{
    if (!is_initialized_)
        return;

    if (is_first_simulation_)
    {
        prev_update_time_ = ros::Time::now();
        is_first_simulation_ = false;
    }

    const double dt = (ros::Time::now() - prev_update_time_).toSec();
    prev_update_time_ = ros::Time::now();
    vehicle_model_.updateEuler(dt);

    current_pose_.position.x = vehicle_model_.getX();
    current_pose_.position.y = vehicle_model_.getY();
    current_pose_.position.z = 0.0;
    current_pose_.orientation = getQuaternionFromYaw(vehicle_model_.getYaw());
    if (current_waypoints_ptr_ && current_closest_waypoint_ptr_)
    {
        current_pose_.position.z = current_waypoints_ptr_->waypoints[current_closest_waypoint_ptr_->data].pose.pose.position.z;
    }

    current_twist_.linear.x = vehicle_model_.getVx();
    if (vehicle_model_type_ == VehicleModelType::ANGVEL)
    {
        current_twist_.angular.z = vehicle_model_.getWz();
    }
    else if (vehicle_model_type_ == VehicleModelType::STEERING)
    {
        const double steer = vehicle_model_.getSteer();
        current_twist_.angular.z = current_twist_.linear.x * std::tan(steer) / wheelbase_;
        autoware_msgs::VehicleStatus vs;
        vs.header.stamp = ros::Time::now();
        vs.header.frame_id = "base_link";
        vs.speed = current_twist_.linear.x;
        vs.angle = steer;
        pub_vehicle_status_.publish(vs);
    }

    publishPoseTwist(current_pose_, current_twist_);
    
}

void WFSimulator::timerCallbackPublishTF(const ros::TimerEvent &e)
{
    publishTF(current_pose_, current_twist_);
}

void WFSimulator::callbackVehicleCmd(const autoware_msgs::VehicleCmdConstPtr &msg)
{
    current_vehicle_cmd_ptr_ = std::make_shared<autoware_msgs::VehicleCmd>(*msg);
    if (vehicle_model_type_ == VehicleModelType::ANGVEL)
    {
        Eigen::VectorXd input(2);
        input << msg->twist_cmd.twist.linear.x, msg->twist_cmd.twist.angular.z;
        vehicle_model_.setInput(input);
    }
    else if (vehicle_model_type_ == VehicleModelType::STEERING)
    {
        Eigen::VectorXd input(2);
        input << msg->ctrl_cmd.linear_velocity, msg->ctrl_cmd.steering_angle;
        vehicle_model_.setInput(input);
    }
}

void WFSimulator::callbackWaypoints(const autoware_msgs::LaneConstPtr &msg)
{
    current_waypoints_ptr_ = std::make_shared<autoware_msgs::Lane>(*msg);
}

void WFSimulator::callbackClosestWaypoint(const std_msgs::Int32ConstPtr &msg)
{
    current_closest_waypoint_ptr_ = std::make_shared<std_msgs::Int32>(*msg);
}

void WFSimulator::callbackInitialPoseWithCov(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    geometry_msgs::Twist initial_twist; // initialized with zero for all components
    setInitialStateWithPoseTransform(*msg, initial_twist);
}

void WFSimulator::callbackInitialPoseStamped(const geometry_msgs::PoseStampedConstPtr &msg)
{
    geometry_msgs::Twist initial_twist; // initialized with zero for all components
    setInitialStateWithPoseTransform(*msg, initial_twist);
}

void WFSimulator::setInitialStateWithPoseTransform(const geometry_msgs::PoseStamped &pose_stamped, const geometry_msgs::Twist &twist)
{
    tf::StampedTransform transform;
    getTransformFromTF(map_frame_id_, pose_stamped.header.frame_id, transform);
    geometry_msgs::Pose pose;
    pose.position.x = pose_stamped.pose.position.x + transform.getOrigin().x();
    pose.position.y = pose_stamped.pose.position.y + transform.getOrigin().y();
    pose.position.z = pose_stamped.pose.position.z + transform.getOrigin().z();
    pose.orientation = pose_stamped.pose.orientation;
    setInitialState(pose, twist);
}

void WFSimulator::setInitialStateWithPoseTransform(const geometry_msgs::PoseWithCovarianceStamped &pose, const geometry_msgs::Twist &twist)
{
    geometry_msgs::PoseStamped ps;
    ps.header = pose.header;
    ps.pose = pose.pose.pose;
    setInitialStateWithPoseTransform(ps, twist);
}

void WFSimulator::setInitialState(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist)
{
    const double x = pose.position.x;
    const double y = pose.position.y;
    const double yaw = tf2::getYaw(pose.orientation);
    const double vx = twist.linear.x;
    const double wz = twist.angular.z;
    const double steer = 0.0;

    if (vehicle_model_type_ == VehicleModelType::ANGVEL)
    {
        Eigen::VectorXd state(5);
        state << x, y, yaw, vx, wz;
        vehicle_model_.setState(state);
    }
    else if (vehicle_model_type_ == VehicleModelType::STEERING)
    {
        Eigen::VectorXd state(5);
        state << x, y, yaw, vx, steer;
        std::cout << "initial set" << state << std::endl;
        vehicle_model_.setState(state);
    }
    else
    {
        ROS_WARN("undesired vehicle model type! Initialization failed.");
        return;
    }

    is_initialized_ = true;
}

void WFSimulator::getTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
    while (1)
    {
        try
        {
            tf_listener_.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
            break;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}

void WFSimulator::publishPoseTwist(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist)
{
    ros::Time current_time = ros::Time::now();

    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = map_frame_id_;
    ps.header.stamp = current_time;
    ps.pose = pose;
    pub_pose_.publish(ps);

    geometry_msgs::TwistStamped ts;
    ts.header.frame_id = "base_link";
    ts.header.stamp = current_time;
    ts.twist = twist;
    pub_twist_.publish(ts);
}

void WFSimulator::publishTF(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &twist)
{
    ros::Time current_time = ros::Time::now();

    // send odom transform
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = map_frame_id_;
    odom_trans.child_frame_id = simulation_frame_id_;
    odom_trans.transform.translation.x = pose.position.x;
    odom_trans.transform.translation.y = pose.position.y;
    odom_trans.transform.translation.z = pose.position.z;
    odom_trans.transform.rotation = pose.orientation;
    tf_broadcaster_.sendTransform(odom_trans);

    // send lidar transform
    geometry_msgs::TransformStamped lidar_trans;
    lidar_trans.header.stamp = odom_trans.header.stamp;
    lidar_trans.header.frame_id = simulation_frame_id_;
    lidar_trans.child_frame_id = lidar_frame_id_;
    lidar_trans.transform.translation.z += lidar_height_;
    lidar_trans.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(lidar_trans);
}

geometry_msgs::Quaternion WFSimulator::getQuaternionFromYaw(const double &_yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, _yaw);
    
    return tf2::toMsg(q);
}