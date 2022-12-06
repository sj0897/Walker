// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file walker.cpp
 * @author Sparsh <sparshjaiswal97@gmail.com>
 * @brief File to implement code for walker behavior
 * @version 0.1
 * @date 2022-12-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

/**
 * @brief Class to implement code for walker behavior
 * 
 */
class Walker : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Walker object
     * 
     */
    Walker(): Node("Walker"), collision_distance_(0.5) {
        RCLCPP_INFO(this->get_logger(), "Setting up publisher and subcriber");
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                                            "cmd_vel", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Walker::scan_callback, this, _1));
        RCLCPP_INFO(this->get_logger(), "Walker starts");
    }

 private:
    /**
     * @brief Callback function to detect obstacle and move the robot
     * 
     * @param scan_msg 
     */
    void scan_callback(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) const {
        int start_idx = 45;
        int end_idx = 315;
        geometry_msgs::msg::Twist cmd_vel_msg;
        double scan_max = scan_msg->range_max;
        double min_dist_to_obstacle = scan_max;

        for ( int i = 0 ; i < scan_msg->ranges.size() ; i++ ) {
            if (i <= start_idx || i >= end_idx) {
                if (!std::isnan(scan_msg->ranges[i])) {
                    double scan_dist = scan_msg->ranges[i];
                    if (scan_dist < min_dist_to_obstacle) {
                        min_dist_to_obstacle = scan_dist;
                    }
                }
            }
        }
        if (min_dist_to_obstacle <= collision_distance_) {
            RCLCPP_WARN(this->get_logger(), "Obstacle found");
            RCLCPP_INFO(this->get_logger(), "Rotating");
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.5;
        } else {
            RCLCPP_INFO(this->get_logger(), "Path clear");
            cmd_vel_msg.linear.x = 0.3;
            cmd_vel_msg.angular.z = 0.0;
        }
        vel_pub_->publish(cmd_vel_msg);
    }
    /**
     * @brief Publisher to drive the robot
     * 
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    /**
     * @brief Threshold collision distance
     * 
     */
    double collision_distance_;
    /**
     * @brief Subscriber to laser topic
     * 
     */
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    /**
     * @brief Topic to publish velocity commands to
     * 
     */
    std::string cmd_vel_topic = "/cmd_vel";
    /**
     * @brief Topic to subscribe for laser scans
     * 
     */
    std::string laser_topic = "/scan";
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Walker>());
    rclcpp::shutdown();
    return 0;
}