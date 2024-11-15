#pragma once

#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <opencv2/opencv.hpp>
#include "robotinfo_msgs/RobotInfo10Fields.h"

#include <string>
#include <chrono>

class RobotGUI {
public:
    // constructor
    RobotGUI();
    
    //GUI window running
    void run();



private:
    // robot info subscriber class members
    ros::Subscriber robotinfo_sub;
    std::string robotinfo_topic;
    robotinfo_msgs::RobotInfo10Fields robotinfo_data;

    void robotinfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr& robotinfo_data);
    std::chrono::steady_clock::time_point last_message_time;
    const std::chrono::seconds MESSAGE_TIMEOUT = std::chrono::seconds(2); // 1 second timeout

    void resetDisplayData();
    // CMD BUTTONS
    ros::Publisher velocity_pub;
    geometry_msgs::Twist velocity_data;
    std::string velocity_topic;
    float linear_velocity_step = 0.3;
    float angular_velocity_step = 0.3;
    
    ros::Timer velocity_timer;
    void publishVelocity(const ros::TimerEvent&);  // timer callback function
    
    // ODOM subscribers
    ros::Subscriber velocities_sub;
    void velocitiesCallback(const geometry_msgs::Twist::ConstPtr& velocities_data);
    float linear_velocity = 0.0;
    float angular_velocity = 0.0;

    ros::Subscriber position_sub;
    void positionCallback(const nav_msgs::Odometry::ConstPtr& position_data);
    std::string position_topic;
    float x_position = 0.0;
    float y_position = 0.0;
    float z_position = 0.0;

    // Tracking
    ros::ServiceClient service_client;
    // create a service request
    std_srvs::Trigger srv_req;
    std::string service_name;
    std::string last_service_call_msg;
    int service_call_counter = 0;
    //----------------------------------

    // general GUI data member
    const std::string WINDOW_NAME = "MIR Panel";

    // methods to make up the GUI application
    void robotInfo(cv::Mat& frame);
    void cmdButtons(cv::Mat& frame);
    void robotVelocities(cv::Mat& frame);
    void robotPosition(cv::Mat& frame);
    void distanceTracker(cv::Mat& frame);
    //------------------------------------------------


};