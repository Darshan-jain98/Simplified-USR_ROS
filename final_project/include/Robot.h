/**
 * @file Robot.h
 * @author Rutvik Kevadiya (rutvik08@umd.edu) Param Dave(pdave1@umd.edu) Darshan Jain(d.jain12@umd.edu)
 * @brief 
 * @version 1
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef ROBOT_H
#define ROBOT_H

#include <geometry_msgs/Twist.h>   //for geometry_msgs::Twist
#include <ros/ros.h>
#include <string>

/**
 * @brief A class for the robot which can be used to perform simple operations such as rotate, stop, etc.
 * 
 */
class Robot
{
    public:
    /**
     * @brief Construct a new Robot object
     * 
     * @param nodehandle 
     */
    Robot(ros::NodeHandle *nodehandle);
    /**
     * @brief Publish velocity to the controller to rotate the explorer.
     * 
     */
    void rotate();
    /**
     * @brief Publish velocity to the controller to stop the explorer.
     * 
     */
    void stop();
    /**
     * @brief Set the parameter /broadcast_frame a boolean value on the parameter server.
     * 
     * @param b 
     */
    void start_broadcasting(bool b);
    /**
     * @brief Return the value of the parameter /marker_found from the parameter server. 
     * 
     * @return true 
     * @return false 
     */
    bool marker_found();
    /**
     * @brief Set the parameter /marker_found a boolean value on the parameter server.
     * 
     * @param b 
     */
    void set_marker_found(int b);

    private:
    ros::Publisher vel_publisher;
    ros::NodeHandle nh;
};
#endif
