/**
 * @file Broadcast.h
 * @author Rutvik Kevadiya (rutvik08@umd.edu) Param Dave(pdave1@umd.edu) Darshan Jain(d.jain12@umd.edu)
 * @brief 
 * @version 1
 * @date 2021-12-14
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef BROADCAST_H
#define BROADCAST_H

#include <ros/ros.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <tf2_ros/transform_broadcaster.h>
/**
 * @brief Class for broadcaster
 * 
 */
class Broadcast{
    public:
    /**
     * @brief Construct a new Broadcast object
     * 
     * @param nodehandle 
     */
    Broadcast(ros::NodeHandle *nodehandle);
    /**
     * @brief Callback function which broadcasts marker_frame when an aruco marker is detected.
     * 
     * @param msg 
     */
    void broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg);
    /**
     * @brief Set a value for the parameter /marker_id on the parameter server.
     * 
     */
    void set_marker_id();
    /**
     * @brief Return the value of the private variable d.
     * 
     * @return double 
     */
    double get_d();
    /**
     * @brief Return the value of the parameter /broadcast_frame from the parameter server.
     * 
     * @return true 
     * @return false 
     */
    bool start_broadcast();

    private:
    ros::NodeHandle BR;
    ros::Subscriber fiducial_sub;
    double d;
    int fid_id;
};
#endif