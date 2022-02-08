#include "../include/Robot.h"

Robot::Robot(ros::NodeHandle *nodehandle): nh{*nodehandle}{
    vel_publisher = nh.advertise<geometry_msgs::Twist>("/explorer/cmd_vel", 10);
}

void Robot::rotate(){
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.5;
    vel_publisher.publish(cmd);
}

void Robot::stop(){
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    vel_publisher.publish(cmd);
}

void Robot::start_broadcasting(bool b){
    nh.setParam("/broadcast_frame", b);
}

bool Robot::marker_found(){
    int marker_f;
    nh.getParam("/marker_found", marker_f);
    return marker_f;
}

void Robot::set_marker_found(int b){
    nh.setParam("/marker_found", b);
}