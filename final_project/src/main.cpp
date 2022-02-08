#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <array>
#include "../include/Robot.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
  bool explorer_goal_sent = false;
  bool follower_goal_sent = false;

  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  Robot explorer(&nh);

  std::array<XmlRpc::XmlRpcValue,4> target;
  nh.getParam("/aruco_lookup_locations/target_1", target[0]);
  nh.getParam("/aruco_lookup_locations/target_2", target[1]);
  nh.getParam("/aruco_lookup_locations/target_3", target[2]);
  nh.getParam("/aruco_lookup_locations/target_4", target[3]);

  MoveBaseClient explorer_client("/explorer/move_base", true);
  MoveBaseClient follower_client("/follower/move_base", true);

  while (!explorer_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for explorer");
  }

  while (!follower_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up for follower");
  }

  move_base_msgs::MoveBaseGoal explorer_goal;
  move_base_msgs::MoveBaseGoal follower_goal;

  for(int i=0;i<4;i++){
    explorer_goal.target_pose.header.frame_id = "map";
    explorer_goal.target_pose.header.stamp = ros::Time::now();
    explorer_goal.target_pose.pose.position.x = target[i][0];//
    explorer_goal.target_pose.pose.position.y = target[i][1];//
    explorer_goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    explorer_client.sendGoal(explorer_goal);
    explorer_client.waitForResult();
    if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Hooray, robot reached goal");
    }
    ros::Rate r(10);
    explorer.start_broadcasting(true);
    while (ros::ok())
    {
      explorer.rotate();
      ros::spinOnce();
      if(explorer.marker_found() == 1){
        explorer.stop();
        break;
      }
      r.sleep();
    }
    explorer.set_marker_found(0);
  }
  
  explorer_goal.target_pose.header.frame_id = "map";
  explorer_goal.target_pose.header.stamp = ros::Time::now();
  explorer_goal.target_pose.pose.position.x = -4;//
  explorer_goal.target_pose.pose.position.y = 2.5;//
  explorer_goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending goal");
  explorer_client.sendGoal(explorer_goal);
  explorer_client.waitForResult();
  if (explorer_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, robot reached goal");
  }
  
  XmlRpc::XmlRpcValue marker_pos;
  nh.getParam("/marker_locations", marker_pos);

  for(int j=0;j<4;j++){
    
    follower_goal.target_pose.header.frame_id = "map";
    follower_goal.target_pose.header.stamp = ros::Time::now();
    follower_goal.target_pose.pose.position.x = marker_pos[j][0];//
    follower_goal.target_pose.pose.position.y = marker_pos[j][1];//
    follower_goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    follower_client.sendGoal(follower_goal);
    follower_client.waitForResult();
    if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, robot reached goal");
    }
  }

  follower_goal.target_pose.header.frame_id = "map";
  follower_goal.target_pose.header.stamp = ros::Time::now();
  follower_goal.target_pose.pose.position.x = -4;//
  follower_goal.target_pose.pose.position.y = 3.5;//
  follower_goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  follower_client.sendGoal(follower_goal);
  follower_client.waitForResult();
  if (follower_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, robot reached goal");
  }

  // explorer_client.waitForResult();

  

  
  


}