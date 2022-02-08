#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <array>
#include <xmlrpcpp/XmlRpcValue.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "listener");
    ros::NodeHandle ls;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate r(10);
    geometry_msgs::TransformStamped transformStamped;
    float x;
    float y;
    bool cap;
    XmlRpc::XmlRpcValue marker_pos;
    ls.getParam("/marker_locations", marker_pos);
    int fid_id;
    bool broadcast_f;
    while(ros::ok()){
        try{
            ls.getParam("/broadcast_frame", broadcast_f);
            if(broadcast_f){
                transformStamped = tfBuffer.lookupTransform("map", "marker_frame", ros::Time(0));
                x = transformStamped.transform.translation.x;
                y = transformStamped.transform.translation.y;
                ls.getParam("/marker_id", fid_id);
                marker_pos[fid_id][0] = x;
                marker_pos[fid_id][1] = y; 
                ls.setParam("/marker_locations", marker_pos);
                ls.getParam("/capture", cap);
                
            
                if(x!=0 && y!=0 && cap){
                    ls.setParam("/marker_found", 1);
                    ls.setParam("/broadcast_frame", false);
                } 
            }
            
        }
        catch(tf2::TransformException& ex){
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        r.sleep();
    }
}