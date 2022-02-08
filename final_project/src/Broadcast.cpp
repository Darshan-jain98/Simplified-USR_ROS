#include "../include/Broadcast.h"
#include <math.h>

Broadcast::Broadcast(ros::NodeHandle *nodehandle): BR{*nodehandle}{
    fiducial_sub = BR.subscribe("/fiducial_transforms", 1, &Broadcast::broadcast, this);
}

void Broadcast::broadcast(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg){
    if(!msg->transforms.empty() && this->start_broadcast()){
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "explorer_tf/camera_rgb_optical_frame";
        transformStamped.child_frame_id = "marker_frame";
        double x_marker = msg->transforms[0].transform.translation.x;
        double y_marker = msg->transforms[0].transform.translation.y;
        double z_marker = msg->transforms[0].transform.translation.z;
        d = sqrt(x_marker*x_marker + y_marker*y_marker + z_marker*z_marker);
        transformStamped.transform.translation.x = x_marker*(d - 0.4)/d;
        transformStamped.transform.translation.y = y_marker*(d - 0.4)/d;
        transformStamped.transform.translation.z = z_marker*(d - 0.4)/d;
        transformStamped.transform.rotation.x = msg->transforms[0].transform.rotation.x;
        transformStamped.transform.rotation.y = msg->transforms[0].transform.rotation.y;
        transformStamped.transform.rotation.z = msg->transforms[0].transform.rotation.z;
        transformStamped.transform.rotation.w = msg->transforms[0].transform.rotation.w;
        fid_id = msg->transforms[0].fiducial_id;
        br.sendTransform(transformStamped);
    }
}

void Broadcast::set_marker_id(){
    BR.setParam("/marker_id", fid_id);
}

double Broadcast::get_d(){
    return d;
}

bool Broadcast::start_broadcast(){
    bool broadcast_f;
    BR.getParam("/broadcast_frame", broadcast_f);
    return broadcast_f;
}
