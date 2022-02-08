#include "../include/Broadcast.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "broadcaster");
    ros::NodeHandle BR;
    Broadcast broadcaster(&BR);
    ros::Rate r(10);
    XmlRpc::XmlRpcValue marker_pos;
    while(ros::ok()){
        if (broadcaster.get_d() < 2) BR.setParam("/capture", true);
        else BR.setParam("/capture", false);
        ros::spinOnce();
        broadcaster.set_marker_id();
        r.sleep();
    }
}
