#include "../include/agent.h"

#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "agent");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Agent agent(nh);

    int rate;
    nh_private.getParam("rate",rate);

    ros::Rate r(rate);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}