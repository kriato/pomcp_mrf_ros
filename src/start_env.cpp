#include "../include/env.h"

#include <ros/ros.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "env");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    Environment env(nh);

    int rate;
    bool debug;
    nh_private.getParam("rate", rate);
    nh_private.getParam("debug", debug);

    ros::Rate r(rate);
    
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    std::string childFrame;

    while (ros::ok())
    {
        if (debug)
        {
            for (size_t i = 0; i < env.rockNumber; i++)
            {
                transform.setOrigin(tf::Vector3(env.rocksCartesianPositions[i * 2], env.rocksCartesianPositions[i * 2 + 1], 0.0f));
                transform.setRotation(tf::Quaternion(0,0,0,1));

                // transform.header.stamp = ros::Time::now();
                // transform.header.frame_id = "odom";
                // transform.child_frame_id = ;

                childFrame = "rock_" + std::to_string(i);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", childFrame));
            }

            transform.setOrigin(tf::Vector3(env.gridSize * env.squareSize/* + env.squareSize*/, env.gridSize * env.squareSize/* + env.squareSize*/, 0.0f));
            transform.setRotation(tf::Quaternion(0,0,0,1));
            childFrame = "corner_2";
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", childFrame));

            transform.setOrigin(tf::Vector3(env.gridSize * env.squareSize/* + env.squareSize*/, 0.0f, 0.0f));
            transform.setRotation(tf::Quaternion(0,0,0,1));
            childFrame = "corner_1";
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", childFrame));
            
            transform.setOrigin(tf::Vector3(0.0f, env.gridSize * env.squareSize/* + env.squareSize*/, 0.0f));
            transform.setRotation(tf::Quaternion(0,0,0,1));
            childFrame = "corner_3";
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", childFrame));
            
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}