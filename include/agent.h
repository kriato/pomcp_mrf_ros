#ifndef AGENT_H
#define AGENT_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>

#define NO_ODOM -999

class Agent
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tfListener;

    ros::Subscriber subGoal;
    ros::Publisher pubCmdVel, pubFeedback, pubMoveBaseGoal;

    std::string goalTopic, odomFrame, baseFrame, cmdVelTopic, agentFeedbackTopic, turtlebotModel, moveBaseTopic, moveBaseFBTopic;

    geometry_msgs::Twist stop, rotateRight, rotateLeft, goStraight;

    float angularVel, linearVel, squareSize, linearError, angularError;
    double offset2centre, agentX0, agentY0;
    int rate;
    bool useNavigationStack;

    void getOdom(double &angle, geometry_msgs::Point &pos);
    void goToGoal(const geometry_msgs::Point::ConstPtr &goalGridCoords);

public:
    Agent(ros::NodeHandle &nh);
    ~Agent() {};

};

#endif