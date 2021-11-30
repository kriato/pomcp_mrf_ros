#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <fstream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "yaml-cpp/yaml.h"
#include "coord.h"
#include "utils.h"
#include "grid.h"
#include "ros_rocksample.h"

class Environment
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber subObservation, subReset;
    ros::Publisher pubObservation, pubEnvInfo;
    tf::TransformListener tfListener;

    std::string obsSubTopic, obsPubTopic, envInfoTopic, yamlPath, robotBaseTF, resetSubTopic;
    bool *isRockValuable, useSpecial11, hardcodedConf, hardcodedConf2, learnMRFConf;
    double halfEfficiencyDistance;
    int seed;
    int RUN_COUNTER;
    std::vector<std::vector<bool>> configs;

    void ObservationCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void ResetCallback(const std_msgs::Int8::ConstPtr &msg);

    enum
    {
        E_NONE, E_GOOD, E_BAD
    };

public:
    float GetDistance(int rock);
    Environment(ros::NodeHandle &nh);
    ~Environment() 
    {
        delete [] isRockValuable;
    };

    std::vector<float> rocksCartesianPositions;
    int rockNumber, gridSize;
    double squareSize;

};

#endif