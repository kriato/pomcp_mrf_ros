#include "../include/agent.h"

Agent::Agent(ros::NodeHandle &nh)
:   nh_(nh),
    private_nh_("~")
{
    private_nh_.getParam("angular_vel", angularVel);
    private_nh_.getParam("linear_vel", linearVel);  
    private_nh_.getParam("odom_frame", odomFrame);  
    private_nh_.getParam("base_frame", baseFrame);  
    private_nh_.getParam("cmd_vel_topic", cmdVelTopic);
    private_nh_.getParam("rate", rate);
    private_nh_.getParam("linear_error", linearError);  
    private_nh_.getParam("angular_error", angularError);

    private_nh_.getParam("move_base_topic", moveBaseTopic);
    private_nh_.getParam("move_base_fb_topic", moveBaseFBTopic);
    private_nh_.getParam("turtlebot_model", turtlebotModel);
    private_nh_.getParam("initial_pose_cart_x", agentX0);
    private_nh_.getParam("initial_pose_cart_y", agentY0);
    private_nh_.getParam("use_move_base", useNavigationStack);
      
    nh_.getParam("agent_fb_topic", agentFeedbackTopic);  
    nh_.getParam("agent_goal_topic", goalTopic);
    nh_.getParam("square_size", squareSize);

    subGoal = nh_.subscribe(goalTopic, 1, &Agent::goToGoal, this);
    pubFeedback = nh_.advertise<std_msgs::Int8>(agentFeedbackTopic, 1);
    
    if (useNavigationStack) 
    {
        pubMoveBaseGoal = nh_.advertise<geometry_msgs::PoseStamped>(moveBaseTopic, 1);
    }
    else
    {
        stop = geometry_msgs::Twist();
        rotateRight = geometry_msgs::Twist();
        rotateLeft = geometry_msgs::Twist();
        goStraight = geometry_msgs::Twist();

        rotateRight.angular.z = -angularVel;
        rotateLeft.angular.z = angularVel;
        goStraight.linear.x = linearVel;
        pubCmdVel = nh_.advertise<geometry_msgs::Twist>(cmdVelTopic, 1);
    }

    pid_t child = fork();
    if (child == 0)
    {
        std::string del_cmd = "rosservice call gazebo/delete_model '{model_name: turtlebot3_"+ turtlebotModel + "}'";
        int ret = system(del_cmd.c_str());

        std::string spawn_cmd = "rosrun gazebo_ros spawn_model -urdf -model turtlebot3_" + turtlebotModel + " -x " + std::to_string(agentX0) + " -y " + std::to_string(agentY0) + " -z 0 -param robot_description";
        ret = system(spawn_cmd.c_str());

        if (ret != 0)
        {
            ROS_ERROR("Agent not spawned.");
            exit(-1);
        }    
    
    }
    
    ROS_INFO("Agent started.");
    ros::Duration(3.0f).sleep();
}

void Agent::getOdom(double &angle, geometry_msgs::Point &pos)
{
    tf::StampedTransform tfs;
    try
    {
        tfListener.lookupTransform(odomFrame, baseFrame, ros::Time(0), tfs);

        double r, p, y;
        tf::Matrix3x3(tfs.getRotation()).getRPY(r, p, y); // X Y Z

        angle = y * (180.0f/M_PI);
        pos.x = tfs.getOrigin().getX();
        pos.y = tfs.getOrigin().getY();
        pos.z = tfs.getOrigin().getZ();

    }
    catch (tf2::TransformException &e)
    {
        ROS_WARN("%s", e.what());
        angle = NO_ODOM;
    }
}

void Agent::goToGoal(const geometry_msgs::Point::ConstPtr &goalGridCoords)
{
    double angle;
    geometry_msgs::Point pos = geometry_msgs::Point();
    ros::Rate r(10);

    double goalX = goalGridCoords->x * squareSize + squareSize / 2.0f;
    double goalY = goalGridCoords->y * squareSize + squareSize / 2.0f;

    std::cout << "New goal received: [" << goalX << "," << goalY << "]" << std::endl;

    if (useNavigationStack) 
    {   
        // std::string tmp;
        // std::cin >> tmp;
        geometry_msgs::PoseStamped goalMsg = geometry_msgs::PoseStamped();
        goalMsg.header.frame_id = "odom";
        goalMsg.header.stamp = ros::Time::now();
        goalMsg.pose.position.x = goalX;
        goalMsg.pose.position.y = goalY;
        goalMsg.pose.orientation.w = 1;
        pubMoveBaseGoal.publish(goalMsg);
        ros::spinOnce();

        ROS_INFO("Waiting for move_base feedback...");
        auto res = ros::topic::waitForMessage<move_base_msgs::MoveBaseActionResult>(moveBaseFBTopic, nh_, ros::Duration(500.0f));

        if (res)
        {
            if (res->status.status == res->status.SUCCEEDED)
            {
                ROS_INFO("SUCCEEDED");
                pubFeedback.publish(std_msgs::Int8());
                ros::spinOnce();
            }
            // else if (res->status.status == res->status.REJECTED)
            // {
            //     ROS_ERROR("REJECTED");
            //     exit(1);
            // }
            // else if (res->status.status == res->status.ABORTED)
            // {
            //     ROS_ERROR("ABORTED");
            //     exit(1);
            // }
            // else
            // {
            //     ROS_ERROR("Unknown error");
            //     std::cout << "CODE: " << res->status.status << std::endl;
            //     exit(1);
            // }
        } 
        else
        {
            ROS_INFO("Move base timeout.");
            pubFeedback.publish(std_msgs::Int8());
            ros::spinOnce();
            // exit(1);
        }
    } // Stupid behaviour
    else 
    {
        while (true)
        {
            getOdom(angle, pos);

            if (angle == NO_ODOM)
            {
                ROS_WARN("Failed to get odometry.");
                continue;
            }

            double distance = sqrt(pow(goalX - pos.x, 2) + pow(goalY - pos.y, 2));
            double heading = atan2(goalY - pos.y, goalX - pos.x) * (180.0f/M_PI);
            double goal_angle = angle - heading;
            
            if (goal_angle < -angularError)
            {
                // ROS_INFO("Rotate left.");
                pubCmdVel.publish(rotateLeft);
            }
            else if (goal_angle > angularError)
            {
                // ROS_INFO("Rotate right.");
                pubCmdVel.publish(rotateRight);
            }
            else if (distance > linearError)
            {
                // ROS_INFO("Go straight.");
                pubCmdVel.publish(goStraight);
            }
            else
            {
                ROS_INFO("Goal reached.");
                pubCmdVel.publish(stop);
                pubFeedback.publish(std_msgs::Int8());
                ros::spinOnce();
                r.sleep();
                break;
            }

            ros::spinOnce();
            r.sleep();
        }
    }
}