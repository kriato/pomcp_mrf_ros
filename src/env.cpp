#include "../include/env.h"

Environment::Environment(ros::NodeHandle &nh)
:   nh_(nh),
    private_nh_("~")
{
    RUN_COUNTER = 0;
    nh_.getParam("plan2env_obs_topic", obsSubTopic);
    nh_.getParam("env2plan_obs_topic", obsPubTopic);
    nh_.getParam("env_info_topic", envInfoTopic);
    nh_.getParam("square_size", squareSize);
    nh_.getParam("robot_base_tf", robotBaseTF);
    nh_.getParam("reset_pub_topic", resetSubTopic);

    private_nh_.getParam("half_efficiency_distance", halfEfficiencyDistance);
    private_nh_.getParam("yaml_path", yamlPath);
    private_nh_.getParam("seed", seed);

    nh_.getParam("learn_mrf_conf", learnMRFConf);
    nh_.getParam("use_special_11", useSpecial11);
    nh_.getParam("hardcoded_conf", hardcodedConf);
    nh_.getParam("hardcoded_conf2", hardcodedConf2);

    std::string configPath;
    if (learnMRFConf)
    {
        gridSize = 5;
        rockNumber = 8;
        nh.getParam("config_path", configPath);
    }
    if (useSpecial11 || hardcodedConf)
    {
        gridSize = 11;
        rockNumber = 11;
    }
    else
    {
        nh_.getParam("grid_size", gridSize);
        nh_.getParam("rock_number", rockNumber);
    }

    subReset = nh_.subscribe(resetSubTopic, 1, &Environment::ResetCallback, this);
    subObservation = nh_.subscribe(obsSubTopic, 1, &Environment::ObservationCallback, this);
    pubObservation = nh_.advertise<std_msgs::Float32MultiArray>(obsPubTopic, 1);
    pubEnvInfo = nh_.advertise<std_msgs::Float32MultiArray>(envInfoTopic, 1, true);

    UTILS::RandomSeed(seed);

    std_msgs::Float32MultiArray envInfo = std_msgs::Float32MultiArray();
    envInfo.data.clear();

    COORD StartPos;
    COORD rocks[rockNumber];

    std::vector<std::string> lines;
    std::string line;
    std::ifstream in(configPath);

    if (learnMRFConf)
    {
        rocks[0] = COORD(0, 4);
        rocks[1] = COORD(2, 4);
        rocks[2] = COORD(1, 1);
        rocks[3] = COORD(1, 0);
        rocks[4] = COORD(1, 4);
        rocks[5] = COORD(0, 3);
        rocks[6] = COORD(2, 1);
        rocks[7] = COORD(3, 2);

        StartPos = COORD(0, 0);

        std::cout << "Config path: " << configPath << std::endl;
        if (in.is_open())
        {
            while (std::getline(in, line))
            {
                lines.push_back(line);
            }
        }

        for (size_t i = 0; i < lines.size(); i++)
        {
            std::vector<bool> t;

            size_t start;
            size_t end = 0;
            bool tmp_bool;
            while ((start = lines[i].find_first_not_of('\t', end)) != std::string::npos)
            {
                end = lines[i].find('\t', start);
                auto val = atoi(lines[i].substr(start, end - start).c_str()) == 1 ? true : false;
                t.push_back(val);
            }
            
            if (t.size() != rockNumber)
            {
                std::cerr << rockNumber << " != " << t.size() << std::endl;
                std::cerr << "State size in configurations is different from the parameter set." << std::endl;
                exit(-1);
            }

            configs.push_back(t);
        }

        // if (configs.size() != MRFParams.nLearningRuns)
        // {
        //     std::cerr << MRFParams.nLearningRuns << " != " << configs.size() << std::endl;
        //     std::cerr << "Number of configurations is different from number of runs." << std::endl;
        //     exit(-1);
        // }
    } 
    else if (useSpecial11)
    {
        rocks[0] = COORD(0, 3);
        rocks[1] = COORD(0, 7);
        rocks[2] = COORD(1, 8);
        rocks[3] = COORD(2, 4);
        rocks[4] = COORD(3, 3);
        rocks[5] = COORD(3, 8);
        rocks[6] = COORD(4, 3);
        rocks[7] = COORD(5, 8);
        rocks[8] = COORD(6, 1);
        rocks[9] = COORD(9, 3);
        rocks[10] = COORD(9, 9);

        StartPos = COORD(0, 5);
    }
    else if (hardcodedConf)
    {
        rocks[0] = COORD(0, 0);
        rocks[1] = COORD(1, 1);
        rocks[2] = COORD(2, 2);
        rocks[3] = COORD(3, 3);
        rocks[4] = COORD(4, 4);
        rocks[5] = COORD(5, 5);
        rocks[6] = COORD(6, 6);
        rocks[7] = COORD(7, 7);
        rocks[8] = COORD(8, 8);
        rocks[9] = COORD(9, 9);
        rocks[10] = COORD(10, 10);

        StartPos = COORD(0, 0);
    }
    else if (hardcodedConf2)
    {
        rocks[0] = COORD(0, 5);
        rocks[1] = COORD(1, 5);
        rocks[2] = COORD(2, 5);
        rocks[3] = COORD(3, 5);
        rocks[4] = COORD(4, 5);
        rocks[5] = COORD(5, 5);
        rocks[6] = COORD(6, 5);
        rocks[7] = COORD(7, 5);
        rocks[8] = COORD(8, 5);
        rocks[9] = COORD(9, 5);
        rocks[10] = COORD(10, 5);

        StartPos = COORD(0, 5);
    }
    else
    {
        StartPos = COORD(0, gridSize / 2);
    }

    envInfo.data.push_back(halfEfficiencyDistance);

    std::vector<COORD> RockPos;
    GRID<int> grid(gridSize, gridSize);
    isRockValuable = new bool[rockNumber];
    rocksCartesianPositions = std::vector<float>();

    envInfo.data.push_back(StartPos.X);
    envInfo.data.push_back(StartPos.Y);

    grid.SetAllValues(-1);

    std::cout << "Agent S0: " << StartPos << std::endl;

    for (size_t i = 0; i < rockNumber; i++)
    {
        COORD pos;

        if (useSpecial11 || hardcodedConf || hardcodedConf2 || learnMRFConf)
        {
            pos = rocks[i];
        }
        else
        {
            do
            {
                pos = COORD(UTILS::Random(gridSize), UTILS::Random(gridSize));
            }
            while (grid(pos) >= 0);
        }

        grid(pos) = i;
        RockPos.push_back(pos);

        envInfo.data.push_back(pos.X);
        envInfo.data.push_back(pos.Y);

        rocksCartesianPositions.push_back(pos.X * squareSize + squareSize / 2.0f);
        rocksCartesianPositions.push_back(pos.Y * squareSize + squareSize / 2.0f);

        std::cout << "Rock " << i << ": " << pos << " ";

        if (learnMRFConf)
        {
            std::string printme = configs[0][i] ? "VAL\n" : "NOT VAL\n";
            envInfo.data.push_back(-1.0f);
            std::cout << printme;
        }
        else
        {
            isRockValuable[i] = UTILS::Bernoulli(0.5);
            if (isRockValuable[i])
            {
                envInfo.data.push_back(1.0f);
                std::cout << "VAL\n";
            }
            else
            {
                envInfo.data.push_back(0.0f);
                std::cout << "NOT VAL\n";
            }
        }
    }

    pubEnvInfo.publish(envInfo);
    ros::spinOnce();

    YAML::Node agentYaml = YAML::Node();
    agentYaml["initial_pose_grid_x"] = StartPos.X;
    agentYaml["initial_pose_grid_y"] = StartPos.Y;
    agentYaml["initial_pose_cart_x"] = StartPos.X * squareSize + squareSize / 2.0f;
    agentYaml["initial_pose_cart_y"] = StartPos.Y * squareSize + squareSize / 2.0f;

    std::cout << agentYaml << std::endl;
    std::ofstream out(yamlPath);
    out << agentYaml;
    out.close();

    ROS_INFO("Published env info.");
}

float Environment::GetDistance(int rock)
{
    tf::StampedTransform tfs;

    try
    {
        tfListener.lookupTransform("rock_" + std::to_string(rock), robotBaseTF, ros::Time(0), tfs);

        return sqrt(pow(tfs.getOrigin().getX(), 2) + pow(tfs.getOrigin().getY(), 2));

    }
    catch (tf2::TransformException &e)
    {
        ROS_WARN("%s", e.what());
        return -1;
    }
}

void Environment::ObservationCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    // Get real position of the agent

    ros::Duration(0.5).sleep();
    COORD agent(msg->data[0], msg->data[1]);
    int rockIdx = (int) msg->data[2];
    COORD rock(msg->data[3], msg->data[4]);

    float distance = GetDistance(rockIdx);
    if (distance == -1)
    {
        distance = COORD::EuclideanDistance(agent, rock) * squareSize;
    }

    // std::cout << "Agent: " << agent << std::endl;
    // std::cout << "Rock: " << rock << std::endl;
    // std::cout << distance << std::endl;
    double efficiency = (1 + pow(2, -distance / halfEfficiencyDistance)) * 0.5;

    std_msgs::Float32MultiArray obs = std_msgs::Float32MultiArray();

    std::string out_obs;
    std::cout << "Observing rock " << rockIdx;
    if (UTILS::Bernoulli(efficiency))
    {
        if (learnMRFConf)
        {
            obs.data.push_back(configs[RUN_COUNTER][rockIdx] ? E_GOOD : E_BAD);
            out_obs = configs[RUN_COUNTER][rockIdx] ? "GOOD" : "BAD";
        }
        else
        {
            obs.data.push_back(isRockValuable[rockIdx] ? E_GOOD : E_BAD);
            out_obs = isRockValuable[rockIdx] ? "GOOD" : "BAD";
        }

        std::cout << " ---> " << out_obs << "  V" << std::endl;
    }
    else
    {
        if (learnMRFConf)
        {
            obs.data.push_back(configs[RUN_COUNTER][rockIdx] ? E_BAD : E_GOOD);
            out_obs = configs[RUN_COUNTER][rockIdx] ? "BAD" : "GOOD";
        }
        else
        {
            obs.data.push_back(isRockValuable[rockIdx] ? E_BAD : E_GOOD);
            out_obs = isRockValuable[rockIdx] ? "BAD" : "GOOD";
        }

        std::cout << " ---> " << out_obs << "  X"  << std::endl;
    }

    obs.data.push_back(distance);
    pubObservation.publish(obs);
}

// Reset env will change config, increase RUN COUNTER
void Environment::ResetCallback(const std_msgs::Int8::ConstPtr &msg)
{
    RUN_COUNTER++;

    ROS_INFO("Reset done.");
    std::cout << "STATE: \t\t";
    for (size_t j = 0; j < configs[RUN_COUNTER].size(); j++)
        std::cout << configs[RUN_COUNTER][j] << " ";
    std::cout << std::endl;
}