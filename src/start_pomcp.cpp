#include "../include/mcts.h"
#include "../include/ros_rocksample.h"
#include "../include/experiment.h"

void disableBufferedIO(void)
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    setbuf(stderr, NULL);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "pomcp", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    MCTS::PARAMS searchParams;
    EXPERIMENT::PARAMS expParams;
    SIMULATOR::KNOWLEDGE knowledge;
    EXPERIMENT::MRFPARAMS mrfParams;

    int size, number, seed, specificConfig;
    std::string problem, outputfile, policy, configPath, beliefOutputPath, rndOutputPath, replayOutputPath, discrOutputPath, historyPath;
    bool test, useSpecial11, hardcodedConf, hardcodedConf2, online, /*learnMRFConf,*/ clique;

    private_nh.getParam("learnMRF", mrfParams.LearnMRF);
    private_nh.getParam("problem", problem);
    nh.getParam("use_special_11", useSpecial11);
    nh.getParam("hardcoded_conf", hardcodedConf);
    nh.getParam("hardcoded_conf2", hardcodedConf2);
    nh.getParam("config_path", configPath);

    if (problem == "rosrocksample")
    {
        if (mrfParams.LearnMRF /*&& learnMRFConf*/)
        {
            ROS_INFO("Learning/Testing MRF with info given by env node.");
            nh.getParam("grid_size", size);
            nh.getParam("rock_number", number);
        }
        else if (mrfParams.LearnMRF)
        {
            private_nh.getParam("learning_grid_size", size);
            private_nh.getParam("learning_rock_number", number);
        }
        else if (useSpecial11 || hardcodedConf || hardcodedConf2)
        {
            size = 11;
            number = 11;
        }
        else
        {
            nh.getParam("grid_size", size);
            nh.getParam("rock_number", number);
        }
    }
    else
    {
        ROS_ERROR("The problem is not implemented.\n");
        exit(1);
    }

    private_nh.getParam("online", online);
    private_nh.getParam("outputfile", outputfile);
    private_nh.getParam("policy", policy);
    private_nh.getParam("test", test);
    private_nh.getParam("belief_outputfile", beliefOutputPath);
    private_nh.getParam("rnd_outputfile", rndOutputPath);
    private_nh.getParam("replay_bel_outputfile", replayOutputPath);
    private_nh.getParam("discr_outfile", discrOutputPath);
    private_nh.getParam("config_number", specificConfig);

    private_nh.getParam("verbose", searchParams.Verbose);
    private_nh.getParam("exploration", searchParams.ExplorationConstant);
    private_nh.getParam("usetransforms", searchParams.UseTransforms);
    private_nh.getParam("ravediscount", searchParams.RaveDiscount);
    private_nh.getParam("disabletree", searchParams.DisableTree);
    private_nh.getParam("raveconstant", searchParams.RaveConstant);
    private_nh.getParam("userave", searchParams.UseRave);

    private_nh.getParam("nsamples", mrfParams.NumSamples);
    private_nh.getParam("learning_runs", mrfParams.nLearningRuns);
    private_nh.getParam("learning_steps", mrfParams.nLearningSteps);
    private_nh.getParam("learning_sims", mrfParams.nSimulations);
    private_nh.getParam("output_path", mrfParams.outputPath);
    mrfParams.StateSize = number;

    private_nh.getParam("timeout", expParams.TimeOut);
    private_nh.getParam("mindoubles", expParams.MinDoubles);
    private_nh.getParam("maxdoubles", expParams.MaxDoubles);
    private_nh.getParam("runs", expParams.NumRuns);
    private_nh.getParam("accuracy", expParams.Accuracy);
    private_nh.getParam("horizon", expParams.UndiscountedHorizon);
    private_nh.getParam("numsteps", expParams.NumSteps);
    private_nh.getParam("autoexploration", expParams.AutoExploration);
    private_nh.getParam("transformdoubles", expParams.TransformDoubles);
    private_nh.getParam("transformattempts", expParams.TransformAttempts);

    private_nh.getParam("smarttreevalue", knowledge.SmartTreeValue);
    private_nh.getParam("treeknowledge", knowledge.TreeLevel);
    private_nh.getParam("rolloutknowledge", knowledge.RolloutLevel);
    private_nh.getParam("smarttreecount", knowledge.SmartTreeCount);
    private_nh.getParam("relknowlevel", knowledge.relKnowLevel);
    private_nh.getParam("clique", clique);

    SIMULATOR *real = 0;
    SIMULATOR *simulator = 0;

    private_nh.getParam("seed", seed);
    UTILS::RND RND(seed, mrfParams.nLearningRuns, rndOutputPath);

    #ifdef RNDGENINTERFACE
        // RND.SetSeed();
    #else
        UTILS::RandomSeed(seed);
    #endif
    
    
    if (problem == "rosrocksample")
    {
        real = new ROS_ROCKSAMPLE(size, number, nh, RND, true, online, mrfParams.LearnMRF);
        simulator = new ROS_ROCKSAMPLE(size, number, nh, RND, false, online, mrfParams.LearnMRF);
    }
    else
    {
        ROS_ERROR("The problem is not implemented.\n");
        exit(1);
    }

    simulator->SetKnowledge(knowledge);
    EXPERIMENT experiment(*real, *simulator, outputfile, expParams, searchParams, mrfParams);

    if (mrfParams.LearnMRF && knowledge.relKnowLevel == 0)
    {
        ROS_INFO("LEARNING MRF.");
        experiment.LearningMRF(configPath, beliefOutputPath, online, nh, specificConfig);
    }
    else if (mrfParams.LearnMRF && knowledge.relKnowLevel == 1)
    {
        ROS_INFO("TESTING MRF ");

        std::vector<double> mrf;
        private_nh.getParam("mrf", mrf);

        if (mrf.size() != number*number)
        {
            std::cerr << mrf.size() << " != " << number*number << std::endl;
            ROS_ERROR("Size of MRF does not correspond to the size of the state.");
        }

        std::vector<double> rels_raw;
        private_nh.getParam("rels", rels_raw);
        std::vector<std::pair<int,int>> rels;

        for (size_t i = 0; i < rels_raw.size(); i+=2)
        {
            rels.push_back(std::make_pair(rels_raw[i], rels_raw[i + 1]));
        }

        experiment.LoadMRF(mrf);
        experiment.TestingMRF(configPath, beliefOutputPath, rels, online, nh, "", "", specificConfig);
    }
    else
    {
        ROS_ERROR("CONFIG FILE WRONG.");
    }

    delete real;
    delete simulator;

    return 0;
}


