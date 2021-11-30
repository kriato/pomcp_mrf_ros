#include "../include/experiment.h"
#include "boost/timer.hpp"

using namespace std;

EXPERIMENT::PARAMS::PARAMS()
:   NumRuns(1000),
    NumSteps(100000),
    SimSteps(1000),
    TimeOut(3600),
    MinDoubles(0),
    MaxDoubles(20),
    TransformDoubles(-4),
    TransformAttempts(1000),
    Accuracy(0.01),
    UndiscountedHorizon(1000),
    AutoExploration(true)
{
}

EXPERIMENT::MRFPARAMS::MRFPARAMS()
:   LearnMRF(false),
    // NumConnComp(3),
    NumSamples(10000),
    // RelProb(0.9),
    StateSize(-1),
    nLearningRuns(100),
    nLearningSteps(60),
    nSimulations(100000),
    outputPath("")
{
}

EXPERIMENT::EXPERIMENT(const SIMULATOR& real,
    const SIMULATOR& simulator, const string& outputFile,
    EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams, EXPERIMENT::MRFPARAMS &mrfParams)
:   Real(real),
    Simulator(simulator),
    OutputFile(outputFile.c_str()),
    ExpParams(expParams),
    MRFParams(mrfParams),
    SearchParams(searchParams)
{
    if (ExpParams.AutoExploration)
    {
        if (SearchParams.UseRave)
            SearchParams.ExplorationConstant = 0;
        else
            SearchParams.ExplorationConstant = simulator.GetRewardRange();
    }
    
    MCTS::InitFastUCB(SearchParams.ExplorationConstant);
}

void EXPERIMENT::UpdateMRF(std::vector<int> &s, int step)
{
    // Updating matrix
    for (size_t i = 0; i < s.size(); i++)
    {
        for (size_t j = i + 1; j < s.size(); j++)
        {
            // std::cout << "Comparing " << i << " " << j << "\n";

            if (s[i] == s[j])
            {
                mblCounterMatrix(i,j) += 1;
            }
            
            mblProbMatrix(i,j) = mblCounterMatrix(i,j) / step;
        }
    }
    // std::cout << mblMatrix << std::endl;
}

void EXPERIMENT::LoadMRF(std::vector<double> &v)
{
    Eigen::Map<Eigen::ArrayXXd> m(v.data(), MRFParams.StateSize, MRFParams.StateSize);
    mblProbMatrix = m.transpose();

    // std::cout << mblProbMatrix << std::endl;
}

void EXPERIMENT::InitMRF()
{
    // Initialize UT to 0 and the rest to -1
    mblProbMatrix = Eigen::ArrayXXd::Constant(MRFParams.StateSize, MRFParams.StateSize, -1);
    mblCounterMatrix = Eigen::ArrayXXd::Constant(MRFParams.StateSize, MRFParams.StateSize, -1);

    for (size_t i = 0; i < MRFParams.StateSize; i++)
    {
        for (size_t j = 0; j < MRFParams.StateSize; j++)
        {
            if (i < j)
            {
                mblProbMatrix(i,j) = 0;
                mblCounterMatrix(i,j) = 0;
            }
        }
    }
    // std::cout << mblMatrix << std::endl;
}

void EXPERIMENT::LearningMRF(std::string configPath, std::string beliefOutPath, bool online, ros::NodeHandle &nh, int specificConfig)
{
    nh.getParam("reset_pub_topic", resetPubTopic);
    resetPub = nh.advertise<std_msgs::Int8>(resetPubTopic, 1);

    InitMRF();

    int t;
    double undiscountedReturn, discountedReturn, discount;
    bool terminal, outOfParticles;
    MCTS *mcts;
    STATE *state;

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    // Forcing number of simulations
    if (ExpParams.NumSteps == 100000)
    {
        ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    }

    SearchParams.NumSimulations = MRFParams.nSimulations;
    SearchParams.NumStartStates = MRFParams.nSimulations;

    if (MRFParams.nSimulations + ExpParams.TransformDoubles >= 0)
        SearchParams.NumTransforms = 1 << (MRFParams.nSimulations + ExpParams.TransformDoubles);
    else
        SearchParams.NumTransforms = 1;
    SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;


    // Clean content of file
    std::ofstream mrfOutput, summary;
    mrfOutput.open(MRFParams.outputPath);
    mrfOutput.close();

    // Load config file into vector of configurations
    std::vector<std::string> lines;
    std::vector<std::vector<int>> configs;
    std::string line;
    
    std::ifstream in(configPath);

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
        std::vector<int> t;

        size_t start;
        size_t end = 0;
        while ((start = lines[i].find_first_not_of('\t', end)) != std::string::npos)
        {
            end = lines[i].find('\t', start);
            auto val = atoi(lines[i].substr(start, end - start).c_str());
            t.push_back(val);
        }
        
        if (t.size() != MRFParams.StateSize)
        {
            std::cerr << MRFParams.StateSize << " != " << t.size() << std::endl;
            std::cerr << "State size in configurations is different from the parameter set." << std::endl;
            ::exit(-1);
        }

        configs.push_back(t);
    }

    if (configs.size() != MRFParams.nLearningRuns)
    {
        std::cerr << MRFParams.nLearningRuns << " != " << configs.size() << std::endl;
        std::cerr << "Number of configurations is different from number of runs." << std::endl;
        ::exit(-1);
    }

    for (size_t i = 0; i < MRFParams.nLearningRuns; i++)
    {        
        if (i != specificConfig && specificConfig != -1)
        {
            // Real.RND->verbose = false;
            Real.RND->NextSeed();
            continue;
        }
        
        Real.RND->PrintSeed();

        std::cout << "STATE: \t\t";
        for (size_t j = 0; j < configs[i].size(); j++)
            std::cout << configs[i][j] << " ";
        std::cout << std::endl;

        // Init parameters
        undiscountedReturn = 0.0f;
        discountedReturn = 0.0f;
        discount = 1.0f;
        terminal = false;
        outOfParticles = false;

        // Init start state with given configurations
        Real.UpdateConfig(configs[i]);
        state = Real.CreateStartState();

        Real.RND->SetCheckpoint();
        mcts = new MCTS(Simulator, SearchParams);
        Real.RND->RetrieveCheckpoint();


        for (t = 0; t < MRFParams.nLearningSteps; t++)
        {
            // std::system("clear");
            std::cout << "Step " << t + 1 << "/" << MRFParams.nLearningSteps << "\t | ";
            std::cout << "Run " << i + 1 << "/" << MRFParams.nLearningRuns;

            int observation;
            double reward;
            int action = mcts->SelectAction();

            terminal = Real.Step(*state, action, observation, reward, mcts->BeliefState());
            
            Results.Reward.Add(reward);
            undiscountedReturn += reward;
            discountedReturn += reward * discount;
            discount *= Real.GetDiscount();

            if (terminal)
            {
                std::cout << "\nTerminated before steps." << endl;
                break;
            }

            outOfParticles = !mcts->Update(action, observation, reward);

            std::vector<std::pair<int, int>> belDistribution;
            Real.GetBeliefDistribution(mcts->BeliefState(), belDistribution);

            std::ofstream tmp;
            tmp.open (beliefOutPath, std::ios::app);
            tmp << mcts->BeliefState().GetNumSamples() << ", " << belDistribution.size() << ", ";
            
            std::vector<int> state;

            for (size_t i = 0; i < belDistribution.size(); i++)
            {
                Real.Id2State(belDistribution[i].first, state);
                for (size_t j = 0; j < state.size(); j++)
                    tmp << state[j] << ", ";

                tmp << belDistribution[i].second << ", ";
            }
            
            tmp  << std::endl;
            tmp.close();

            if (outOfParticles)
                break;

            std::cout << " | " << belDistribution[0].first << " | " << discountedReturn << std::endl;
        }

        if (outOfParticles)
        {
            cout << "Out of particles, finishing episode with SelectRandom" << endl;
            HISTORY history = mcts->GetHistory();
            while (++t < ExpParams.NumSteps)
            {
                int observation;
                double reward;

                int action = Simulator.SelectRandom(*state, history, mcts->GetStatus());

                terminal = Real.Step(*state, action, observation, reward, mcts->BeliefState());

                Results.Reward.Add(reward);
                undiscountedReturn += reward;
                discountedReturn += reward * discount;
                discount *= Real.GetDiscount();

                if (SearchParams.Verbose >= 1)
                {
                    Real.DisplayAction(action, cout);
                    Real.DisplayState(*state, cout);
                    Real.DisplayObservation(*state, observation, cout);
                    Real.DisplayReward(reward, cout);
                }

                if (terminal)
                {
                    cout << "\nTerminated" << endl;
                    break;
                }

                history.Add(action, observation);
            }
        }

        // Retrieve Maximum Likelihood belief
        std::vector<std::pair<int, int>> belDistribution;
        Real.GetBeliefDistribution(mcts->BeliefState(), belDistribution);
        std::vector<int> state;
        Real.Id2State(belDistribution[0].first, state);
        
        UpdateMRF(state, i + 1);

        std::cout << mblProbMatrix << std::endl;

        mrfOutput.open (MRFParams.outputPath, std::ios::app);
        for (size_t i = 0; i < state.size(); i++)
            mrfOutput << state[i] << ", ";

        Eigen::IOFormat CommaInitFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

        mrfOutput << mblProbMatrix.format(CommaInitFmt) << std::endl;
        mrfOutput.close();

        if (online)
        {
            ROS_INFO("Reset.");
            Real.ResetAgent();
            resetPub.publish(std_msgs::Int8());
            ros::spinOnce();
        }
    }

    std::cout << "Learning finished" << std::endl;
    // ::exit(-1);
}

void EXPERIMENT::TestingMRF(std::string configPath, std::string beliefOutPath, std::vector<std::pair<int,int>> rels, bool online, ros::NodeHandle &nh, std::string replayOutputPath, std::string discrOutputPath, int specificConfig)
{
    nh.getParam("reset_pub_topic", resetPubTopic);

    resetPub = nh.advertise<std_msgs::Int8>(resetPubTopic, 1);
    
    std::cout << mblProbMatrix << std::endl;

    int t;
    double undiscountedReturn, discountedReturn, discount;
    bool terminal, outOfParticles;
    boost::timer timer;
    MCTS *mcts = 0;
    STATE *state = 0;

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    if (ExpParams.NumSteps == 100000)
    {
        ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    }

    SearchParams.NumSimulations = MRFParams.nSimulations;
    SearchParams.NumStartStates = MRFParams.nSimulations;

    if (MRFParams.nSimulations + ExpParams.TransformDoubles >= 0)
        SearchParams.NumTransforms = 1 << (MRFParams.nSimulations + ExpParams.TransformDoubles);
    else
        SearchParams.NumTransforms = 1;
    SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

    // // Load config file into vector of configurations
    std::vector<std::string> lines;
    std::vector<std::vector<int>> configs;
    std::string line;
    
    std::ifstream in(configPath);

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
        std::vector<int> t;

        size_t start;
        size_t end = 0;
        while ((start = lines[i].find_first_not_of('\t', end)) != std::string::npos)
        {
            end = lines[i].find('\t', start);
            auto val = atoi(lines[i].substr(start, end - start).c_str());
            t.push_back(val);
        }
        
        if (t.size() != MRFParams.StateSize)
        {
            std::cerr << MRFParams.StateSize << " != " << t.size() << std::endl;
            std::cerr << "State size in configurations is different from the parameter set." << std::endl;
            ::exit(-1);
        }

        configs.push_back(t);
    }

    if (configs.size() != MRFParams.nLearningRuns)
    {
        std::cerr << MRFParams.nLearningRuns << " != " << configs.size() << std::endl;
        std::cerr << "Number of configurations is different from number of runs." << std::endl;
        ::exit(-1);
    }

    std::vector<std::pair<int, int>> history;
    std::vector<double*>* stateVarRelationships;
    std::vector<int> already;
    bool uniformBelief;

    int discrepancyCounter = 0;
    std::vector<std::pair<int, int>> discrepancyLog;
    std::vector<std::pair<int, int>> discrepancyNodes;
    std::vector<std::pair<int, int>> outOfParticlesLog;

    bool DISCR = false;
    
    for (size_t i = 0; i < MRFParams.nLearningRuns; i++)
    {        
        if (i != specificConfig && specificConfig != -1)
        {
            Real.RND->NextSeed();
            continue;
        }
        
        Real.RND->PrintSeed();
        
        
        std::cout << "STATE: \t\t";
        for (size_t j = 0; j < configs[i].size(); j++)
            std::cout << configs[i][j] << " ";
        std::cout << std::endl;

        // Init parameters
        undiscountedReturn = 0.0f;
        discountedReturn = 0.0f;
        discount = 1.0f;
        terminal = false;
        outOfParticles = false;
        
        
        history.clear();        
        already.clear();
        uniformBelief = false;

        stateVarRelationships = new std::vector<double*>();
        for (const auto &r : rels)
        {
            stateVarRelationships->push_back(new double[3] {
                (double) r.first,
                (double) r.second,
                mblProbMatrix(r.first, r.second)
            });
        }

        std::cout << "MRF:" << std::endl;
        for(std::vector<double*>::iterator it = stateVarRelationships->begin() ; it != stateVarRelationships->end(); ++it)
        {
            std::cout << (*it)[0] << ", " << (*it)[1] << ", " << (*it)[2] <<  endl;
        }

        // Init start state with given configurations
        Real.UpdateConfig(configs[i]);
        state = Real.CreateStartState();

        Real.RND->SetCheckpoint();
        if (mcts != 0)
        {
            delete mcts;
        }

        mcts = new MCTS(Simulator, stateVarRelationships, MRFParams.NumSamples, SearchParams);
        Real.RND->RetrieveCheckpoint();

        for (t = 0; t < MRFParams.nLearningSteps; t++)
        {
            // std::system("clear");
            
            std::cout << "Step " << t + 1 << "/" << MRFParams.nLearningSteps << "\t | ";
            std::cout << "Run " << i + 1 << "/" << MRFParams.nLearningRuns;

            int observation, historyObservation;
            double reward;
            int action = mcts->SelectAction();


            terminal = Real.Step(*state, action, observation, reward, mcts->BeliefState());

            history.push_back(std::make_pair(action, observation));

            Results.Reward.Add(reward);
            undiscountedReturn += reward;
            discountedReturn += reward * discount;
            discount *= Real.GetDiscount();

            if (terminal)
            {
                std::cout << "\nTerminated before steps." << endl;
                break;
            }

            // Check discrepancy between MRF and current run
            // if (adaptive)
            // {
                // Adaptive MRF here !   
            // }
            // else
            // {
                // They should have the same behaviour when transforms are not used
                if (uniformBelief)
                {
                    outOfParticles = !mcts->Update(action, observation, reward);
                }
                else
                {
                    outOfParticles = !mcts->Update(action, observation, reward, stateVarRelationships);
                }
            // }

            if (outOfParticles)
            {
                std::ofstream tmp;
                tmp.open (discrOutputPath, std::ios::app);
                tmp << "--- | ";
                tmp << i + 1 << " | " << t + 1 <<std::endl;
                tmp << "--------------OUT----------------" << std::endl;
                tmp.close();

                // Fill history
                for (t = t + 1; t < MRFParams.nLearningSteps; t++)
                {
                    // std::cout << t << std::endl;
                    Real.writeHistory("-1 0 0 0 0");
                }

                ROS_ERROR("Run out of particles.");
                ::exit(-1);
                break;
                
            }

            std::vector<std::pair<int, int>> belDistribution;
            Real.GetBeliefDistribution(mcts->BeliefState(), belDistribution);

            std::ofstream tmp;
            tmp.open (beliefOutPath, std::ios::app);
            tmp << mcts->BeliefState().GetNumSamples() << ", " << belDistribution.size() << ", ";
            
            std::vector<int> state;
           
            for (size_t i = 0; i < belDistribution.size(); i++)
            {
                Real.Id2State(belDistribution[i].first, state);
                for (size_t j = 0; j < state.size(); j++)
                    tmp << state[j] << ", ";

                tmp << belDistribution[i].second << ", ";
            
                if (i == 6)
                    break;
            }
            
            tmp  << std::endl;

            tmp.close();

            std::cout << " | " << discountedReturn << std::endl;

            Real.RND->Log();
        }

        if (online)
        {
            ROS_INFO("Reset.");
            Real.ResetAgent();
            resetPub.publish(std_msgs::Int8());
            ros::spinOnce();
        }

        Real.RND->PrintSeed();
    }

    std::cout << "Discrepancies: " << discrepancyCounter << std::endl;
    std::cout << "Discrepancy log of size " << discrepancyLog.size() << std::endl;

    std::ofstream tmp;
    tmp.open (discrOutputPath, std::ios::app);
    for (size_t idx = 0; idx < discrepancyLog.size(); idx++)
    {
        tmp << discrepancyLog[idx].first + 1 << " | " << discrepancyLog[idx].second + 1 << " (" << discrepancyNodes[idx].first << 
            "|" << discrepancyNodes[idx].second << ")" <<std::endl;
    }
    tmp << "----------------------------------" << std::endl;
    tmp.close();

    // ::exit(-1);

    return;
}
//----------------------------------------------------------------------------
