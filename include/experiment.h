#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "mcts.h"
#include "simulator.h"
#include "statistic.h"
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <std_msgs/Int8.h>

//----------------------------------------------------------------------------

struct RESULTS
{
    void Clear();

    STATISTIC Time;
    STATISTIC Reward;
    STATISTIC DiscountedReturn;
    STATISTIC UndiscountedReturn;
};

inline void RESULTS::Clear()
{
    Time.Clear();
    Reward.Clear();
    DiscountedReturn.Clear();
    UndiscountedReturn.Clear();
}

//----------------------------------------------------------------------------

class EXPERIMENT
{
public:

    struct PARAMS
    {
        PARAMS();

        int NumRuns;
        int NumSteps;
        int SimSteps;
        double TimeOut;
        int MinDoubles, MaxDoubles;
        int TransformDoubles;
        int TransformAttempts;
        double Accuracy;
        int UndiscountedHorizon;
        bool AutoExploration;
    };

    struct MRFPARAMS
    {
        MRFPARAMS();

        bool LearnMRF;
        int NumSamples;
        int StateSize;
        int nLearningRuns;
        int nLearningSteps;
        int nSimulations;
        std::string outputPath;
    };

    EXPERIMENT(const SIMULATOR& real, const SIMULATOR& simulator,
        const std::string& outputFile,
        EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams, EXPERIMENT::MRFPARAMS &mrfParams);

    // void Run();
    // void MultiRun();
    // void DiscountedReturn();
    // void AverageReward();

    void LearningMRF(std::string configPath, std::string beliefOutPath, bool online, ros::NodeHandle &nh, int specificConfig);
    void TestingMRF(std::string configPath, std::string beliefOutPath, std::vector<std::pair<int,int>> rels, bool online, ros::NodeHandle &nh, std::string replayOutputPath="", std::string discrOutputPath="", int specificConfig=-1);
    void InitMRF();
    void LoadMRF(std::vector<double> &v);
    void UpdateMRF(std::vector<int> &s, int step);


private:

    const SIMULATOR& Real;
    const SIMULATOR& Simulator;
    EXPERIMENT::PARAMS& ExpParams;
    EXPERIMENT::MRFPARAMS &MRFParams;
    MCTS::PARAMS& SearchParams;
    RESULTS Results;

    Eigen::ArrayXXd mblProbMatrix, mblCounterMatrix;

    std::ofstream OutputFile;

    ros::Publisher resetPub;
    std::string resetPubTopic;
};

//----------------------------------------------------------------------------

#endif // EXPERIMENT_H
