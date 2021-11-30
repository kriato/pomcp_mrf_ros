#ifndef ROSROCKSAMPLE_H
#define ROSROCKSAMPLE_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <fstream>

#include <bits/stdc++.h>

#include <unordered_map>
#include <vector>

#define ALWAYS_INSIDE_GRID
// #define ONLINEDEBUG

class ROS_ROCKSAMPLE_STATE : public STATE
{
public:

    COORD AgentPos;
    struct ENTRY
    {
        bool Valuable;
        bool Collected;
        int Count;    				// Smart knowledge
        int Measured; 				// Smart knowledge
        double LikelihoodValuable;	// Smart knowledge
        double LikelihoodWorthless;	// Smart knowledge
        double ProbValuable;		// Smart knowledge
    };
    std::vector<ENTRY> Rocks;
    int Target; // Smart knowledge
};

class ROS_ROCKSAMPLE : public SIMULATOR
{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pubAgentGoal, pubAskForObservation;
    ros::Subscriber subEnvInfo;

    std::string rockCoordsTopic, agentGoalTopic, envInfoTopic, getObservationTopic, askObservationTopic, agentFeedbackTopic;

    std::string historyOutputFile;
    mutable MEMORY_POOL<ROS_ROCKSAMPLE_STATE> MemoryPool;


public:
    ROS_ROCKSAMPLE(int size, int rocks, ros::NodeHandle &nh, UTILS::RND &rnd, bool isReal_ = false, bool online_ = false, bool learnMRF_ = false);

    virtual STATE* CreateStartState() const;
    virtual void UpdateConfig(std::vector<int> &vals) const;

    virtual bool Step(STATE& state, int action, int& observation, double& reward, const BELIEF_STATE& beliefState) const;

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual void FreeState(STATE* state) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history, int stepObservation, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState, std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;
    
    virtual void DisplayBeliefDistribution(const BELIEF_STATE& beliefState,std::ostream& ostr) const;
    std::vector<double*>* CreateStateRelKnowledge(const STATE& state, int nConnComp, double relProbab) const;
    STATE* CreateStartState(std::vector<STATE*>* allParticles, std::vector<double> allParticleCumProbs, int id=-1) const;    
    
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObservation, const STATUS& status, std::vector<double*>* stateVarRelationships) const;

    virtual void PropagateChange(STATE& state, int changedVariableIndex, std::vector<double*>* stateVarRelationships, std::vector<int>* alreadyExploredVarIndices) const;
    virtual void PropagateChangeToConnectedComponent(STATE& state, int changedVariable, int newVal, std::vector<double*>* stateVarRelationships) const;

    virtual std::vector<double*> FindUnexploredAdjacentVariableIndices(int currentVarIndex, std::vector<double*>* stateVarRelationships, std::vector<int>* alreadyExploredVarIndices) const;
    virtual double ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const;
    virtual void ResetAgent() const;

    // ------------------------------- ADDED -------------------------------------------
    virtual void writeHistory(std::string what, bool append = true) const;
    virtual void Id2State(const int id, std::vector<int> &state) const;
    virtual void GetBeliefDistribution(const BELIEF_STATE &beliefState, std::vector<std::pair<int, int>> &beliefs) const;
    void GenerateLegal(const STATE& state, const HISTORY& history, std::vector<int>& legal, const STATUS& status) const;

    // Actually never tried, only when tree knowledge is set to SMART
    void GeneratePreferred(const STATE& state, const HISTORY& history, std::vector<int>& legal, const STATUS& status) const;

protected:

    enum
    {
        E_NONE, E_GOOD, E_BAD
    };

    enum
    {
        E_SAMPLE = 4
    };

    GRID<int> Grid;
    int Size, NumRocks;
    double SmartMoveProb, RewardMove, RewardCheck, RewardExitEast;
    int UncertaintyCount;
    std::vector<COORD> RockPos;
    COORD StartPos;
    double HalfEfficiencyDistance, squareSize;
    int *isRockValuable;
    bool learnMRF;

    int SelectTarget(const ROS_ROCKSAMPLE_STATE& rockstate) const;
    int GetObservation(const ROS_ROCKSAMPLE_STATE& rockstate, int rock, float &dist) const;

    // To be used with const_cast
    void waitForMessage(std::string topic, std_msgs::Float32MultiArray::ConstPtr &msg);
    void waitForMessage(std::string topic, std_msgs::Int8::ConstPtr &msg);

    
};

#endif
