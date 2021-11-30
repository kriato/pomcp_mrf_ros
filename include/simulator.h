#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "history.h"
#include "node.h"
#include "utils.h"
#include <iostream>
#include <math.h>

class BELIEF_STATE;

class STATE : public MEMORY_OBJECT
{
public:
    int id = -1;
};

class CHECKPOINT
{
};

class VHISTORY
{
public:
    bool initialized = false;
    bool sim;
    // VHISTORY() : initialized(false) {};
    virtual void Clear() = 0;
    virtual void Print() = 0;
    virtual int Size() = 0;
};

class SIMULATOR
{
public:

    struct KNOWLEDGE
    {
        enum
        {
            PURE,
            LEGAL,
            SMART,
            NUM_LEVELS
        };

        KNOWLEDGE();

        int RolloutLevel;
        int TreeLevel;
        int SmartTreeCount;
        double SmartTreeValue;

        int Level(int phase) const
        {
            assert(phase < STATUS::NUM_PHASES);
            if (phase == STATUS::TREE)
                return TreeLevel;
            else
                return RolloutLevel;
        }

        // state-variable relationship knowledge
        // 0    no knowledge about state-var relationships
        // 1    MRF available
        // 2    Oracle
        // 3    Adaptive MRF
        int relKnowLevel;

        // State of the real environment
        STATE* realState;
    };

    struct STATUS
    {
        STATUS();

        enum
        {
            TREE,
            ROLLOUT,
            NUM_PHASES
        };

        enum
        {
            CONSISTENT,
            INCONSISTENT,
            RESAMPLED,
            OUT_OF_PARTICLES
        };

        int Phase;
        int Particles;
    };

    SIMULATOR();
    SIMULATOR(int numActions, int numObservations, double discount = 1.0);
    virtual ~SIMULATOR();

    // Create start start state (can be stochastic)
    virtual STATE* CreateStartState() const = 0;
    
    virtual void UpdateConfig(std::vector<int> &vals) const = 0;

    // Free memory for state
    virtual void FreeState(STATE* state) const = 0;

    // Update state according to action, and get observation and reward.
    // Return value of true indicates termination of episode (if episodic)
    virtual bool Step(STATE& state, int action,
        int& observation, double& reward, const BELIEF_STATE &beliefState) const = 0;

    // Create new state and copy argument (must be same type)
    virtual STATE* Copy(const STATE& state) const = 0;

    // Sanity check
    virtual void Validate(const STATE& state) const;

    // Modify state stochastically to some related state
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status) const;

    // Use domain knowledge to assign prior value and confidence to actions
    // Should only use fully observable state variables
    void Prior(const STATE* state, const HISTORY& history, VNODE* vnode,
        const STATUS& status) const;

    // Use domain knowledge to select actions stochastically during rollouts
    // Should only use fully observable state variables
    int SelectRandom(const STATE& state, const HISTORY& history,
        const STATUS& status) const;

    // Generate set of legal actions
    virtual void GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& actions, const STATUS& status) const;

    // Generate set of preferred actions
    virtual void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& actions, const STATUS& status) const;

    // For explicit POMDP computation only
    virtual bool HasAlpha() const;
    virtual void AlphaValue(const QNODE& qnode, double& q, int& n) const;
    virtual void UpdateAlpha(QNODE& qnode, const STATE& state) const;

    // Textual display
    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayReward(double reward, std::ostream& ostr) const;

    virtual void GetBeliefDistribution(const BELIEF_STATE &beliefState, std::vector<std::pair<int, int>> &beliefs) const;
    virtual void Id2State(const int id, std::vector<int> &state) const;

    int GetRelKnowLevel() const;
    virtual void DisplayBeliefDistribution(const BELIEF_STATE& beliefState, std::ostream& ostr) const;
    virtual STATE* CreateStartState(std::vector<STATE*>* allParticles,
        std::vector<double> allParticleCumProbs, int id=-1) const;
    
    virtual std::vector<double*>* CreateStateRelKnowledge(const STATE& state, int nConnComp, double relProbab) const;
    // // Modify state stochastically to some related state
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObs, const STATUS& status, std::vector<double*>* stateVarRelationships) const;

    virtual void PropagateChange(STATE& state, int changedVariableIndex, std::vector<double*>* stateVarRelationships,
        std::vector<int>* alreadyExploredVarIndices) const;
    virtual void PropagateChangeToConnectedComponent(STATE& state, int changedVariableIndex, int newVal, std::vector<double*>* stateVarRelationships) const;
    virtual std::vector<double*> FindUnexploredAdjacentVariableIndices(int currentVarIndex,
        std::vector<double*>* stateVarRelationships, std::vector<int>* alreadyExploredVarIndices) const;
    virtual double ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const;
    virtual void ResetAgent() const;

    virtual void writeHistory(std::string what, bool append = true) const;
    // ------------------------------- ADDED -------------------------------------------

    // Accessors

    void SetKnowledge(const KNOWLEDGE& knowledge) { Knowledge = knowledge; }
    int GetNumActions() const { return NumActions; }
    int GetNumObservations() const { return NumObservations; }
    bool IsEpisodic() const { return false; }
    double GetDiscount() const { return Discount; }
    double GetRewardRange() const { return RewardRange; }
    double GetHorizon(double accuracy, int undiscountedHorizon = 100) const;

    bool isReal, online;
    UTILS::RND *RND;

protected:

    int NumActions, NumObservations;
    double Discount, RewardRange;
    KNOWLEDGE Knowledge;
};

#endif // SIMULATOR_H
