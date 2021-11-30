#include "../include/mcts.h"
#include <math.h>

#include <algorithm>

using namespace std;
using namespace UTILS;

//-----------------------------------------------------------------------------

MCTS::PARAMS::PARAMS()
:   Verbose(0),
    MaxDepth(100),
    NumSimulations(1000),
    NumStartStates(1000),
    UseTransforms(true),
    NumTransforms(0),
    MaxAttempts(0),
    ExpandCount(1),
    ExplorationConstant(1),
    UseRave(false),
    RaveDiscount(1.0),
    RaveConstant(0.01),
    DisableTree(false)
{
}

MCTS::MCTS(const SIMULATOR& simulator, const PARAMS& params)
:   Simulator(simulator),
    Params(params),
    TreeDepth(0)
{
    // std::cout << "Initializing std MCTS" << std::endl;
    VNODE::NumChildren = Simulator.GetNumActions();
    QNODE::NumChildren = Simulator.GetNumObservations();

    Root = ExpandNode(Simulator.CreateStartState());

    for (int i = 0; i < Params.NumStartStates; i++)
        Root->Beliefs().AddSample(Simulator.CreateStartState());
}

MCTS::~MCTS()
{
    VNODE::Free(Root, Simulator);
    VNODE::FreeAll();
}

// MRF
// Knowledge about state-variable relationship is available.
// The real state is used to generate the simulator states

MCTS::MCTS(const SIMULATOR& simulator, std::vector<double*>* stateVarRelationships, int nAllParticles, const PARAMS& params) // Generates an MCTS with the root VNODE with children QNODEs initialized to knowledge and a belief of numStartStates
:   Simulator(simulator),   // Gets the simulator
    Params(params),         // Gets the parameters
    TreeDepth(0)            // Sets tree depth=0
{
    // std::cout << "Initializing MCTS with MRF" << std::endl;
    VNODE::NumChildren = Simulator.GetNumActions();         // Set number of actions
    QNODE::NumChildren = Simulator.GetNumObservations();    // Set number of obs

    std::vector<STATE*>* allParticles = new std::vector<STATE*>(nAllParticles);
    std::vector<double> allParticleProb(nAllParticles);
    STATE* particle;
    double prob;

    for (int i = 0; i < nAllParticles; i++)
    {
        particle = Simulator.CreateStartState(); // Creates a random particle
        (*allParticles)[i] = particle; // Add particle to allParticles
        prob = Simulator.ComputeParticleProbability(*particle, stateVarRelationships);
        allParticleProb[i] = prob;
    }

    // Compute cumulative probability
    double sumAllParticleProb = 0.0;
    for(int i = 0; i < nAllParticles; i++)
    {
        sumAllParticleProb += allParticleProb[i];
    }

    std::vector<double> allParticleCumProbs(nAllParticles + 1);
    allParticleCumProbs[0] = 0.0;

    for(unsigned long i = 0; i < allParticleProb.size(); i++)
    {
        allParticleCumProbs[i+1] = allParticleCumProbs[i] + allParticleProb[i] / sumAllParticleProb;
    }

    Root = ExpandNode(Simulator.CreateStartState(allParticles, allParticleCumProbs));

    // End its (action) children QNODES and puts knowledge in them

    for (int i = 0; i < Params.NumStartStates; i++)
    {
        // Add states samples to root beliefs allParticles,allParticleProb
        Root->Beliefs().AddSample(Simulator.CreateStartState(allParticles, allParticleCumProbs));
    }
}

bool MCTS::Update(int action, int observation, double reward)
{
    History.Add(action, observation);
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
    QNODE& qnode = Root->Child(action);
    VNODE* vnode = qnode.Child(observation);
    if (vnode)
    {
        if (Params.Verbose >= 1)
            cout << "Matched " << vnode->Beliefs().GetNumSamples() << " states" << endl;
        beliefs.Copy(vnode->Beliefs(), Simulator);
    }
    else
    {
        if (Params.Verbose >= 1)
            cout << "No matching node found" << endl;
    }

    // Generate transformed states to avoid particle deprivation
    if (Params.UseTransforms)
        AddTransforms(Root, beliefs);

    // If we still have no particles, fail
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

    if (Params.Verbose >= 1)
        Simulator.DisplayBeliefs(beliefs, cout);

    // Find a state to initialise prior (only requires fully observed state)
    const STATE* state = 0;
    if (vnode && !vnode->Beliefs().Empty())
        state = vnode->Beliefs().GetSample(0);
    else
        state = beliefs.GetSample(0);

    // Delete old tree and create new root
    VNODE::Free(Root, Simulator);
    VNODE* newRoot = ExpandNode(state);
    newRoot->Beliefs() = beliefs;
    Root = newRoot;
    return true;
}

// Updates the tree removing the current root, moving it to the node hao. Updates the history.
bool MCTS::Update(int action, int observation, double reward, std::vector<double*>* stateVarRelationships)
{
    if (Params.Verbose >= 1)
        std::cout << "In MCTS::Update " << endl;
    int nRels = stateVarRelationships->size();
    History.Add(action, observation);   // Update history with action performed and observation got from the real environment
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
    QNODE& qnode = Root->Child(action);         // action QNODE
    VNODE* vnode = qnode.Child(observation);    // history VNODE
    if (vnode)  // History node (VNODE) already in tree
    {
        if (Params.Verbose >= 1)
            std::cout << "Matched " << vnode->Beliefs().GetNumSamples() << " states" << endl;
        beliefs.Copy(vnode->Beliefs(), Simulator);  // Copy the belief of vnode (containing states from simulations) into variable beliefs
    }
    else    // History node (VNODE) not in tree
    {
        if (Params.Verbose >= 1)
            std::cout << "No matching node found" << endl;
    }

    // Generate transformed states (distance 1) from the states in the root to avoid particle deprivation
    if (Params.UseTransforms)
        AddTransforms(Root, beliefs, stateVarRelationships);

    // If we still have no particles, fail
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

    if (Params.Verbose >= 2)
        Simulator.DisplayBeliefs(beliefs, cout);

    // Find a state to initialise prior (only requires fully observed state)
    const STATE* state = 0;
    if (vnode && !vnode->Beliefs().Empty()) // History node (VNODE) already in tree and has belief states
        state = vnode->Beliefs().GetSample(0);  // Take the first state
    else
        state = beliefs.GetSample(0);   // Take the first from beliefs (which are copies of beliefs of vnode plus transforms and surely contains some belief)

    // Delete old tree and create new root
    VNODE::Free(Root, Simulator);
    VNODE* newRoot = ExpandNode(state); // Why doesn't it keeps vnode, which already contains a piece of learned tree???
                                        // NOTICE: only the belief of the new node is kept, not the values and counts of the old simulations
    newRoot->Beliefs() = beliefs;
    Root = newRoot;
    return true;
}

int MCTS::SelectAction()
{
    if (Params.DisableTree)
    {
        RolloutSearch();
    }
    else
    {
        UCTSearch();
    }

    return GreedyUCB(Root, false);
}

void MCTS::RolloutSearch()
{
	std::vector<double> totals(Simulator.GetNumActions(), 0.0);
	int historyDepth = History.Size();
	std::vector<int> legal;
	assert(BeliefState().GetNumSamples() > 0);
	Simulator.GenerateLegal(*BeliefState().GetSample(0), GetHistory(), legal, GetStatus());
	random_shuffle(legal.begin(), legal.end());

	for (int i = 0; i < Params.NumSimulations; i++)
	{
		int action = legal[i % legal.size()];
		STATE* state = Root->Beliefs().CreateSample(Simulator);
		Simulator.Validate(*state);

		int observation;
		double immediateReward, delayedReward, totalReward;


        const BELIEF_STATE& doingNothing = BELIEF_STATE();
		bool terminal = Simulator.Step(*state, action, observation, immediateReward, doingNothing);

		VNODE*& vnode = Root->Child(action).Child(observation);
		if (!vnode && !terminal)
		{
			vnode = ExpandNode(state);
			AddSample(vnode, *state);
		}
		History.Add(action, observation);

		delayedReward = Rollout(*state);
		totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
		Root->Child(action).Value.Add(totalReward);

		Simulator.FreeState(state);
		History.Truncate(historyDepth);
	}
}

void MCTS::UCTSearch()
{
    ClearStatistics();

    int historyDepth = History.Size();

    for (int n = 0; n < Params.NumSimulations; n++)
    {
        STATE* state = Root->Beliefs().CreateSample(Simulator);

        Simulator.Validate(*state);
        Status.Phase = SIMULATOR::STATUS::TREE;
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation" << endl;
            Simulator.DisplayState(*state, cout);
        }

        TreeDepth = 0;
        PeakTreeDepth = 0;
        double totalReward = SimulateV(*state, Root);
        StatTotalReward.Add(totalReward);
        StatTreeDepth.Add(PeakTreeDepth);

        if (Params.Verbose >= 2)
            cout << "Total reward = " << totalReward << endl;
        if (Params.Verbose >= 3)
            DisplayValue(4, cout);

        Simulator.FreeState(state);
        History.Truncate(historyDepth);
    }

    DisplayStatistics(cout);
}

double MCTS::SimulateV(STATE& state, VNODE* vnode)
{
    int action = GreedyUCB(vnode, true);

    PeakTreeDepth = TreeDepth;
    if (TreeDepth >= Params.MaxDepth) // search horizon reached
        return 0;

    if (TreeDepth == 1)
        AddSample(vnode, state);

    QNODE& qnode = vnode->Child(action);
    double totalReward = SimulateQ(state, qnode, action);
    vnode->Value.Add(totalReward);
    AddRave(vnode, totalReward);
    return totalReward;
}

double MCTS::SimulateQ(STATE& state, QNODE& qnode, int action)
{
    int observation;
    double immediateReward, delayedReward = 0;

    if (Simulator.HasAlpha())
        Simulator.UpdateAlpha(qnode, state);

    // Added just as a temporary fix
    const BELIEF_STATE& beliefsTmp = BELIEF_STATE();
    bool terminal = Simulator.Step(state, action, observation, immediateReward, beliefsTmp);
    assert(observation >= 0 && observation < Simulator.GetNumObservations());
    History.Add(action, observation);

    if (Params.Verbose >= 3)
    {
        Simulator.DisplayAction(action, cout);
        Simulator.DisplayObservation(state, observation, cout);
        Simulator.DisplayReward(immediateReward, cout);
        Simulator.DisplayState(state, cout);
    }

    VNODE*& vnode = qnode.Child(observation);
    if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount)
        vnode = ExpandNode(&state);

    if (!terminal)
    {
        TreeDepth++;
        if (vnode)
            delayedReward = SimulateV(state, vnode);
        else
            delayedReward = Rollout(state);
        TreeDepth--;
    }

    double totalReward = immediateReward + Simulator.GetDiscount() * delayedReward;
    qnode.Value.Add(totalReward);
    return totalReward;
}

void MCTS::AddRave(VNODE* vnode, double totalReward)
{
    double totalDiscount = 1.0;
    for (int t = TreeDepth; t < History.Size(); ++t)
    {
        QNODE& qnode = vnode->Child(History[t].Action);
        qnode.AMAF.Add(totalReward, totalDiscount);
        totalDiscount *= Params.RaveDiscount;
    }
}

VNODE* MCTS::ExpandNode(const STATE* state)
{
    VNODE* vnode = VNODE::Create();
    vnode->Value.Set(0, 0);
    Simulator.Prior(state, History, vnode, Status);

    if (Params.Verbose >= 2)
    {
        cout << "Expanding node: ";
        History.Display(cout);
        cout << endl;
    }

    return vnode;
}

void MCTS::AddSample(VNODE* node, const STATE& state)
{
    STATE* sample = Simulator.Copy(state);
    node->Beliefs().AddSample(sample);
    if (Params.Verbose >= 2)
    {
        cout << "Adding sample:" << endl;
        Simulator.DisplayState(*sample, cout);
    }
}

int MCTS::GreedyUCB(VNODE* vnode, bool ucb) const
{
    static vector<int> besta;
    besta.clear();
    double bestq = -pInfinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);
    bool hasalpha = Simulator.HasAlpha();

    for (int action = 0; action < Simulator.GetNumActions(); action++)
    {
        double q, alphaq;
        int n, alphan;

        QNODE& qnode = vnode->Child(action);
        q = qnode.Value.GetValue();
        n = qnode.Value.GetCount();

        if (Params.UseRave && qnode.AMAF.GetCount() > 0)
        {
            double n2 = qnode.AMAF.GetCount();
            double beta = n2 / (n + n2 + Params.RaveConstant * n * n2);
            q = (1.0 - beta) * q + beta * qnode.AMAF.GetValue();
        }

        if (hasalpha && n > 0)
        {
            Simulator.AlphaValue(qnode, alphaq, alphan);
            q = (n * q + alphan * alphaq) / (n + alphan);
            //cout << "N = " << n << ", alphaN = " << alphan << endl;
            //cout << "Q = " << q << ", alphaQ = " << alphaq << endl;
        }

        if (ucb)
            q += FastUCB(N, n, logN);

        if (q >= bestq)
        {
            if (q > bestq)
                besta.clear();
            bestq = q;
            besta.push_back(action);
        }
    }

    assert(!besta.empty());

    #ifdef RNDGENINTERFACE
        return besta[Simulator.RND->Random(besta.size())];
    #else
        return besta[Random(besta.size())];
    #endif
}

double MCTS::Rollout(STATE& state)
{
    Status.Phase = SIMULATOR::STATUS::ROLLOUT;
    if (Params.Verbose >= 3)
        cout << "Starting rollout" << endl;

    double totalReward = 0.0;
    double discount = 1.0;
    bool terminal = false;
    int numSteps;
    for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
    {
        int observation;
        double reward;

        int action = Simulator.SelectRandom(state, History, Status);

        // Added just as a temporary fix
        const BELIEF_STATE& beliefsTmp=BELIEF_STATE();
        terminal = Simulator.Step(state, action, observation, reward, beliefsTmp);
        History.Add(action, observation);

        if (Params.Verbose >= 4)
        {
            Simulator.DisplayAction(action, cout);
            Simulator.DisplayObservation(state, observation, cout);
            Simulator.DisplayReward(reward, cout);
            Simulator.DisplayState(state, cout);
        }

        totalReward += reward * discount;
        discount *= Simulator.GetDiscount();
    }

    StatRolloutDepth.Add(numSteps);
    if (Params.Verbose >= 3)
        cout << "Ending rollout after " << numSteps
            << " steps, with total reward " << totalReward << endl;
    return totalReward;
}

void MCTS::AddTransforms(VNODE* root, BELIEF_STATE& beliefs)
{
    int attempts = 0, added = 0;

    // Local transformations of state that are consistent with history
    while (added < Params.NumTransforms && attempts < Params.MaxAttempts)
    {
        STATE* transform = CreateTransform();
        if (transform)
        {
            beliefs.AddSample(transform);
            added++;
        }
        attempts++;
    }

    if (Params.Verbose >= 1)
    {
        cout << "Created " << added << " local transformations out of "
            << attempts << " attempts" << endl;
    }
}

void MCTS::AddTransforms(VNODE* root, BELIEF_STATE& beliefs, std::vector<double*>* stateVarRelationships)
{
    if (Params.Verbose >= 2){
        std::cout << "In MCTS::AddTransforms (ACA)" << endl;
        std::cout << "Params.NumTransforms: " << Params.NumTransforms << "Params.MaxAttempts: " << Params.MaxAttempts << endl;
    }
    int attempts = 0, added = 0;

    // Local transformations of state that are consistent with history
    while (added < Params.NumTransforms && attempts < Params.MaxAttempts)
    {
        //std::cout << "added" << added <<  "attempts" << attempts << endl;
        STATE* transform = CreateTransform(stateVarRelationships);
        if (transform)  // If it is a good transform
        {
            beliefs.AddSample(transform);   // Add the new state to the belief state
            added++;
        }
        attempts++;
    }

    if (Params.Verbose >= 1)
    {
        std::cout << "Created " << added << " local transformations out of "
            << attempts << " attempts" << endl;
    }
}

STATE* MCTS::CreateTransform() const
{
    int stepObs;
    double stepReward;

    STATE* state = Root->Beliefs().CreateSample(Simulator);

    // Added just as a temporary fix
    const BELIEF_STATE& beliefsTmp=BELIEF_STATE();

    Simulator.Step(*state, History.Back().Action, stepObs, stepReward, beliefsTmp);
    if (Simulator.LocalMove(*state, History, stepObs, Status))
        return state;
    Simulator.FreeState(state);
    return 0;
}

// Gets a state from the belief state of the Root (previews node?).
// Apply to it a local transformation (e.g., a single rock value is changed)
STATE* MCTS::CreateTransform(std::vector<double*>* stateVarRelationships) const
{
    //if (Params.Verbose >= 2)
    //    std::cout << "In MCTS::CreateTransform (ACA)" << endl;
    int stepObs;
    double stepReward;

    STATE* state = Root->Beliefs().CreateSample(Simulator); // Get a random state from the root belief state
    const BELIEF_STATE& beliefsTmp=BELIEF_STATE();
    Simulator.Step(*state, History.Back().Action, stepObs, stepReward, beliefsTmp); // Re-Simulate the last action, get the new state (consistent with the last action), obs and reward

    if (Params.Verbose >= 2)
    {
        std::cout << "Performing local move to state " << endl;
        Simulator.DisplayState(*state,cout);
    }

    // Apply a local change to state (it works with a certain probability related to
    // the fact of obtaining an observation equal to that in the history)
    if (Simulator.LocalMove(*state, History, stepObs, Status, stateVarRelationships))
        return state;

    // Transformation not obtained
    Simulator.FreeState(state);
    return 0;
}

double MCTS::UCB[UCB_N][UCB_n];
bool MCTS::InitialisedFastUCB = true;

void MCTS::InitFastUCB(double exploration)
{
    // cout << "Initialising fast UCB table... ";
    for (int N = 0; N < UCB_N; ++N)
        for (int n = 0; n < UCB_n; ++n)
            if (n == 0)
                UCB[N][n] = pInfinity;
            else
                UCB[N][n] = exploration * sqrt(log(N + 1) / n);
    // cout << "done" << endl;
    InitialisedFastUCB = true;
}

inline double MCTS::FastUCB(int N, int n, double logN) const
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
        return UCB[N][n];

    if (n == 0)
        return pInfinity;
    else
        return Params.ExplorationConstant * sqrt(logN / n);
}

void MCTS::ClearStatistics()
{
    StatTreeDepth.Clear();
    StatRolloutDepth.Clear();
    StatTotalReward.Clear();
}

void MCTS::DisplayStatistics(ostream& ostr) const
{
    if (Params.Verbose >= 1)
    {
        StatTreeDepth.Print("Tree depth", ostr);
        StatRolloutDepth.Print("Rollout depth", ostr);
        StatTotalReward.Print("Total reward", ostr);
    }

    if (Params.Verbose >= 2)
    {
        ostr << "Policy after " << Params.NumSimulations << " simulations" << endl;
        DisplayPolicy(6, ostr);
        ostr << "Values after " << Params.NumSimulations << " simulations" << endl;
        DisplayValue(6, ostr);
    }
}

void MCTS::DisplayValue(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Values:" << endl;
    Root->DisplayValue(history, depth, ostr);
}

void MCTS::DisplayPolicy(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Policy:" << endl;
    Root->DisplayPolicy(history, depth, ostr);
}