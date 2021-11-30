#include "../include/ros_rocksample.h"
#include "../include/utils.h"

// Is there the need to change hardcoded values here?
ROS_ROCKSAMPLE::ROS_ROCKSAMPLE(int size, int rocks, ros::NodeHandle &nh, UTILS::RND &rnd, bool isReal_, bool online_, bool learnMRF_)
    :   Grid(size, size),
        Size(size),
        NumRocks(rocks),
        SmartMoveProb(0.95),
        UncertaintyCount(0),
        nh_(nh),
        private_nh_("~")
{
    NumActions = NumRocks + 5;
    NumObservations = 3;
    RewardRange = 20;
    Discount = 0.95;
    isReal = isReal_;
    online = online_;
    learnMRF = learnMRF_;
    
    RND = &rnd;
    isRockValuable = new int[NumRocks];
    
    nh_.getParam("square_size", squareSize);
    nh_.getParam("agent_goal_topic", agentGoalTopic);
    nh_.getParam("agent_fb_topic", agentFeedbackTopic);
    nh_.getParam("env_info_topic", envInfoTopic);
    nh_.getParam("plan2env_obs_topic", askObservationTopic);
    nh_.getParam("env2plan_obs_topic", getObservationTopic);

    private_nh_.getParam("reward_exit_grid", RewardExitEast);
    private_nh_.getParam("reward_check", RewardCheck);
    private_nh_.getParam("reward_move", RewardMove);
    private_nh_.getParam("history_outputfile", historyOutputFile);

    std::string startState = "";
    Grid.SetAllValues(-1);

    if (learnMRF && !online)
    {
        HalfEfficiencyDistance = 10.0;
        StartPos.X = 0;
        StartPos.Y = 0;

        std::cout << NumRocks << std::endl;
        COORD rocks[NumRocks];

        // Change configuration here when not learning from env node
        rocks[0] = COORD(0, 4);
        rocks[1] = COORD(2, 4);
        rocks[2] = COORD(1, 1);
        rocks[3] = COORD(1, 0);
        rocks[4] = COORD(1, 4);
        rocks[5] = COORD(0, 3);
        rocks[6] = COORD(2, 1);
        rocks[7] = COORD(3, 2);

        for (size_t i = 0; i < NumRocks; i++)
        {
            Grid(rocks[i]) = i;
            RockPos.push_back(rocks[i]);
        }
    }
    else if (learnMRF && online)
    {
        pubAgentGoal = nh_.advertise<geometry_msgs::Point>(agentGoalTopic, 1, true);
        pubAskForObservation = nh_.advertise<std_msgs::Float32MultiArray>(askObservationTopic, 1);
        std_msgs::Float32MultiArray::ConstPtr envInfo = ros::topic::waitForMessage<std_msgs::Float32MultiArray>(envInfoTopic, nh_);
        HalfEfficiencyDistance = envInfo->data[0];
        StartPos.X = envInfo->data[1];
        StartPos.Y = envInfo->data[2];
    
        for (size_t i = 0; i < NumRocks; i++)
        {

            int rockGridX = envInfo->data[3 + i * 3];
            int rockGridY = envInfo->data[4 + i * 3];

            COORD pos(rockGridX, rockGridY);
            std::cout << pos << std::endl;
            Grid(pos) = i;
            RockPos.push_back(pos);
            // isRockValuable[i] = rockVal;
        }
    }
    else
    {
        pubAgentGoal = nh_.advertise<geometry_msgs::Point>(agentGoalTopic, 1, true);
        pubAskForObservation = nh_.advertise<std_msgs::Float32MultiArray>(askObservationTopic, 1);
        std_msgs::Float32MultiArray::ConstPtr envInfo = ros::topic::waitForMessage<std_msgs::Float32MultiArray>(envInfoTopic, nh_);
        HalfEfficiencyDistance = envInfo->data[0];
        StartPos.X = envInfo->data[1];
        StartPos.Y = envInfo->data[2];

        for (size_t i = 0; i < NumRocks; i++)
        {

            int rockGridX = envInfo->data[3 + i * 3];
            int rockGridY = envInfo->data[4 + i * 3];
            bool rockVal;

            if (envInfo->data[5 + i * 3])
            {
                rockVal = true;
            }
            else
            {
                rockVal = false;
            }

            COORD pos(rockGridX, rockGridY);
            Grid(pos) = i;
            RockPos.push_back(pos);
            isRockValuable[i] = rockVal;

            startState += std::to_string(rockGridX) + " " +  std::to_string(rockGridY) + " " + std::to_string(envInfo->data[5 + i * 3]) + "\n";
        }

        startState += std::to_string(size) + " " + std::to_string(NumRocks) + " ";
        startState += std::to_string(StartPos.X) + " " +  std::to_string(StartPos.Y) + "\n";

        #ifdef ONLINEDEBUG
        writeHistory(startState, false);
        #endif
    }
}

void ROS_ROCKSAMPLE::writeHistory(std::string what, bool append) const
{
    std::ofstream historyFile;

    if (append)
    {
        historyFile.open(historyOutputFile, std::ios::out | std::ios::app);
    }
    else
    {
        historyFile.open(historyOutputFile, std::ios::out | std::ios::trunc);
    }
    historyFile << what << std::endl;
    historyFile.close();
}

void ROS_ROCKSAMPLE::UpdateConfig(std::vector<int> &vals) const
{
    for (int i = 0; i < NumRocks; i++)
    {
        isRockValuable[i] = vals[i];
    }
}

STATE* ROS_ROCKSAMPLE::CreateStartState() const
{
    ROS_ROCKSAMPLE_STATE* rockstate = MemoryPool.Allocate();
    rockstate->AgentPos = StartPos;
    rockstate->Rocks.clear();

    for (int i = 0; i < NumRocks; i++)
    {
        ROS_ROCKSAMPLE_STATE::ENTRY entry;
        entry.Collected = false;

        if (isReal)
        {
            entry.Valuable = isRockValuable[i];
        }
        else
        {
            #ifdef RNDGENINTERFACE
                entry.Valuable = RND->Bernoulli(0.5);
            #else
                entry.Valuable = UTILS::Bernoulli(0.5);
            #endif
        }

        entry.Count = 0;
        entry.Measured = 0;
        entry.ProbValuable = 0.5;
        entry.LikelihoodValuable = 1.0;
        entry.LikelihoodWorthless = 1.0;
        rockstate->Rocks.push_back(entry);
    }
    rockstate->Target = SelectTarget(*rockstate);
    return rockstate;
}

bool ROS_ROCKSAMPLE::Step(STATE& state, int action, int& observation, double& reward, const BELIEF_STATE& beliefState) const
{
    ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<ROS_ROCKSAMPLE_STATE&>(state);
    reward = 0;
    observation = E_NONE;

    bool ok = false;
    bool outFromGridEast = false;
    int rock;

    if (action < E_SAMPLE) // move
    {
        switch (action)
        {
            case COORD::E_EAST:
                if (rockstate.AgentPos.X + 1 < Size)
                {
                    rockstate.AgentPos.X++;
                    ok = true;

                    reward = RewardMove;
                }
                else
                {
                    #ifdef ALWAYS_INSIDE_GRID
                        reward = -100;
                    #else
                        reward = RewardExitEast;
                        outFromGridEast = true;

                        if (isReal && online)
                        {
                            rockstate.AgentPos.X++;
                            ok = true;
                        }
                    #endif
                }
                break;

            case COORD::E_NORTH:
                if (rockstate.AgentPos.Y + 1 < Size)
                {
                    rockstate.AgentPos.Y++;
                    ok = true;
                    reward = RewardMove;
                }
                else
                {
                    reward = -100;
                }
                break;

            case COORD::E_SOUTH:
                if (rockstate.AgentPos.Y - 1 >= 0)
                {
                    rockstate.AgentPos.Y--;
                    ok = true;
                    reward = RewardMove;
                }
                else
                {
                    reward = -100;
                }
                break;

            case COORD::E_WEST:
                if (rockstate.AgentPos.X - 1 >= 0)
                {
                    rockstate.AgentPos.X--;
                    ok = true;
                    reward = RewardMove;
                }
                else
                {
                    reward = -100;
                }
                break;
        }

        if (isReal && ok && online)
        {
            geometry_msgs::Point actionMsg = geometry_msgs::Point();

            actionMsg.x = rockstate.AgentPos.X;
            actionMsg.y = rockstate.AgentPos.Y;

            pubAgentGoal.publish(actionMsg);
            ros::spinOnce();
            ros::Duration(0.5).sleep();
            // ROS_INFO("SENT MOVE TO REAL AGENT");
            // std::cout << "Current action: ";
            // switch (action)
            // {
            //     case COORD::E_EAST: std::cout << "MOVE EAST\n"; break;
            //     case COORD::E_NORTH: std::cout << "MOVE NORTH\n"; break;
            //     case COORD::E_SOUTH: std::cout << "MOVE SOUTH\n"; break;
            //     case COORD::E_WEST: std::cout << "MOVE WEST\n"; break;
            // }

            std_msgs::Int8::ConstPtr obs;
            const_cast<ROS_ROCKSAMPLE *>(this)->waitForMessage(agentFeedbackTopic, obs);
        }
    }

    // Send sample to ENV
    if (action == E_SAMPLE) // sample
    {
        rock = Grid(rockstate.AgentPos);
        if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        {
            rockstate.Rocks[rock].Collected = true;
            if (rockstate.Rocks[rock].Valuable)
            {
                reward = +10;
                // Trying to return observation after sample
                // observation = E_GOOD;
            }
            else
            {
                reward = -10;
                // trying to return observation after sample
                // observation = E_BAD;
            }
        }
        else
        {
            reward = -100;
        }
    }

    double distance, efficiency;
    float rDistance, rEfficiency;

    if (action > E_SAMPLE) // check
    {
        reward = RewardCheck;
        rock = action - E_SAMPLE - 1;
        assert(rock < NumRocks);
        observation = GetObservation(rockstate, rock, rDistance);
        rockstate.Rocks[rock].Measured++;

        distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]) * squareSize;
    	efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

        // This efficiency is computed with the real distance [Different only for real robot]
    	rEfficiency = (1 + pow(2, -rDistance / HalfEfficiencyDistance)) * 0.5;

        // Use "synthetic" efficiency to update probabilities
        // In this way, only the observations are affected by the errors of the real environment
        // A cleaner solution may be to use the real cartesian coordinates to compute the grid ones
        // In this way, there may be a discrepancy only when the agent is in a different cell of the grid
        if (observation == E_GOOD)
        {
            rockstate.Rocks[rock].Count++;
            // rockstate.Rocks[rock].LikelihoodValuable *= rEfficiency;
            // rockstate.Rocks[rock].LikelihoodWorthless *= 1.0 - rEfficiency;
            
            rockstate.Rocks[rock].LikelihoodValuable *= efficiency;
            rockstate.Rocks[rock].LikelihoodWorthless *= 1.0 - efficiency;
        }
        else
        {
            rockstate.Rocks[rock].Count--;
            // rockstate.Rocks[rock].LikelihoodWorthless *= rEfficiency;
            // rockstate.Rocks[rock].LikelihoodValuable *= 1.0 - rEfficiency;
            rockstate.Rocks[rock].LikelihoodWorthless *= efficiency;
            rockstate.Rocks[rock].LikelihoodValuable *= 1.0 - efficiency;
		}
		double denom = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) +
			(0.5 * rockstate.Rocks[rock].LikelihoodWorthless);

		rockstate.Rocks[rock].ProbValuable = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) / denom;
    }

    if (rockstate.Target < 0 || rockstate.AgentPos == RockPos[rockstate.Target])
        rockstate.Target = SelectTarget(rockstate);

    assert(reward != -100);

    if (isReal)
    {

        std::string what = "";

        // std::cout << "Action: \t";
        switch (action)
        {
            case COORD::E_EAST: 
                // std::cout << "MOVE EAST\n"; 
                what += "ME"; 
                break;
            case COORD::E_NORTH: 
                // std::cout << "MOVE NORTH\n";
                what += "MN";  
                break;
            case COORD::E_SOUTH: 
                // std::cout << "MOVE SOUTH\n";
                what += "MS";  
                break;
            case COORD::E_WEST: 
                // std::cout << "MOVE WEST\n";
                what += "MW";  
                break;
            case E_SAMPLE: 
                // std::cout << "SAMPLE\n";
                what += "S " + std::to_string(rock) ;  
                break;
            default: 
                // std::cout << "CHECK" << std::endl;
                what += "C " + std::to_string(rock) ;  
                break;
        }

        if (action > E_SAMPLE)
        {
            what += " " + std::to_string(observation) + " " + std::to_string(distance) + " " + std::to_string(rDistance) + " " + std::to_string(efficiency)  + " " + std::to_string(rEfficiency);
        }

        what += " " + std::to_string(reward);

        for(size_t i = 0; i < NumRocks; i++)
        {
            what += " " + std::to_string(rockstate.Rocks[i].ProbValuable);
        }

        std::vector<std::pair<int, int>> belDistribution;
        GetBeliefDistribution(beliefState, belDistribution);
        std::vector<int> state;

        Id2State(belDistribution[0].first, state);
        for(size_t i = 0; i < state.size(); i++)
        {
            what += " " + std::to_string(state[i]);
        }

        writeHistory(what);

        #ifndef ALWAYS_INSIDE_GRID
        if (outFromGridEast)
        {
            writeHistory("---");
        }
        #endif
    }

    return outFromGridEast;
}

void ROS_ROCKSAMPLE::waitForMessage(std::string topic, std_msgs::Int8::ConstPtr &msg)
{
    msg = ros::topic::waitForMessage<std_msgs::Int8>(topic, nh_);
}

void ROS_ROCKSAMPLE::waitForMessage(std::string topic, std_msgs::Float32MultiArray::ConstPtr &msg)
{
    msg = ros::topic::waitForMessage<std_msgs::Float32MultiArray>(topic, nh_);
}

int ROS_ROCKSAMPLE::GetObservation(const ROS_ROCKSAMPLE_STATE& rockstate, int rock, float &dist) const
{
    if (isReal && online)
    {
        std_msgs::Float32MultiArray msg = std_msgs::Float32MultiArray();
        msg.data.push_back(rockstate.AgentPos.X);
        msg.data.push_back(rockstate.AgentPos.Y);
        msg.data.push_back(rock);
        msg.data.push_back(RockPos[rock].X);
        msg.data.push_back(RockPos[rock].Y);

        pubAskForObservation.publish(msg);
        ros::spinOnce();

        std_msgs::Float32MultiArray::ConstPtr obs;
        // Now it's only using the distance from the real environment
        // the observation in obs->data[0] is not even considered
        const_cast<ROS_ROCKSAMPLE *>(this)->waitForMessage(getObservationTopic, obs);

        dist = obs->data[1];
        
        double efficiency = (1 + pow(2, -obs->data[1] / HalfEfficiencyDistance)) * 0.5;


        #ifdef RNDGENINTERFACE
            if (RND->Bernoulli(efficiency))
        #else
            if (UTILS::Bernoulli(efficiency))
        #endif
        {
            return rockstate.Rocks[rock].Valuable ? E_GOOD : E_BAD;
        }
        else
        {
            return rockstate.Rocks[rock].Valuable ? E_BAD : E_GOOD;
        }

        // return (int) obs->data[0];
    }
    else
    {
        double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]) * squareSize;
        double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

        dist = distance;

        
        #ifdef RNDGENINTERFACE
            if (RND->Bernoulli(efficiency))
        #else
            if (UTILS::Bernoulli(efficiency))
        #endif
        {
            return rockstate.Rocks[rock].Valuable ? E_GOOD : E_BAD;
        }
        else
        {
            return rockstate.Rocks[rock].Valuable ? E_BAD : E_GOOD;
        }
    }
}

STATE* ROS_ROCKSAMPLE::Copy(const STATE& state) const
{
    const ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<const ROS_ROCKSAMPLE_STATE&>(state);
    ROS_ROCKSAMPLE_STATE* newstate = MemoryPool.Allocate();
    *newstate = rockstate;
    return newstate;
}

void ROS_ROCKSAMPLE::Validate(const STATE& state) const
{
    const ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<const ROS_ROCKSAMPLE_STATE&>(state);
    assert(Grid.Inside(rockstate.AgentPos));
}

void ROS_ROCKSAMPLE::FreeState(STATE* state) const
{
    ROS_ROCKSAMPLE_STATE* rockstate = safe_cast<ROS_ROCKSAMPLE_STATE*>(state);
    MemoryPool.Free(rockstate);
}

bool ROS_ROCKSAMPLE::LocalMove(STATE& state, const HISTORY& history, int stepObs, const STATUS& status) const
{
    ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<ROS_ROCKSAMPLE_STATE&>(state);
    
    #ifdef RNDGENINTERFACE
        int rock = RND->Random(NumRocks);
    #else
        int rock = UTILS::Random(NumRocks);
    #endif

    rockstate.Rocks[rock].Valuable = !rockstate.Rocks[rock].Valuable;

    if (history.Back().Action > E_SAMPLE) // check rock
    {
        rock = history.Back().Action - E_SAMPLE - 1;
        int realObs = history.Back().Observation;

        // Condition new state on real observation
        float placeholder;
        int newObs = GetObservation(rockstate, rock, placeholder);
        if (newObs != realObs)
            return false;

        // Update counts to be consistent with real observation
        if (realObs == E_GOOD && stepObs == E_BAD)
            rockstate.Rocks[rock].Count += 2;
        if (realObs == E_BAD && stepObs == E_GOOD)
            rockstate.Rocks[rock].Count -= 2;
    }
    return true;
}

void ROS_ROCKSAMPLE::DisplayBeliefs(const BELIEF_STATE& beliefState,
    std::ostream& ostr) const
{
}

void ROS_ROCKSAMPLE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<const ROS_ROCKSAMPLE_STATE&>(state);
    ostr << std::endl;
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << std::endl;
    for (int y = Size - 1; y >= 0; y--)
    {
        ostr << "# ";
        for (int x = 0; x < Size; x++)
        {
            COORD pos(x, y);
            int rock = Grid(pos);
            const ROS_ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
            if (rockstate.AgentPos == COORD(x, y))
                ostr << "* ";
            else if (rock >= 0 && !entry.Collected)
                ostr << rock << (entry.Valuable ? "$" : "X");
            else
                ostr << ". ";
        }
        ostr << "#" << std::endl;
    }
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << std::endl;
}

void ROS_ROCKSAMPLE::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
    switch (observation)
    {
    case E_NONE:
        break;
    case E_GOOD:
        ostr << "Observed good" << std::endl;
        break;
    case E_BAD:
        ostr << "Observed bad" << std::endl;
        break;
    }
}

void ROS_ROCKSAMPLE::DisplayAction(int action, std::ostream& ostr) const
{
    if (action < E_SAMPLE)
        ostr << COORD::CompassString[action] << std::endl;
    if (action == E_SAMPLE)
        ostr << "Sample" << std::endl;
    if (action > E_SAMPLE)
        ostr << "Check " << action - E_SAMPLE << std::endl;
}

void ROS_ROCKSAMPLE::GenerateLegal(const STATE& state, const HISTORY& history,
    std::vector<int>& legal, const STATUS& status) const
{

    const ROS_ROCKSAMPLE_STATE& rockstate =
        safe_cast<const ROS_ROCKSAMPLE_STATE&>(state);

    if (rockstate.AgentPos.Y + 1 < Size)
        legal.push_back(COORD::E_NORTH);

    #ifdef ALWAYS_INSIDE_GRID
    if (rockstate.AgentPos.X + 1 < Size)
        legal.push_back(COORD::E_EAST);
    #else
    legal.push_back(COORD::E_EAST);
    #endif

    if (rockstate.AgentPos.Y - 1 >= 0)
        legal.push_back(COORD::E_SOUTH);

    if (rockstate.AgentPos.X - 1 >= 0)
        legal.push_back(COORD::E_WEST);

    int rock = Grid(rockstate.AgentPos);
    if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        legal.push_back(E_SAMPLE);

    for (rock = 0; rock < NumRocks; ++rock)
        if (!rockstate.Rocks[rock].Collected)
            legal.push_back(rock + 1 + E_SAMPLE);
}

void ROS_ROCKSAMPLE::GeneratePreferred(const STATE& state, const HISTORY& history,
    std::vector<int>& actions, const STATUS& status) const
{

	static const bool UseBlindPolicy = false;

	if (UseBlindPolicy)
	{
		actions.push_back(COORD::E_EAST);
		return;
	}

	const ROS_ROCKSAMPLE_STATE& rockstate =
	        safe_cast<const ROS_ROCKSAMPLE_STATE&>(state);

	// Sample rocks with more +ve than -ve observations
	int rock = Grid(rockstate.AgentPos);
	if (rock >= 0 && !rockstate.Rocks[rock].Collected)
	{
		int total = 0;
		for (int t = 0; t < history.Size(); ++t)
		{
			if (history[t].Action == rock + 1 + E_SAMPLE)
			{
				if (history[t].Observation == E_GOOD)
					total++;
				if (history[t].Observation == E_BAD)
					total--;
			}
		}
		if (total > 0)
		{
			actions.push_back(E_SAMPLE);
			return;
		}

	}

	// processes the rocks
	bool all_bad = true;
	bool north_interesting = false;
	bool south_interesting = false;
	bool west_interesting  = false;
	bool east_interesting  = false;

	for (int rock = 0; rock < NumRocks; ++rock)
	{
		const ROS_ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
		if (!entry.Collected)
		{
			int total = 0;
			for (int t = 0; t < history.Size(); ++t)
			{
				if (history[t].Action == rock + 1 + E_SAMPLE)
				{
					if (history[t].Observation == E_GOOD)
						total++;
					if (history[t].Observation == E_BAD)
						total--;
				}
			}

			if (total >= 0)
			{
				all_bad = false;

				if (RockPos[rock].Y > rockstate.AgentPos.Y)
					north_interesting = true;
				if (RockPos[rock].Y < rockstate.AgentPos.Y)
					south_interesting = true;
				if (RockPos[rock].X < rockstate.AgentPos.X)
					west_interesting = true;
				if (RockPos[rock].X > rockstate.AgentPos.X)
					east_interesting = true;
			}
		}
	}

	// if all remaining rocks seem bad, then head east
	if (all_bad)
	{
		actions.push_back(COORD::E_EAST);
		return;
	}

	// generate a random legal move, with the exceptions that:
	//   a) there is no point measuring a rock that is already collected
	//   b) there is no point measuring a rock too often
	//   c) there is no point measuring a rock which is clearly bad or good
	//   d) we never sample a rock (since we need to be sure)
	//   e) we never move in a direction that doesn't take us closer to
	//      either the edge of the map or an interesting rock
	if (rockstate.AgentPos.Y + 1 < Size && north_interesting)
			actions.push_back(COORD::E_NORTH);

	if (east_interesting)
		actions.push_back(COORD::E_EAST);

	if (rockstate.AgentPos.Y - 1 >= 0 && south_interesting)
		actions.push_back(COORD::E_SOUTH);

	if (rockstate.AgentPos.X - 1 >= 0 && west_interesting)
		actions.push_back(COORD::E_WEST);


	for (rock = 0; rock < NumRocks; ++rock)
	{
		if (!rockstate.Rocks[rock].Collected    &&
			rockstate.Rocks[rock].ProbValuable != 0.0 &&
			rockstate.Rocks[rock].ProbValuable != 1.0 &&
			rockstate.Rocks[rock].Measured < 5  &&
			std::abs(rockstate.Rocks[rock].Count) < 2)
		{
			actions.push_back(rock + 1 + E_SAMPLE);
		}
	}
}

int ROS_ROCKSAMPLE::SelectTarget(const ROS_ROCKSAMPLE_STATE& rockstate) const
{
    int bestDist = Size * 2;
    int bestRock = -1;

    for (int rock = 0; rock < NumRocks; ++rock)
    {
        if (!rockstate.Rocks[rock].Collected
            && rockstate.Rocks[rock].Count >= UncertaintyCount)
        {

            int dist = COORD::ManhattanDistance(rockstate.AgentPos, RockPos[rock]);

            if (dist < bestDist)
            {
                bestDist = dist;
                bestRock = rock;

            }
        }
    }

    // if (isReal)
    // {
    //     std::cout << "Target rock: " << bestRock << std::endl;
    //     std::cout << "Best dist: " << bestDist << std::endl;
    // }

    return bestRock;
}

// Generates probabilistic knowledge about relationships between state-variables
std::vector<double*>* ROS_ROCKSAMPLE::CreateStateRelKnowledge(const STATE& state, int nConnComp, double relProbab) const
{
    // std::cout << "In CreateStateRelKnowledge" << std::endl;
    const ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<const ROS_ROCKSAMPLE_STATE&>(state);
    // Vector of knowledge
    std::vector<double*>* stateVarRelationships = new std::vector<double*>();
    // Vector of all possible edges ()
    std::vector<double*>* allStateVarRelationships = new std::vector<double*>();

    for (int i = 0; i < rockstate.Rocks.size() - 1; i++)
    {
        for (int j = i + 1; j < rockstate.Rocks.size(); j++)
        {
            if(rockstate.Rocks[i].Valuable == rockstate.Rocks[j].Valuable){
                allStateVarRelationships->push_back(new double[3]{(double)i, (double)j, relProbab});
            }
        }
    }

    // Check how many connected components there are
    int nTrueConnComp = rockstate.Rocks.size(); // N edge in stateVarRelationships
    int edgeI = -1;
    std::vector<std::vector<int>*>* connComps;
    while(nTrueConnComp>nConnComp)
    {
        //cout << "nTrueConnComp: " << nTrueConnComp << endl;
        #ifdef RNDGENINTERFACE
            edgeI = RND->Random(allStateVarRelationships->size());// Get random edge
        #else
            edgeI = UTILS::Random(allStateVarRelationships->size());// Get random edge
        #endif

        //cout << "adding edge: " << edgeI << ": " << (*allStateVarRelationships)[edgeI][0] << "-" << (*allStateVarRelationships)[edgeI][1] << endl;

        // If the edge is not in the knowledge
        stateVarRelationships->push_back((*allStateVarRelationships)[edgeI]); // Add the edge to the knowledge

        allStateVarRelationships->erase(allStateVarRelationships->begin()+edgeI);

        // Check how many connected components there are
        connComps = UTILS::computeConnComp(stateVarRelationships,rockstate.Rocks.size());
        nTrueConnComp = connComps->size();
    }
    std::cout << "StateRelKnowledge computed." << std::endl;
    UTILS::printConnectedComponents(connComps);
    return stateVarRelationships;
}

double ROS_ROCKSAMPLE::ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const
{
    ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<ROS_ROCKSAMPLE_STATE&>(particle);  // State
    double probab = 1;
    int var0, var1, rock0Val, rock1Val;

    double potential00_or_11, potential01_or_10, potential;
    for(std::vector<double*>::iterator it = stateVarRelationships->begin() ; it != stateVarRelationships->end(); ++it){ // For each edge
        var0 = (*it)[0];
        var1 = (*it)[1];
        potential00_or_11 = (*it)[2] / 2;
        potential01_or_10 = (1.0-(*it)[2]) / 2;

        // Apply the constraint to the state and compute the related potential
        rock0Val = rockstate.Rocks[var0].Valuable;
        rock1Val = rockstate.Rocks[var1].Valuable;
        if((rock0Val == 0 && rock1Val == 0) || (rock0Val == 1 && rock1Val == 1))
        {
            // They are equal
            potential = potential00_or_11;
        }
        else
        {
            // They are different
            potential = potential01_or_10;
        }

        probab = probab * potential;
    }
    return probab;
}

STATE* ROS_ROCKSAMPLE::CreateStartState(std::vector<STATE*>* allParticles, std::vector<double> allParticleCumProbs, int id) const
{
    ROS_ROCKSAMPLE_STATE* rockstate = MemoryPool.Allocate();
    rockstate->AgentPos = StartPos;         // Set agent position
    rockstate->Rocks.clear();               // Set rocks in the state

    rockstate->id = id;

    // Select a particle from allParticles with probability from allParticleProb
    #ifdef RNDGENINTERFACE
        double rnd = RND->RandomDouble(0,0.9999999999);
    #else
        double rnd = UTILS::RandomDouble(0,0.9999999999);        
    #endif

    int partI = 0;
    double cumProbTmp = 0.0;

    while(cumProbTmp<=rnd)
    {
        partI++;
        cumProbTmp = allParticleCumProbs[partI];
    }
    partI--; // Found particle index

    const ROS_ROCKSAMPLE_STATE& rockparticle = safe_cast<const ROS_ROCKSAMPLE_STATE&>(*((*allParticles)[partI]));

    for (int i = 0; i < NumRocks; i++)
    {
        ROS_ROCKSAMPLE_STATE::ENTRY entry;
        entry.Collected = false;            // Init to not collected
        entry.Valuable = rockparticle.Rocks[i].Valuable;    // Put particle values in the rockstate
        entry.Count = 0;
        entry.Measured = 0;
        entry.ProbValuable = 0.5;
        entry.LikelihoodValuable = 1.0;
        entry.LikelihoodWorthless = 1.0;
        rockstate->Rocks.push_back(entry);  // Add rock to rockstate
    }
    rockstate->Target = SelectTarget(*rockstate);
    return rockstate;
}

// Make a local change in the state (i.e., it inverts the value of a rock) with a certain probability related to
// the probability to observe from the new state the observation really taken in the history
bool ROS_ROCKSAMPLE::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status, std::vector<double*>* stateVarRelationships) const
{

    //std::cout << "In ROS_ROCKSAMPLE::LocalMove (ACA)" << endl;
    ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<ROS_ROCKSAMPLE_STATE&>(state);  // State
    #ifdef RNDGENINTERFACE
        int rock = RND->Random(NumRocks);                                        // Take a random rock
    #else
        int rock = UTILS::Random(NumRocks);                                        // Take a random rock
    #endif

    rockstate.Rocks[rock].Valuable = !rockstate.Rocks[rock].Valuable;   // Change the value of one rock of rockstate
    //std::cout << "Changing rock " << rock <<". New value=" << rockstate.Rocks[rock].Valuable << endl;
    // Propagate the change to the rest of the state according to probabilistic knowledge
    std::vector<int> *alreadyExploredVarIndices = new std::vector<int>();
    alreadyExploredVarIndices->push_back(rock);

    PropagateChange(rockstate,rock,stateVarRelationships,alreadyExploredVarIndices);

    if (history.Back().Action > E_SAMPLE) // If the last action was a rock check
    {
        rock = history.Back().Action - E_SAMPLE - 1;    // Get the sampled rock
        int realObs = history.Back().Observation;       // Get the real observation got from the last action

        // Condition new state on real observation
        float placeholder;
        int newObs = GetObservation(rockstate, rock, placeholder);   // Get obs from rock in the new state
        if (newObs != realObs)                          // If the new observation is different from the real one -> false
            return false;

        // Update counts to be consistent with real observation
        if (realObs == E_GOOD && stepObs == E_BAD)
            rockstate.Rocks[rock].Count += 2;
        if (realObs == E_BAD && stepObs == E_GOOD)
            rockstate.Rocks[rock].Count -= 2;
    }
    return true;
}

// Notice: in case of prob=1 changes could be simply propagated in all the connected component
void ROS_ROCKSAMPLE::PropagateChange(STATE& state, int changedVariable,
        std::vector<double*>* stateVarRelationships,  std::vector<int>* alreadyExploredVarIndices) const
{
    ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<ROS_ROCKSAMPLE_STATE&>(state);  // State
    std::vector<double*> adjIs = FindUnexploredAdjacentVariableIndices(changedVariable, stateVarRelationships, alreadyExploredVarIndices);

    if(adjIs.empty()){
        return;     // Nothing new to change. End of the recursion
    }

    double rnd;
    bool varToBeChangedI;
    int varToBeChanged;

    // For each unvisited adjacent variable
    for(std::vector<double*>::iterator it = adjIs.begin() ; it != adjIs.end(); ++it)
    {
        if((int)((*it)[0])!=changedVariable)
            varToBeChangedI=0;
        else
            varToBeChangedI=1;
        varToBeChanged=(int)((*it)[varToBeChangedI]);

        // if the next node to be analyzed has been already modified in previews for cycles
        if((std::find((*alreadyExploredVarIndices).begin(), (*alreadyExploredVarIndices).end(), varToBeChanged) != (*alreadyExploredVarIndices).end())){
            continue;  // Then jump to the new node
        }
        #ifdef RNDGENINTERFACE
            rnd = RND->RandomDouble(0,1);
        #else
            rnd = UTILS::RandomDouble(0,1);
        #endif
        
        if(rnd<=((*it)[2])){    // Make the change
            rockstate.Rocks[varToBeChanged].Valuable = rockstate.Rocks[changedVariable].Valuable;   // With probab in [2] set adjacent equal to current node, otherwise leave it as it is
        }
        (*alreadyExploredVarIndices).push_back(varToBeChanged);
        PropagateChange(rockstate, varToBeChanged, stateVarRelationships, alreadyExploredVarIndices); // Recursive propagation
    }

}

// Notice: in case of prob=1 changes could be simply propagated in all the connected component
void ROS_ROCKSAMPLE::PropagateChangeToConnectedComponent(STATE& state, int changedVariable, int newVal,
        std::vector<double*>* stateVarRelationships) const
{
    ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<ROS_ROCKSAMPLE_STATE&>(state);  // State
    std::vector<std::vector<int>*>* connComps = UTILS::computeConnComp(stateVarRelationships, NumRocks);
    bool found = 0;
    for(std::vector<std::vector<int>*>::iterator it = connComps->begin() ; it != connComps->end(); ++it){ // it=component
        // Check if changedVariable is in connected component it
        for(std::vector<int>::iterator it1 = (*it)->begin() ; it1 != (*it)->end(); ++it1){  // it1=node
            if((*it1) == changedVariable)
            {
                found=1;
                break;
            }
        }
        if(found)
        {
            for(std::vector<int>::iterator it1 = (*it)->begin() ; it1 != (*it)->end(); ++it1)
            {
                // change values of all nodes in the connected component
                rockstate.Rocks[(*it1)].Valuable=newVal;
            }
        }
    }
}

std::vector<double*> ROS_ROCKSAMPLE::FindUnexploredAdjacentVariableIndices(int currentVarIndex,
        std::vector<double*>* stateVarRelationships, std::vector<int>* alreadyExploredVarIndices) const
{

    int nRels = stateVarRelationships->size();
    std::vector<double*> adjIs;

    for(int i=0; i<nRels; i++)
    {
        if((int)(*stateVarRelationships)[i][0]==currentVarIndex)   // Current node in first index
            if(std::find((*alreadyExploredVarIndices).begin(), (*alreadyExploredVarIndices).end(), (int)(*stateVarRelationships)[i][1]) == (*alreadyExploredVarIndices).end()) // If second node not already explored
                adjIs.push_back((*stateVarRelationships)[i]); // Add stateVarRelationships entry to adjIs

        if((int)(*stateVarRelationships)[i][1]==currentVarIndex)   // Current node in first index
            if(std::find((*alreadyExploredVarIndices).begin(), (*alreadyExploredVarIndices).end(), (int)(*stateVarRelationships)[i][0]) == (*alreadyExploredVarIndices).end()) // If second node not already explored
                adjIs.push_back((*stateVarRelationships)[i]); // Add stateVarRelationships entry to adjIs

    }
    return adjIs;
}

void ROS_ROCKSAMPLE::DisplayBeliefDistribution(const BELIEF_STATE& beliefState, std::ostream& ostr) const
{
    std::unordered_map<int, int> dist;
    int id;

    for (int i = 0; i < beliefState.GetNumSamples(); i++)
    {
        const STATE* state = beliefState.GetSample(i);
        const ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<const ROS_ROCKSAMPLE_STATE&>(*state);

        // Compute state id
        id = 0;
        for (int j = 0; j < rockstate.Rocks.size(); j++)
        {
            id += rockstate.Rocks[j].Valuable*(pow(2,j));
        }


        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id] = 1;
        else
            dist[id]++;

        // std::cout << i << " $ " << id << " $ " << dist[id] << std::endl;
    }

    for (auto it = dist.begin(); it != dist.end(); ++it ){  // For each state in the belief
        ostr << it->first << ":" << it->second << " ";
    }
}

void ROS_ROCKSAMPLE::GetBeliefDistribution(const BELIEF_STATE &beliefState, std::vector<std::pair<int, int>> &beliefs) const
{
    std::unordered_map<int, int> dist;
    int id;

    for (int i = 0; i < beliefState.GetNumSamples(); i++)
    {
        const STATE* state = beliefState.GetSample(i);
        const ROS_ROCKSAMPLE_STATE& rockstate = safe_cast<const ROS_ROCKSAMPLE_STATE&>(*state);

        // Compute state id
        id = 0;
        for (int j = 0; j < rockstate.Rocks.size(); j++)
        {
            id += rockstate.Rocks[j].Valuable*(pow(2,j));
        }

        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id] = 1;
        else
            dist[id]++;
    }

    for (auto it = dist.begin(); it != dist.end(); ++it ){
        beliefs.push_back(std::make_pair(it->first, it->second));
    }

    // Sort beliefs by maximum likelihood
    std::sort(beliefs.begin(), beliefs.end(), [](auto &left, auto &right) {
        return left.second > right.second;
    });
}

void ROS_ROCKSAMPLE::Id2State(const int id, std::vector<int> &state) const
{
    state.clear();

    std::string id_s = UTILS::to_base(id, 2);

    // zero padding
    while (id_s.size() < NumRocks)
    {
        id_s = '0' + id_s;
    }

    for (size_t i = 0; i < NumRocks; i++)
    {
        state.push_back((int) id_s[i] - '0');
    }
}

void ROS_ROCKSAMPLE::ResetAgent() const
{
    geometry_msgs::Point actionMsg = geometry_msgs::Point();

    actionMsg.x = StartPos.X;
    actionMsg.y = StartPos.Y;

    pubAgentGoal.publish(actionMsg);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    
    std_msgs::Int8::ConstPtr obs;
    const_cast<ROS_ROCKSAMPLE *>(this)->waitForMessage(agentFeedbackTopic, obs);
}