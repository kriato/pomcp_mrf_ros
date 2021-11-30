#include "../include/velocity_regulation.h"

VELOCITYREGULATION::VELOCITYREGULATION(std::vector<int> nSubSegs, std::vector<std::vector<int>> subSegLengths, int nEnginePowerValues, int nDifficultyValues, int nVelocityValues, std::string historyPath, UTILS::RND &rnd, bool isReal_, bool online_, bool learnMRF_)
:   nSubSegs(nSubSegs), // Generate grid
    subSegLengths(subSegLengths),
    nDifficultyValues(nDifficultyValues),
    nVelocityValues(nVelocityValues),
    nSeg(subSegLengths.size()) // Set number of segments
{
    RND = &rnd;
    isReal = isReal_;
    online = online_;
    learnMRF = learnMRF_;
    realState = new int[nSeg];
    
    firstStep = true;
    // replayStep = 0;
    
    historyOutputFile = historyPath;
    //std::cout << "VELOCITYREGULATION::VELOCITYREGULATION 1" << std::endl;
    // Setting of protected variables of Simulator
    NumActions = nEnginePowerValues; 
    
    NumObservations = 12;  // The current version does not use observations, hence it has only 1 observation. Always 0, which enables to move in the MCTS (node QNODE->VNODE)
    collisionPenaltyTime=10; //100; 
    RewardRange = 13; //200       // TODO_ACA -100 : +100 // 1600032 // penalty 10000 e range 4000 migliore // NB:Penalty 100, reward 100 ok; penalty 100, reward 1000 puo' andare
    Discount = 0.95;
}

STATE* VELOCITYREGULATION::Copy(const STATE& state) const // Makes a copy of the state state
{
    const VELOCITYREGULATION_STATE& bmState = safe_cast<const VELOCITYREGULATION_STATE&>(state);
    VELOCITYREGULATION_STATE* newstate = MemoryPool.Allocate();
    *newstate = bmState;
    return newstate;
}

// NOT USED
void VELOCITYREGULATION::Validate(const STATE& state) const // Checks if batteryManagementState is correct
{
    const VELOCITYREGULATION_STATE& bmState = safe_cast<const VELOCITYREGULATION_STATE&>(state);
    //assert(Grid.Inside(rockstate.AgentPos));
}

void VELOCITYREGULATION::UpdateConfig(std::vector<int> &vals) const
{
    for (int i = 0; i < nSeg; i++)
    {
        realState[i] = vals[i];
    }
}

// Used by standard planner
STATE* VELOCITYREGULATION::CreateStartState() const // Creates a start state with StartPos and Rocks (positions already in the grid, values set here)
{
    
    VELOCITYREGULATION_STATE* bmState = MemoryPool.Allocate();
    bmState->dps.clear(); 
    bmState->ps.clear();
    bmState->vs.clear();
    bmState->acs.clear();
    bmState->dt1s.clear();
    bmState->dt2s.clear();
    bmState->dts.clear();
    bmState->ts.clear();
    bmState->ms.clear();
    bmState->os.clear();
    bmState->rs.clear();
    bmState->r_cums.clear();
    bmState->segDifficulties.clear();
    
    bmState->curSegI = 0;                   // Set agent position (segment i)
    bmState->curSubsegJ=0;                  // Set agent position (subsegment j)
    bmState->p=0;                           // Set initial number of collisions
    bmState->ps.push_back(0);
    bmState->t=0;                           // Set initial time
    bmState->ts.push_back(0);
    bmState->r_cums.push_back(0);                    // Set initial cumulative reward
    
    if (isReal)
    {
        firstStep = true;
    }

    //int id=0;
    int rnd;
    //std::cout << "Created state ";
    for (int i = 0; i < nSeg; ++i) // Randomly set segment difficulties
    {
        if (isReal)
        {
            rnd = realState[i];
        }
        else
        {
            #ifdef RNDGENINTERFACE
                rnd = RND->Random(nDifficultyValues);
            #else
                rnd=Random(nDifficultyValues);
            #endif
        }

        bmState->segDifficulties.push_back(rnd); 
        // std::cout << ", " << rnd;
        //id+=bmState->segDifficulties[i]*(pow(3,nSeg-i-1));
    }
    //std::cout << " ("<< id << ") - diff len "<< bmState->segDifficulties.size()<<  std::endl;
    //std::cout << std::endl;
    
    
    
    return bmState;
}

// Used by CN planner
// Notice, it works only with hard constraints (probab=1 since it works on constraint networks)
STATE* VELOCITYREGULATION::CreateStartState(std::vector<double*>* stateVarRelationships) const // Creates a start state with StartPos and Rocks (positions already in the grid, values set here)
{
    
    //std::cout << "In CreateStartState" << std::endl;
    VELOCITYREGULATION_STATE* bmState = MemoryPool.Allocate();
    /* DA SISTEMARE 
    rockstate->AgentPos = StartPos;         // Set agent position
    rockstate->Rocks.clear();               // Set rocks in the state
    // Standard initialization
    std::vector<std::vector<int>*>* connComps=computeConnComp(stateVarRelationships, NumRocks);
    //printConnectedComponents(connComps);
    
    // 0. Standard initialization
    for (int i = 0; i < NumRocks; i++)      
    {
        VELOCITYREGULATION_STATE::ENTRY entry;
        entry.Collected = false;            // Init to not collected
        entry.Valuable = Bernoulli(0.5);    // 1 with probability 0.5
        entry.Count = 0;
        entry.Measured = 0;
        entry.ProbValuable = 0.5;
        entry.LikelihoodValuable = 1.0;
        entry.LikelihoodWorthless = 1.0;
        rockstate->Rocks.push_back(entry);  // Add rock to rockstate
    }
    rockstate->Target = SelectTarget(*rockstate);
    
    // Update according to connected components
    // For each connected component
    int rnd;
    bool val;
    //printConnectedComponents(connComps);
    for(std::vector<std::vector<int>*>::iterator it = connComps->begin() ; it != connComps->end(); ++it){ // it=component
        rnd=Random((*it)->size());             // Randomly select an index in the component
        val=rockstate->Rocks[(*it)->at(rnd)].Valuable; // Get the value of the selected node 
        //std::cout << "Setting component of node " << (*it)->at(rnd) << "to " << val << std::endl;
        // Propagate the value to all the component 
        for(std::vector<int>::iterator it1 = (*it)->begin() ; it1 != (*it)->end(); ++it1){  // it1=node
            rockstate->Rocks[(*it1)].Valuable=val;
        }
    }
    
    // Update of values according to stateVarRelationships (old, wrong)
//    bool rockAlreadyVisited[NumRocks]={ 0 };
//    double rnd;
//    bool rockToChangeI; // Index in stateVarRelationships of the rock to change (0=first in the edge, 1=second in the edge)
//    int nRockAlreadyVisited=0;
//    while(nRockAlreadyVisited<NumRocks){
//        // Randomly select a rock not already visited
//    }
//    
//    
//    for (int i = 0; i < nRels; i++)      
//    {
//        // Get the state variable not already modified
//        if(rockAlreadyChanged[(int)(*stateVarRelationships)[i][1]]==0) // Second rock in the relationship not already considered
//            rockToChangeI=1;
//        else
//            if(rockAlreadyChanged[(int)(*stateVarRelationships)[i][0]]==0) // First rock in the relationship not already considered
//                rockToChangeI=0;
//            else        
//                break;  // Both rocks already modified, nothing to do with this relationship
//        rnd=RandomDouble(0,1);
//        if(rnd<=(*stateVarRelationships)[i][2])
//            rockstate->Rocks[(int)(*stateVarRelationships)[i][rockToChangeI]].Valuable = rockstate->Rocks[(int)(*stateVarRelationships)[i][!rockToChangeI]].Valuable;
//        else
//            rockstate->Rocks[(int)(*stateVarRelationships)[i][rockToChangeI]].Valuable = !(rockstate->Rocks[(int)(*stateVarRelationships)[i][!rockToChangeI]].Valuable);
//        rockAlreadyChanged[(int)(*stateVarRelationships)[i][rockToChangeI]]=1;  // Mark rock (*stateVarRelationships)[i][rockToChangeI] as already changed
//        rockAlreadyChanged[(int)(*stateVarRelationships)[i][!rockToChangeI]]=1;
//    }
     */
    return bmState;
}

// Used by standard+relationship planner
// Create new state equal to that of a particle randomly chosen from the particle filter 
STATE* VELOCITYREGULATION::CreateStartState(std::vector<STATE*>* allParticles, std::vector<double> allParticleCumProbs, int id) const // Creates a start state with StartPos and Rocks (positions already in the grid, values set here)
{
    VELOCITYREGULATION_STATE* bmState = MemoryPool.Allocate();
    bmState->id = id;

    bmState->dps.clear(); 
    bmState->ps.clear();
    bmState->vs.clear();
    bmState->acs.clear();
    bmState->dt1s.clear();
    bmState->dt2s.clear();
    bmState->dts.clear();
    bmState->ts.clear();
    bmState->ms.clear();
    bmState->os.clear();
    bmState->rs.clear();
    bmState->r_cums.clear();
    bmState->segDifficulties.clear();
    
    bmState->curSegI = 0;                   // Set agent position (segment i)
    bmState->curSubsegJ=0;                  // Set agent position (subsegment j)
    bmState->p=0;                           // Set initial number of collisions
    bmState->ps.push_back(0);
    bmState->t=0;                           // Set initial time
    bmState->ts.push_back(0);
    bmState->r_cums.push_back(0);                    // Set initial cumulative reward
  
    
    // Select a particle from allParticles with probability from allParticleProb
    #ifdef RNDGENINTERFACE
        double rnd = RND->RandomDouble(0,0.9999999999);
    #else
        double rnd = UTILS::RandomDouble(0,0.9999999999);        
    #endif

    int partI=0; // Particle index
    double cumProbTmp=0.0;

    while(cumProbTmp<=rnd)
    {
        partI++;
        cumProbTmp=allParticleCumProbs[partI];
    }
    partI--; // Found particle index

    const VELOCITYREGULATION_STATE& bmParticle = safe_cast<const VELOCITYREGULATION_STATE&>(*((*allParticles)[partI])); // Get the particle (containing rock values)
    
    
    for (int i = 0; i < nSeg; i++) // Set each segment difficulty to that of the particle
    {
        bmState->segDifficulties.push_back(bmParticle.segDifficulties[i]);
    }


    return bmState;
}

void VELOCITYREGULATION::FreeState(STATE* state) const // Free memory of state
{
    VELOCITYREGULATION_STATE* bmState = safe_cast<VELOCITYREGULATION_STATE*>(state);
    MemoryPool.Free(bmState);
}

void VELOCITYREGULATION::writeHistory(std::string what, bool append) const
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

bool VELOCITYREGULATION::StepVerbose(STATE& state, int action, int& observation, VHISTORY& vHistory, double& reward, const BELIEF_STATE& beliefState) const
{
    bool outcome;

    outcome = VELOCITYREGULATION::Step(state, action, observation, reward, beliefState);

    VELOCITYREGULATION_STATE& bmState = safe_cast<VELOCITYREGULATION_STATE&>(state);
    VELOCITYREGULATION_VHISTORY& verboseHistory = safe_cast<VELOCITYREGULATION_VHISTORY&>(vHistory);

    if (firstStep)
    {
        std::cout << "Initialized " << std::endl;
        verboseHistory.Initialize(nSubSegs, isReal);
        firstStep = false;
    }

    verboseHistory.actions.push_back(action);
    verboseHistory.observations.push_back(observation);
    verboseHistory.LogCollision(bmState.dps.back());

    return outcome;
}

bool VELOCITYREGULATION::Step(STATE& state, int action,
    int& observation, double& reward, const BELIEF_STATE& beliefState) const     // Performs a step starting from state, applying action, getting the observation and a reward
{
    // Parameters to be set
    // - Percorso
    // - Velocity model
    // - Collision model
    // - Collision penalty -> Safety
    // - Observation coding/model
    // - Prior knowledge
    //std::cout << "VELOCITYREGULATION::Step" << std::endl;
    VELOCITYREGULATION_STATE& bmState = safe_cast<VELOCITYREGULATION_STATE&>(state);
    //reward = 0;

    // Save action in history
    bmState.acs.push_back(action); // In history

    // Compute next indices
    int nextCurSegI;
    int nextCurSubsegJ;
    if((bmState.curSegI==(nSeg-1)) && (bmState.curSubsegJ==(nSubSegs[bmState.curSegI]-1))){ // Last step, end state
        nextCurSegI=bmState.curSegI;
        nextCurSubsegJ=bmState.curSubsegJ+1;
    }
    else{
        if(bmState.curSubsegJ<(nSubSegs[bmState.curSegI]-1)){
            nextCurSegI=bmState.curSegI;
            nextCurSubsegJ=bmState.curSubsegJ+1;
        }  
        else{
            nextCurSegI=bmState.curSegI+1;
            nextCurSubsegJ=0;
        }
    }
    
    //std::cout << "nextCurSegI: " << nextCurSegI << ", nextCurSubsegJ: " << nextCurSubsegJ << std::endl;
    //std::cout << "Done444aaa333ddd333bbb" << std::endl;
    // #############################
    // Velocity model: a, f -> v ###
    // #############################
    
    // MODEL 1
    /*double pv_0, pv_1, pv_2; // Velocity probabilities
    int nVel=3;
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pv_0=1.0; //0.4;
        pv_1=0.0; //0.4;
        pv_2=0.0; //0.2;
    }
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pv_0=1.0; //0.6;
        pv_1=0.0; //0.35;
        pv_2=0.0; //0.05;
    }
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pv_0=1.0; //0.8;
        pv_1=0.0; //0.2;
        pv_2=0.0; //0.0;
    }
    
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pv_0=1.0; //0.1;
        pv_1=0.0; //0.4;
        pv_2=0.0; //0.5;
    }
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pv_0=1.0; //0.2;
        pv_1=0.0; //0.6;
        pv_2=0.0; //0.2;
    }
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pv_0=1.0; //0.55;
        pv_1=0.0; //0.4;
        pv_2=0.0; //0.05;
    }
    
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pv_0=0.0;
        pv_1=0.0; //0.2;
        pv_2=1.0; //0.8;
    }
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pv_0=0.0;
        pv_1=0.5; //0.4;      
        pv_2=0.5; //0.6;
    }
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pv_0=0.0; //0.1;
        pv_1=1.0; //0.5;
        pv_2=0.0; //    0.4;
    }*/
     
    
    // MODEL 2: RECTANGLE FOR AUTONOMOUS ROBOTS
    
    bmState.v=action; // In state <------------------------------------------- VELOCITY (3)
    bmState.vs.push_back(action); // In history
    //vs[bmState.curSegI].push_back(v);
    
    // #######################################
    // Collision model: a, f, (l)-> dp #######
    // #######################################
    //std::cout << "Done444aaa333ddd333ddd" << std::endl;
    double pdp_0, pdp_1; // 0: no collision, 1: yes collision
    
    // Modello semplificato per test
    // Idea: facile (diff=0) -> vai a 2, difficile -> vai a 0, medio -> probab media
    /*if((action==0) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pdp_1=0.0; // 0.0
    }
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pdp_1=0.0; // 0.025
    }
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pdp_1=0.0; // 0.025
    }
    
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pdp_1=0.0; // 0.0
    }
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pdp_1=0.5; // 0.025
    }
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pdp_1=1.0; // 0.025
    }
    
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pdp_1=0.0; // 0.0
    }
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pdp_1=0.9; // 0.05
    }
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        pdp_1=1.0; // 0.075
    }*/
    
    
    // Modello exp 2 Enrico
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        pdp_1=0.0; 
    }
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        pdp_1=0.0; 
    }
    if((action==0) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        // pdp_1=0.028;        // ICE
        pdp_1=0.0;       // RECTANGULAR
        // pdp_1 = 0.0;     // OUR
    }
    
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        // pdp_1=0.0;          // ICE
        pdp_1=0.033;     // RECTANGULAR
        // pdp_1=0.0;       // OUR
    }
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        // pdp_1=0.056;        // ICE
        pdp_1=0.033;        // RECTANGULAR
        // pdp_1=0.05;      // OUR
    }
    if((action==1) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        // pdp_1=0.011;        // ICE
        pdp_1=0.067;     // RECTANGULAR
        // pdp_1=0.09;      // OUR
    }
    
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==0)){ // Low difficulty
        // pdp_1=0.0;          // ICE
        pdp_1=0.033;        // RECTANGULAR
        // pdp_1=0.0;       // OUR
    }
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==1)){ // Medium difficulty
        // pdp_1=0.14;        // ICE
        pdp_1=0.067;        // RECTANGULAR
        // pdp_1=1.0;      // OUR
    }
    if((action==2) && (bmState.segDifficulties[bmState.curSegI]==2)){ // High difficulty
        // pdp_1=0.25;          // ICE
        pdp_1=0.1;          // RECTANGULAR
        // pdp_1=1.0;        // OUR
    }
    pdp_0=1-pdp_1;
    
    
    #ifdef RNDGENINTERFACE
        double rnd = RND->RandomDouble(0,0.9999999999);
    #else
        double rnd = UTILS::RandomDouble(0,0.9999999999);        
    #endif

    int dp=-1;
    if(rnd<pdp_0){ // No collision
        dp=0;
    }

    if(rnd>=pdp_0 && rnd<pdp_0+pdp_1){ // Yes collision
        dp=1;
    }

    bmState.dp=dp; // In state <------------------------- NCOLLISIONS (1)
    //dps[bmState.curSegI].push_back(dp);
    bmState.dps.push_back(dp); // In history
    //std::cout << "Done444aaa333ddd333eee" << std::endl;
    
    // Compute total collisions from beginning <-----------NCOLLISIONSTOT (2)
    bmState.p=bmState.p+dp;
    bmState.ps.push_back(bmState.p); // Update std::vector of total collisions 
    //ps[nextCurSegI].push_back(bmState.p);
    
    // From velocity, length and collisions compute the time to traverse the subsegment
    int dt, dt1, dt2;
    dt1=(nVelocityValues-bmState.v)*subSegLengths[bmState.curSegI][bmState.curSubsegJ]; // v=0 -> t=3*length; v=2 -> t=1*length
    bmState.dt1s.push_back(dt1); // Update segment of times per subsegment <---- delta TIME (5a)
    //dts[bmState.curSegI].push_back(dt);
    bmState.dt1=dt1;
    //std::cout << "Done444aaa333ddd333fff" << std::endl;
    
    
    dt2=dp*collisionPenaltyTime*subSegLengths[bmState.curSegI][bmState.curSubsegJ]; // dp=0 -> no collision -> dt2=0, dp=1 -> yes collision -> dt2=5*seg length (a random process can be added here)
    bmState.dt2s.push_back(dt2); // Update segment of times per subsegment <---- delta TIME (5b)
    //dts[bmState.curSegI].push_back(dt);
    bmState.dt2=dt2;
    
    dt=dt1+dt2;
    bmState.dts.push_back(dt); // Update segment of times per subsegment <---- delta TIME (5b)
    //dts[bmState.curSegI].push_back(dt);
    bmState.dt=dt;
    //std::cout << "Done444aaa333ddd333ggg" << std::endl;
    
    // Compute next cumulative time <------- TIME_CUM (6)
    bmState.t=bmState.t+dt1+dt2;
    
    bmState.ts.push_back(bmState.t); // Update std::vector of cumulative times per subsegment  
    //ts[nextCurSegI].push_back(bmState.t);
    //std::cout << "Done444aaa333ddd333hhh" << std::endl;
    
    // #######################################
    // Angular velocity model: f -> m #######
    // #######################################
    double pm_0, pm_1; // 0: low number of times in which angular velocity used, 1: high number of times in which angular velocity used
    if(bmState.segDifficulties[bmState.curSegI]==0){ // Low difficulty
        // pm_1=0.083;             // ICE
        pm_1=0.17;           // RECTANGULAR
        // pm_1=0.17;           // OUR
    }
    if(bmState.segDifficulties[bmState.curSegI]==1){ // Medium difficulty
        // pm_1=0.3;              // ICE
        pm_1=0.24;          // RECTANGULAR
        // pm_1=0.24;          // OUR
    }
    if(bmState.segDifficulties[bmState.curSegI]==2){ // High difficulty
        // pm_1=0.3;             // ICE
        pm_1=0.53;         // RECTANGULAR
        // pm_1=0.53;             // OUR

    }
    pm_0=1-pm_1;
    
    #ifdef RNDGENINTERFACE
        rnd = RND->RandomDouble(0,0.9999999999);
    #else
        rnd = UTILS::RandomDouble(0,0.9999999999);        
    #endif

    int m=-1;
    if(rnd<pm_0){ // low number of times in which angular velocity used
        m=0;
    }
    else
        m=1;
    bmState.m=m; // In state <------------------------- 
    bmState.ms.push_back(m); // In history
    
    // #######################################
    // Occupancy model: f -> o ###############
    // #######################################
    double po_0, po_1; // 0: low occupancy of the subsegment, 1: high occupancy of the subsegment
    
    // Modello osservativo piu informativo per test
    /*if(bmState.segDifficulties[bmState.curSegI]==0){ // Low difficulty
        po_1=0.0; // 0.17
    }
    if(bmState.segDifficulties[bmState.curSegI]==1){ // Medium difficulty
        po_1=0.5; // 0.29
    }
    if(bmState.segDifficulties[bmState.curSegI]==2){ // High difficulty
        po_1=1.0; // 0.51
    }*/
    
    // Modello osservativo exp 2 Enrico 
    if(bmState.segDifficulties[bmState.curSegI]==0){ // Low difficulty
        // po_1=0.65; //            // ICE
        po_1=0.6; //            // RECTANGULAR
        // po_1=0.0; //         // OUR
    }
    if(bmState.segDifficulties[bmState.curSegI]==1){ // Medium difficulty
        // po_1=0.83; //           // ICE
        po_1=0.69; //           // RECTANGULAR
        // po_1=0.5; //         // OUR
    }
    if(bmState.segDifficulties[bmState.curSegI]==2){ // High difficulty
        // po_1=0.93; //           // ICE
        po_1=0.94; //           // RECTANGULAR
        // po_1=1.0; //         // OUR
    }

    // EDIT ME HEEERE!!!!
    // po_1 = 0;

    po_0=1-po_1;

    #ifdef RNDGENINTERFACE
        rnd = RND->RandomDouble(0,0.9999999999);
    #else
        rnd = UTILS::RandomDouble(0,0.9999999999);        
    #endif

    int o=-1;
    if(rnd<po_0){ // low number of times in which angular velocity used
        o=0;
    }
    else
        o=1;
    bmState.o=o; // In state <------------------------- 
    bmState.os.push_back(o); // In history
    
    // Observation
    // observation=0;
    observation=(dt1-1) + nVelocityValues*m + nVelocityValues*2*o; // ANGULAR VELOCITY
    // observation=(dt1-1) + nVelocityValues + nVelocityValues*2*o;
    // observation=(dt1-1) + nVelocityValues;

    //observation=(dt1-1)+nVel*dp; // obs= 0: dp=0,dt1=1, 
                                //      1: dp=0,dt1=2, 
                                //      2: dp=0,dt1=3, 
                                //      3: dp=1,dt1=1,
                                //      4: dp=1,dt1=2,
                                //      5: dp=1,dt1=3,
    //observation=(dt1-1);     // Observation of the robot velocity ()
    //observation=(dt1-1)+nVel*dp;// Observation of the robot velocity and collisions 
    //observation=dp;
    
    // Reward
    reward=-dt;
    
    bmState.r=reward; // <-------- REWARD (7)
    bmState.rs.push_back(reward);
    //rs[bmState.curSegI].push_back(reward);
    bmState.r_cum=bmState.r_cum+reward;
    //r_cums[nextCurSegI].push_back(bmState.r_cum);
    bmState.r_cums.push_back(bmState.r_cum);
    
    // Print
    /*
    std::cout << "------ curSegI: " << bmState.curSegI << ", curSubsegJ: " << bmState.curSubsegJ << std::endl;
    std::cout << "--- Current difficulty: " << bmState.segDifficulties[bmState.curSegI] << std::endl;
    std::cout << "--- action (a): " << action << std::endl;
    std::cout << "--- Velocity (v): " << v << std::endl;
    std::cout << "--- Next voltage (p): " << bmState.p << std::endl;
    std::cout << "--- Next time (t): " << bmState.t << std::endl;
    std::cout << "--- Reward (r): " << reward << std::endl;
    std::cout << "--- Cumulative Reward (r_cum): " << bmState.r_cum << std::endl;
    std::cout << "Delta voltage (dp): " << dp << std::endl;
    std::cout << "dt: " << dt << std::endl;
    std::cout << "pv_0: " << pv_0 << ", pv_1: " << pv_1 << ", pv_2: " << pv_2 << ", rnd: " << rnd << std::endl;
    std::cout << "pdp_0: " << pdp_0 << ", pdp_1: " << pdp_1 << ", rnd1: " << rnd1 << std::endl;
    */
    //std::cout << "Done444aaa333ddd333lll" << std::endl;
    
    // Save current distance state-belief (only for data analysis)
    double dist=0; // Total distance real state <-> belief
    int hammingDist=0; // Hamming distance between single state and belief
    int res=0;
    
    if(beliefState.GetNumSamples()!=0){ // Not fake belief, used to identify simulation steps
        std::unordered_map<int, int> nParticlesPerState;
        int stateId;
        int stateTmp;
        for (int i = 0; i < beliefState.GetNumSamples(); i++){ // Compute map nParticlesPerState: state -> nParticles
            const STATE* state = beliefState.GetSample(i);
            const VELOCITYREGULATION_STATE& particleState = safe_cast<const VELOCITYREGULATION_STATE&>(*state);

            // Compute state id
            stateId=0;   
            for (int j = 0; j<nSeg; j++)  // For each segment difficulty
            {
                stateId+=particleState.segDifficulties[j]*(pow(nDifficultyValues,nSeg-j-1));
            }
            // If it is not in nParticlesPerState then add it and initialize to 1
            if (nParticlesPerState.find(stateId) == nParticlesPerState.end())
                nParticlesPerState[stateId]= 1;
            else
                nParticlesPerState[stateId]++;
        }
        // Compute distance as weighted sum of hamming distance between particle state and real state (where weight is belief probability)
        for (auto s = nParticlesPerState.begin(); s != nParticlesPerState.end(); ++s ){  // For each state in the belief
            //std::cout << "??????? Particle State: " << s->first << std::endl;
            stateTmp=s->first; // stateId
            hammingDist=0;
            //std::cout << "Real state: ";
            for(int i = nSeg-1; i>=0; i--)// Compute Hamming distance between real state and particle s
            {
                //std::cout << bmState.segDifficulties[i] << ", ";
                res=stateTmp%nDifficultyValues;
                stateTmp=stateTmp/nDifficultyValues;
                hammingDist=hammingDist+std::abs(bmState.segDifficulties[i]-res); // Hamming distance between real state 
            }
            //std::cout << std::endl;
            
            //std::cout << "Hamming dist: " << hammingDist << std::endl;
            dist=dist + ((double)s->second/beliefState.GetNumSamples())*hammingDist;
            //std::cout << "# particles: " << s->second << std::endl;
            //std::cout << "# all particles: " << beliefState.GetNumSamples() << std::endl;
            //std::cout << "Added dist: " << ((double)s->second/beliefState.GetNumSamples())*hammingDist << std::endl;
        }
        bmState.dist_state_beliefs.push_back(dist);

    }

    //else{
    //    std::cout << "+++++++++++++++++++++++++++++++++++++++++++++Sim: No belief analysis" << std::endl;
    //}
    
    if (isReal)
    {
        std::string what =  std::to_string(reward) + " " + 
                            std::to_string(action) + " " + 
                            std::to_string(observation) + " " + 
                            std::to_string(dp) + " " + 
                            std::to_string(dist);

        std::vector<std::pair<int, int>> belDistribution;
        GetBeliefDistribution(beliefState, belDistribution);
        std::vector<int> state;

        Id2State(belDistribution[0].first, state);
        for(size_t i = 0; i < state.size(); i++)
        {
            what += " " + std::to_string(state[i]);
        }

        writeHistory(what);
    }

    
    if((bmState.curSegI==(nSeg-1)) && (bmState.curSubsegJ==(nSubSegs[bmState.curSegI])-1)) {// path end reached, battery not exhausted
        //std::cout << "VELOCITYREGULATION::Step: end of path reached with not exhausted battery!" << std::endl;
        //std::cout << "------------------------" << std::endl;
        return true; // True means terminal state
    }
    else{
        //std::cout << "------------------------" << std::endl;
        // Update position, to be ready for next step
        bmState.curSegI=nextCurSegI;
        bmState.curSubsegJ=nextCurSubsegJ;
        return false; // True means non-terminal state
    }
}

// TO BE IMPLEMENTED: now it returns the same state provided in input
bool VELOCITYREGULATION::LocalMove(STATE& state, const HISTORY& history,    // Make a local change in the state (i.e., it inverts the value of a rock) with a certain probability related to the probability to observe from the new state the observation really taken in the history
    int stepObs, const STATUS& status) const
{
    /* TO BE IMPLEMENTED
    VELOCITYREGULATION_STATE& bmState = safe_cast<VELOCITYREGULATION_STATE&>(state);  // State
    
    int rock = Random(NumRocks);                                        // Take a random rock
    rockstate.Rocks[rock].Valuable = !rockstate.Rocks[rock].Valuable;   // Change the value of one rock of rockstate

    if (history.Back().Action > E_SAMPLE) // If the last action was a rock check
    {
        rock = history.Back().Action - E_SAMPLE - 1;    // Get the sampled rock
        int realObs = history.Back().Observation;       // Get the observation

        // Condition new state on real observation
        int newObs = GetObservation(rockstate, rock);   // Get obs from rock in the new state
        if (newObs != realObs)                          // If the new observation is different from the real one -> false
            return false;

        // Update counts to be consistent with real observation
        if (realObs == E_GOOD && stepObs == E_BAD)
            rockstate.Rocks[rock].Count += 2;
        if (realObs == E_BAD && stepObs == E_GOOD)
            rockstate.Rocks[rock].Count -= 2;
    }
    */
    return true;
}



// Puts in legal a set of legal actions that can be taken from state
void VELOCITYREGULATION::GenerateLegal(const STATE& state, const HISTORY& history,
    std::vector<int>& legal, const STATUS& status) const
{
    //const VELOCITYREGULATION_STATE& bmState = safe_cast<const VELOCITYREGULATION_STATE&>(state);
    legal.push_back(0);
    legal.push_back(1);
    legal.push_back(2);
}

void VELOCITYREGULATION::GeneratePreferred(const STATE& state, const HISTORY& history,
    std::vector<int>& actions, const STATUS& status) const
{
    // Not implemented so far. This is a copy of GenerateLegal
    actions.push_back(0);
    actions.push_back(1);
    actions.push_back(2);
    
//	static const bool UseBlindPolicy = false;
//
//	if (UseBlindPolicy)
//	{
//		actions.push_back(COORD::E_EAST);
//		return;
//	}
//
//	const VELOCITYREGULATION_STATE& rockstate =
//	        safe_cast<const VELOCITYREGULATION_STATE&>(state);
//
//	// Sample rocks with more +ve than -ve observations
//	int rock = Grid(rockstate.AgentPos);
//	if (rock >= 0 && !rockstate.Rocks[rock].Collected)
//	{
//		int total = 0;
//		for (int t = 0; t < history.Size(); ++t)
//		{
//			if (history[t].Action == rock + 1 + E_SAMPLE)
//			{
//				if (history[t].Observation == E_GOOD)
//					total++;
//				if (history[t].Observation == E_BAD)
//					total--;
//			}
//		}
//		if (total > 0)
//		{
//			actions.push_back(E_SAMPLE);
//			return;
//		}
//
//	}
//
//	// processes the rocks
//	bool all_bad = true;
//	bool north_interesting = false;
//	bool south_interesting = false;
//	bool west_interesting  = false;
//	bool east_interesting  = false;
//
//	for (int rock = 0; rock < NumRocks; ++rock)
//	{
//		const VELOCITYREGULATION_STATE::ENTRY& entry = rockstate.Rocks[rock];
//		if (!entry.Collected)
//		{
//			int total = 0;
//			for (int t = 0; t < history.Size(); ++t)
//			{
//				if (history[t].Action == rock + 1 + E_SAMPLE)
//				{
//					if (history[t].Observation == E_GOOD)
//						total++;
//					if (history[t].Observation == E_BAD)
//						total--;
//				}
//			}
//
//			if (total >= 0)
//			{
//				all_bad = false;
//
//				if (RockPos[rock].Y > rockstate.AgentPos.Y)
//					north_interesting = true;
//				if (RockPos[rock].Y < rockstate.AgentPos.Y)
//					south_interesting = true;
//				if (RockPos[rock].X < rockstate.AgentPos.X)
//					west_interesting = true;
//				if (RockPos[rock].X > rockstate.AgentPos.X)
//					east_interesting = true;
//			}
//		}
//	}
//
//	// if all remaining rocks seem bad, then head east
//	if (all_bad)
//	{
//		actions.push_back(COORD::E_EAST);
//		return;
//	}
//
//	// generate a random legal move, with the exceptions that:
//	//   a) there is no point measuring a rock that is already collected
//	//   b) there is no point measuring a rock too often
//	//   c) there is no point measuring a rock which is clearly bad or good
//	//   d) we never sample a rock (since we need to be sure)
//	//   e) we never move in a direction that doesn't take us closer to
//	//      either the edge of the map or an interesting rock
//	if (rockstate.AgentPos.Y + 1 < Size && north_interesting)
//			actions.push_back(COORD::E_NORTH);
//
//	if (east_interesting)
//		actions.push_back(COORD::E_EAST);
//
//	if (rockstate.AgentPos.Y - 1 >= 0 && south_interesting)
//		actions.push_back(COORD::E_SOUTH);
//
//	if (rockstate.AgentPos.X - 1 >= 0 && west_interesting)
//		actions.push_back(COORD::E_WEST);
//
//
//	for (rock = 0; rock < NumRocks; ++rock)
//	{
//		if (!rockstate.Rocks[rock].Collected    &&
//			rockstate.Rocks[rock].ProbValuable != 0.0 &&
//			rockstate.Rocks[rock].ProbValuable != 1.0 &&
//			rockstate.Rocks[rock].Measured < 5  &&
//			std::abs(rockstate.Rocks[rock].Count) < 2)
//		{
//			actions.push_back(rock + 1 + E_SAMPLE);
//		}
//	}
}

// Not needed by simulator
//int VELOCITYREGULATION::GetObservation(const VELOCITYREGULATION_STATE& rockstate, int rock) const // Computes the rock sample observation
//{
//    double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);  // Compute distance
//    double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;     // Compute efficiency
//
//    if (Bernoulli(efficiency))  // Computes observation based on efficiency
//        return rockstate.Rocks[rock].Valuable ? E_GOOD : E_BAD;      // Coorect observation               
//    else
//        return rockstate.Rocks[rock].Valuable ? E_BAD : E_GOOD;      // Wrong observation
//    return 0; // Notice: not used so far
//}

// Not needed: Only for rocksample
//int VELOCITYREGULATION::SelectTarget(const VELOCITYREGULATION_STATE& rockstate) const   // Should select the closest rock but now always return -1
//{
//    int bestDist = Size * 2;
//    int bestRock = -1;
//    for (int rock = 0; rock < NumRocks; ++rock)
//    {
//        if (!rockstate.Rocks[rock].Collected
//            && rockstate.Rocks[rock].Count >= UncertaintyCount)
//        {
//            int dist = COORD::ManhattanDistance(rockstate.AgentPos, RockPos[rock]); // Compute distance to rock
//            if (dist < bestDist)
//                bestDist = dist;  // bestRock not updated here!!!          
//        }
//    }
//    return bestRock;
//}


// Display methods -------------------------
void VELOCITYREGULATION::DisplayBeliefs(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
    std::ostream& ostr) const
{// ACA
    std::cout << "VELOCITYREGULATION::DisplayBeliefs start" << std::endl;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayState(*s,std::cout);
    }
    std::cout << "VELOCITYREGULATION::DisplayBeliefs end" << std::endl;
}

// ACA
void VELOCITYREGULATION::DisplayBeliefIds(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
    std::ostream& ostr) const
{
    std::cout << "VELOCITYREGULATION::DisplayBeliefIds: [";
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* s = beliefState.GetSample(i);
        DisplayStateId(*s, std::cout);std::cout <<"; ";
    }
    std::cout << "VELOCITYREGULATION::DisplayBeliefs end" << std::endl;
}

// ACAB
void VELOCITYREGULATION::DisplayBeliefDistribution(const BELIEF_STATE& beliefState,    // Displays all states in beliefState
    std::ostream& ostr) const
{
    std::unordered_map<int, int> dist;
    int id;
    for (int i = 0; i < beliefState.GetNumSamples(); i++){
        const STATE* state = beliefState.GetSample(i);
        const VELOCITYREGULATION_STATE& bmState = safe_cast<const VELOCITYREGULATION_STATE&>(*state);
        
        // Compute state id
        id=0;   
        for (int j = 0; j<nSeg; j++)  // For each segment difficulty
        {
            id+=bmState.segDifficulties[j]*(pow(3,nSeg-j-1));
        }
        // If it is not in dist then add it and initialize to 1
        if (dist.find(id) == dist.end())
            dist[id]= 1;
        else
            dist[id]++;
    }
    for (auto it = dist.begin(); it != dist.end(); ++it ){  // For each state in the belief
        ostr << it->first << ":" << it->second << ", ";
    }
}

void VELOCITYREGULATION::DisplayState(const STATE& state, std::ostream& ostr) const // Displays the STATE (grid with agent and rocks)
{
    const VELOCITYREGULATION_STATE& bmState = safe_cast<const VELOCITYREGULATION_STATE&>(state);
    ostr << std::endl;
    
    // Display segment difficulties
    ostr << "## STATE ############" << std::endl;
    ostr << "Difficulties: ";
    for (int i = 0; i < nSeg; i++)
        ostr << i << ":" << bmState.segDifficulties[i] << ", ";
    ostr << std::endl;
    
    // Display agent's position
    ostr << "Position: i:" << bmState.curSegI << ", j:" << bmState.curSubsegJ << std::endl;
    
    // 1. Display delta battery voltage in last subsegment
    // ostr << "Delta battery voltage in last subsegment: " << bmState.dp << std::endl;
    ostr << "dp: " << bmState.dp << std::endl;
    
    // 2. Display voltage
    //ostr << "Current voltage: " << bmState.p << std::endl;
    ostr << "p: " << bmState.p << std::endl;
    
    // 3. Display Velocity in the last subsegment
    // ostr << "Velocity in the last subsegment: " << bmState.v << std::endl;
    ostr << "v: " << bmState.v << std::endl;
   
    
    // 5a. Delta time due to normal navigation in last subsegment
    // ostr << "Delta time in last subsegment: " << bmState.dt << std::endl;
    ostr << "dt1: " << bmState.dt1 << std::endl;
    
    // 5b. Delta time due to collisions in last subsegment
    // ostr << "Delta time in last subsegment: " << bmState.dt << std::endl;
    ostr << "dt2: " << bmState.dt2 << std::endl;
    
    // 5. Delta time in last subsegment
    // ostr << "Delta time in last subsegment: " << bmState.dt << std::endl;
    ostr << "dt: " << bmState.dt << std::endl;
    
    
    // 6. Display time
    // ostr << "Current time: " << bmState.t << std::endl;
    ostr << "t: " << bmState.t << std::endl;
    
    // 7. Avg ang velocity
    // ostr << "Current time: " << bmState.t << std::endl;
    ostr << "m: " << bmState.m << std::endl;
    
    // 8. Occupancy
    // ostr << "Reward last subsegment: " << bmState.r << std::endl;
    ostr << "o: " << bmState.o << std::endl;
    
    // 9. Reward
    // ostr << "Reward last subsegment: " << bmState.r << std::endl;
    ostr << "r: " << bmState.r << std::endl;
    
    // 10. Display cumulative reward
    // ostr << "Current cumulative reward: " << bmState.r_cum << std::endl;
    ostr << "r_cum: " << bmState.r_cum << std::endl;


    ostr << "#######################" << std::endl<< std::endl;
}

void VELOCITYREGULATION::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const // Prints the observation
{
    switch (observation)
    {
    case 0:    // observation=E_NONE -> Nothing observed (action!=sampling)
        ostr << "Observed dp=0, dt1=1" << std::endl;
        break;
    case 1:    // observation=E_NONE -> Nothing observed (action!=sampling)
        ostr << "Observed dp=0, dt1=2" << std::endl;
        break;
    case 2:    // observation=E_NONE -> Nothing observed (action!=sampling)
        ostr << "Observed dp=0, dt1=3" << std::endl;
        break;
    case 3:    // observation=E_NONE -> Nothing observed (action!=sampling)
        ostr << "Observed dp=1, dt1=1" << std::endl;
        break;
    case 4:    // observation=E_NONE -> Nothing observed (action!=sampling)
        ostr << "Observed dp=1, dt1=2" << std::endl;
        break;
    case 5:    // observation=E_NONE -> Nothing observed (action!=sampling)
        ostr << "Observed dp=1, dt1=3" << std::endl;
        break;
    }
}

void VELOCITYREGULATION::DisplayAction(int action, std::ostream& ostr) const // Prints the action performed
{
    if (action == 0)  
        ostr << "Action: Low power (0)" << std::endl;
    if (action == 1) 
        ostr << "Action: Medium power (1)" << std::endl;
    if (action == 2) 
        ostr << "Action: High power (2)" << std::endl;
}


// ACA METHODS

double VELOCITYREGULATION::JointProb(const STATE& state) const // Displays the STATE (grid with agent and rocks)
{
    // not implemented yet
    return 1.0;
//    const VELOCITYREGULATION_STATE& rockstate = safe_cast<const VELOCITYREGULATION_STATE&>(state);
//    int nRel=4; // Two known relationships
//    double rockRelationships[nRel][4];  
//    // Relationship between rock 1 and rock 2
//    rockRelationships[0][0]=0;
//    rockRelationships[0][1]=1;
//    // Relationship between rock 3 and rock 4
//    rockRelationships[1][0]=2;
//    rockRelationships[1][1]=3;
//    
//    rockRelationships[2][0]=4;
//    rockRelationships[2][1]=5;
//    
//    rockRelationships[3][0]=6;
//    rockRelationships[3][1]=7;
//    
//    double jp=1.0;  // Joint probability
//    for (int i = 0; i<nRel; i++)
//    {
//        if(rockstate.Rocks[rockRelationships[i][0]].Valuable == rockstate.Rocks[rockRelationships[i][1]].Valuable)
//            jp*=1.0;    // Hard 
//        else
//            jp*=0.0;
//        
//    }
//    
//    return jp;
}

void VELOCITYREGULATION::DisplayStateId(const STATE& state, std::ostream& ostr) const // Displays an id from rock values
{
    const VELOCITYREGULATION_STATE& bmState = safe_cast<const VELOCITYREGULATION_STATE&>(state);
    int id=0;
    std::string s="State id: ";
    for (int i = 0; i<nSeg; i++)  // For each segment difficulty
    {
        id+=bmState.segDifficulties[i]*(pow(nDifficultyValues,nSeg-i-1));
        s=s+std::to_string(bmState.segDifficulties[i]);
        if(i!=(nSeg-1)){
            s=s+"-";
        }
    }
    std::cout << s << "(" << id << ")";
    //std::cout << id;
}

void VELOCITYREGULATION::DisplayStateHist(const STATE& state, const char* fileName) const // Displays an id from rock values
{
    const VELOCITYREGULATION_STATE& bmState = safe_cast<const VELOCITYREGULATION_STATE&>(state);
    std::ofstream outFile;
    outFile.open(fileName, std::ofstream::out | std::ofstream::app);
    
    std::string s;
//    std::string s="Segment difficulties";
//    for (int i = 0; i<bmState.segDifficulties.size(); i++)  // For each segment difficulty
//    {
//        s=s + ", " + std::to_string(bmState.segDifficulties[i]);
//    }
//    outFile << s << std::endl;
    
    s="Step";
    int step=0;
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + std::to_string(step);
            step=step+1;
        }
    }
    outFile << s << std::endl;
    
    s="Segments";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + std::to_string(i);
        }
    }
    outFile << s << std::endl;
    
    s="SubSegments";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + std::to_string(j);
        }
    }
    outFile << s << std::endl;
    
    s="Lengths";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + std::to_string(subSegLengths[i][j]);
        }
    }
    outFile << s << std::endl;
    
    int stateId=0;   
    for (int i = 0; i<nSeg; i++)  // For each segment difficulty
    {
        stateId+=bmState.segDifficulties[i]*(pow(3,nSeg-i-1));
    }
    s="Difficulties ("+ std::to_string(stateId)+")";
    for (int i = 0; i<nSeg; i++)  // For each segment
    {
        for (int j = 0; j<nSubSegs[i]; j++)  // For each subsegment
        {
            s=s + ", " + std::to_string(bmState.segDifficulties[i]);
        }
    }
    outFile << s << std::endl;
    
    
    
    //std::cout << std::endl << "State evolution:" << std::endl;
    // Delta battery voltage in last subsegment (1)
    s="dps";
    for (int i = 0; i<bmState.dps.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.dps[i]);
    }
    
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Battery voltage at the end of the last subsegment (2)
    s="p";
    for (int i = 0; i<bmState.ps.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.ps[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Velocity in the last subsegment (3)
    s="v";
    for (int i = 0; i<bmState.vs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.vs[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Actions (4)
    s="acs";
    for (int i = 0; i<bmState.acs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.acs[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Delta time in last segment (5a)
    s="dt1s";
    for (int i = 0; i<bmState.dt1s.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.dt1s[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Delta time in last segment (5b)
    s="dt2s";
    for (int i = 0; i<bmState.dt2s.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.dt2s[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Delta time in last segment (5)
    s="dts";
    for (int i = 0; i<bmState.dts.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.dts[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Time from the beginning (cumulative) (6)
    s="ts";
    for (int i = 0; i<bmState.ts.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.ts[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Average angular velocity (m)
    s="ms";
    for (int i = 0; i<bmState.ms.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.ms[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Occupancy (o)
    s="os";
    for (int i = 0; i<bmState.os.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.os[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Reward last subsegment (7)
    s="rs";
    for (int i = 0; i<bmState.rs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.rs[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Cumulative reward (8)
    s="r_cums";
    for (int i = 0; i<bmState.r_cums.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.r_cums[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl;
    
    // Cumulative reward (8)
    s="dist_state_bel";
    for (int i = 0; i<bmState.dist_state_beliefs.size(); i++)  // For each segment difficulty
    {
        s=s + ", " + std::to_string(bmState.dist_state_beliefs[i]);
    }
    //std::cout << s << std::endl;
    outFile << s << std::endl<< std::endl;

    outFile.close();
    //std::cout << id;
}


std::vector<double*>* VELOCITYREGULATION::CreateStateRelKnowledge(const STATE& state, int nConnComp, double relProbab) const // Generates probabilistic knowledge about relationships between state-variables
{
    std::cout << "In CreateStateRelKnowledge" << std::endl;
    const VELOCITYREGULATION_STATE& oaState = safe_cast<const VELOCITYREGULATION_STATE&>(state);
    // std::vector of knowledge
    std::vector<double*>* stateVarRelationships=new std::vector<double*>();
    // std::vector of all possible edges ()
    std::vector<double*>* allStateVarRelationships=new std::vector<double*>();
    for (int i = 0; i<oaState.segDifficulties.size()-1; i++)
    {
        for (int j = i+1; j<oaState.segDifficulties.size(); j++)
        {
            //std::cout << "aaa1" << std::endl;
            if(oaState.segDifficulties[i]==oaState.segDifficulties[j]){
                allStateVarRelationships->push_back(new double[3]{(double)i, (double)j,relProbab});
            }
        }
    }
    // Check how many connected components there are
    
    int nTrueConnComp=oaState.segDifficulties.size(); // N edge in stateVarRelationships
    int edgeI=-1;
    std::vector<std::vector<int>*>* connComps;
    //std::cout << "bbb" << std::endl;
    while(nTrueConnComp>nConnComp){
        //std::cout << "nTrueConnComp: " << nTrueConnComp << std::endl;
        
        #ifdef RNDGENINTERFACE
            edgeI = RND->Random(allStateVarRelationships->size());// Get random edge
        #else
            edgeI = UTILS::Random(allStateVarRelationships->size());// Get random edge
        #endif

        //std::cout << "adding edge: " << edgeI << ": " << (*allStateVarRelationships)[edgeI][0] << "-" << (*allStateVarRelationships)[edgeI][1] << std::endl;
        // If the edge is not in the knowledge
        stateVarRelationships->push_back((*allStateVarRelationships)[edgeI]); // Add the edge to the knowledge
        allStateVarRelationships->erase(allStateVarRelationships->begin()+edgeI);
        // Check how many connected components there are
        connComps=UTILS::computeConnComp(stateVarRelationships,oaState.segDifficulties.size());
        nTrueConnComp=connComps->size();
    }
    std::cout << "StateRelKnowledge computed. Connected components:" << std::endl;
    UTILS::printConnectedComponents(connComps);
    return stateVarRelationships;
}


// ACA: it works with knowledge about state-variable relationships
bool VELOCITYREGULATION::LocalMove(STATE& state, const HISTORY& history,    // Make a local change in the state (i.e., it inverts the value of a rock) with a certain probability related to the probability to observe from the new state the observation really taken in the history
    int stepObs, const STATUS& status, std::vector<double*>* stateVarRelationships) const
{
    /* TO BE IMPLEMENTED
    //std::cout << "In VELOCITYREGULATION::LocalMove (ACA)" << std::endl;
    
    VELOCITYREGULATION_STATE& rockstate = safe_cast<VELOCITYREGULATION_STATE&>(state);  // State
    int rock = Random(NumRocks);                                        // Take a random rock
    rockstate.Rocks[rock].Valuable = !rockstate.Rocks[rock].Valuable;   // Change the value of one rock of rockstate
    //std::cout << "Changing rock " << rock <<". New value=" << rockstate.Rocks[rock].Valuable << std::endl;
    // Propagate the change to the rest of the state according to probabilistic knowledge
    std::vector<int> *alreadyExploredVarIndices = new std::vector<int>();
    alreadyExploredVarIndices->push_back(rock);
    
    PropagateChange(rockstate,rock,stateVarRelationships,alreadyExploredVarIndices); 
    
    if (history.Back().Action > E_SAMPLE) // If the last action was a rock check
    {
        rock = history.Back().Action - E_SAMPLE - 1;    // Get the sampled rock
        int realObs = history.Back().Observation;       // Get the real observation got from the last action

        // Condition new state on real observation
        int newObs = GetObservation(rockstate, rock);   // Get obs from rock in the new state
        if (newObs != realObs)                          // If the new observation is different from the real one -> false
            return false;

        // Update counts to be consistent with real observation
        if (realObs == E_GOOD && stepObs == E_BAD)
            rockstate.Rocks[rock].Count += 2;
        if (realObs == E_BAD && stepObs == E_GOOD)
            rockstate.Rocks[rock].Count -= 2;
    }
    */
    return true;
}

// TO BE IMPLEMENTED
// Notice: in case of prob=1 changes could be simply propagated in all the connected component
void VELOCITYREGULATION::PropagateChange(STATE& state, int changedVariable,  
        std::vector<double*>* stateVarRelationships,  std::vector<int>* alreadyExploredVarIndices) const
{
    
    //std::cout << std::endl;
    //std::cout << "In VELOCITYREGULATION::PropagateChange (ACA)" << std::endl;
    //std::cout << "Analyzing node " << changedVariable << std::endl;
    /* TO BE IMPLEMENTED
    VELOCITYREGULATION_STATE& rockstate = safe_cast<VELOCITYREGULATION_STATE&>(state);  // State    
    std::vector<double*> adjIs = FindUnexploredAdjacentVariableIndices(changedVariable,stateVarRelationships,alreadyExploredVarIndices);
    //std::cout << "adjIs.size():" << adjIs.size() << std::endl;
    //for(std::vector<double*>::iterator it = adjIs.begin() ; it != adjIs.end(); ++it)    // For each unvisited adjacent variable
    //{
    //    std::cout << "adjacent node entry: " << (*it)[0] << ", " << (*it)[1] << ", " << (*it)[2] << std::endl;
    //}
    //std::cout << "p1" << std::endl;
    
    if(adjIs.empty()){
        //std::cout << "returning" << std::endl << std::endl;
        return;     // Nothing new to change. End of the recursion  
    }
        
    double rnd;
    bool varToBeChangedI;
    int varToBeChanged;
    for(std::vector<double*>::iterator it = adjIs.begin() ; it != adjIs.end(); ++it)    // For each unvisited adjacent variable
    {
        if((int)((*it)[0])!=changedVariable)
            varToBeChangedI=0;
        else
            varToBeChangedI=1;
        varToBeChanged=(int)((*it)[varToBeChangedI]);
        //std::cout << "varToBeChangedI: " << varToBeChangedI << std::endl;
        //std::cout << "varToBeChanged: " << varToBeChanged << std::endl;
        
        //std::cout << "size(*alreadyExploredVarIndices): " << (*alreadyExploredVarIndices).size() << std::endl;
        if((std::find((*alreadyExploredVarIndices).begin(), (*alreadyExploredVarIndices).end(), varToBeChanged) != (*alreadyExploredVarIndices).end())){ // if the next node to be analyzed has been already modified in previews for cycles
            //std::cout << "continue" << std::endl;
            continue;  // Then jump to the new node
        }
        rnd=RandomDouble(0,1);
        if(rnd<=((*it)[2])){    // Make the change
            //std::cout << "In if 1" << std::endl;
            //std::cout << "varToBeChanged: " << varToBeChanged << std::endl;
            //std::cout << "rockstate.Rocks[varToBeChanged].Valuable: " << rockstate.Rocks[varToBeChanged].Valuable << std::endl;
            //std::cout << "changedVariable: " << changedVariable << std::endl;
            //std::cout << "rockstate.Rocks[changedVariable].Valuable: " << rockstate.Rocks[changedVariable].Valuable << std::endl;
            
            rockstate.Rocks[varToBeChanged].Valuable = rockstate.Rocks[changedVariable].Valuable;   // With probab in [2] set adjacent equal to current node, otherwise leave it as it is
            //std::cout << "In if 2" << std::endl;
        }
        //std::cout << "New value of rock " << varToBeChanged << ": " << rockstate.Rocks[varToBeChanged].Valuable << std::endl;
        (*alreadyExploredVarIndices).push_back(varToBeChanged);
        //std::cout << "size(*alreadyExploredVarIndices): " << (*alreadyExploredVarIndices).size() << std::endl;
        PropagateChange(rockstate, varToBeChanged, stateVarRelationships, alreadyExploredVarIndices); // Recursive propagation
    }
    //std::cout << "End of for cycle: returning " << std::endl<< std::endl;
    */
}

// Notice: in case of prob=1 changes could be simply propagated in all the connected component
void VELOCITYREGULATION::PropagateChangeToConnectedComponent(STATE& state, int changedVariable, int newVal, 
        std::vector<double*>* stateVarRelationships) const
{
    /* To be implemented
    VELOCITYREGULATION_STATE& rockstate = safe_cast<VELOCITYREGULATION_STATE&>(state);  // State   
    std::vector<std::vector<int>*>* connComps = computeConnComp(stateVarRelationships, NumRocks);
    bool found=0;
    for(std::vector<std::vector<int>*>::iterator it = connComps->begin() ; it != connComps->end(); ++it){ // it=component
        // Check if changedVariable is in connected component it
        for(std::vector<int>::iterator it1 = (*it)->begin() ; it1 != (*it)->end(); ++it1){  // it1=node
            if((*it1)==changedVariable){
                found=1;
                break;
            }
        }
        if(found){
            for(std::vector<int>::iterator it1 = (*it)->begin() ; it1 != (*it)->end(); ++it1){  // change values of all nodes in the connected component
                rockstate.Rocks[(*it1)].Valuable=newVal;
            }
        }
    }
    */
}




std::vector<double*> VELOCITYREGULATION::FindUnexploredAdjacentVariableIndices(int currentVarIndex, 
        std::vector<double*>* stateVarRelationships, std::vector<int>* alreadyExploredVarIndices) const
{

    //std::cout << "VELOCITYREGULATION::FindUnexploredAdjacentVariableIndices (ACA)" << std::endl;
    int nRels=stateVarRelationships->size();
    std::vector<double*> adjIs;
    
    /* TO BE IMPLEMENTED
    for(int i=0; i<nRels; i++)
    {
        if((int)(*stateVarRelationships)[i][0]==currentVarIndex)   // Current node in first index
            if(std::find((*alreadyExploredVarIndices).begin(), (*alreadyExploredVarIndices).end(), (int)(*stateVarRelationships)[i][1]) == (*alreadyExploredVarIndices).end()) // If second node not already explored
                adjIs.push_back((*stateVarRelationships)[i]); // Add stateVarRelationships entry to adjIs
        
        if((int)(*stateVarRelationships)[i][1]==currentVarIndex)   // Current node in first index
            if(std::find((*alreadyExploredVarIndices).begin(), (*alreadyExploredVarIndices).end(), (int)(*stateVarRelationships)[i][0]) == (*alreadyExploredVarIndices).end()) // If second node not already explored
                adjIs.push_back((*stateVarRelationships)[i]); // Add stateVarRelationships entry to adjIs
        
    }
     */
    return adjIs;
}

double VELOCITYREGULATION::ComputeParticleProbability(STATE& particle, std::vector<double*>* stateVarRelationships) const {
    VELOCITYREGULATION_STATE& oastate = safe_cast<VELOCITYREGULATION_STATE&>(particle);  // State   
    double probab=1.0;
    
    int var0, var1, seg0Val, seg1Val;
    
    double potentialEqualValues, potentialDifferentValues, potential;
    for(std::vector<double*>::iterator it = stateVarRelationships->begin() ; it != stateVarRelationships->end(); ++it){ // For each edge of stateVarRelationships
        var0=(*it)[0]; // First segment
        var1=(*it)[1]; // Second segment

        potentialEqualValues=(*it)[2]/nDifficultyValues; // 00 or 11 or 22
        potentialDifferentValues=(1.0-(*it)[2])/6; // 6 is the number of possible different pairs of 3 numbers (01, 10, 02, 20, 12, 21) <- NB: TO BE CHECHED IF MORE THAN 3 DIFFICULTY VALUES IS USED
        // Apply the constraint to the state and compute the related potential
        seg0Val=oastate.segDifficulties[var0];
        seg1Val=oastate.segDifficulties[var1];
        if((seg0Val==seg1Val)){   // They are equal
            potential=potentialEqualValues;
        }
        else{   // They are different
            potential=potentialDifferentValues;
        }
        probab=probab*potential;
    }
    return probab;
}

void VELOCITYREGULATION::GetBeliefDistribution(const BELIEF_STATE &beliefState, std::vector<std::pair<int, int>> &beliefs) const
{
    std::unordered_map<int, int> dist;
    int id;

    for (int i = 0; i < beliefState.GetNumSamples(); i++)
    {
        const STATE* state = beliefState.GetSample(i);
        const VELOCITYREGULATION_STATE& bmState = safe_cast<const VELOCITYREGULATION_STATE&>(*state);

        // Compute state id
        id = 0;
        for (int j = 0; j < nSeg; j++)
        {
            id += bmState.segDifficulties[j]*(pow(3,nSeg-j-1));
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

void VELOCITYREGULATION::Id2State(const int id, std::vector<int> &state) const
{
    state.clear();

    std::string id_s = UTILS::to_base(id, 3);

    // zero padding
    while (id_s.size() < nSeg)
    {
        id_s = '0' + id_s;
    }
    // if (id_s.size() < nSeg)

    // std::string id_s = std::bitset<MAX_ID>(id).to_string().substr(MAX_ID - nSeg);

    // std::reverse(id_s.begin(), id_s.end());

    // std::cout << id_s << " " << id << std::endl;



    // for (size_t i = 0; i < nSeg; i++)
    for (size_t i = 0; i < id_s.size(); i++)
    {
        state.push_back((int) id_s[i] - '0');
    }

    // for (size_t i = 0; i < NumRocks; i++)
    // {
    //     std::cout << state[i] << " ";
    // }
    // std::cout << std::endl;
}

bool VELOCITYREGULATION::CheckDiscrepancy(STATE &state, std::vector<double*>* stateVarRelationships, VHISTORY &verboseHistory, std::pair<int, int> &nodes, std::vector<int> &already) const
{   
    // VELOCITYREGULATION_STATE& bmState = safe_cast<VELOCITYREGULATION_STATE&>(state);

    VELOCITYREGULATION_VHISTORY& vHistory = safe_cast<VELOCITYREGULATION_VHISTORY&>(verboseHistory);

    std::vector<int> segCollisions;

    for (size_t i = 0; i < vHistory.collisions.size(); i++)
    {
        int add;
        for (size_t j = 0; j < vHistory.collisions[i].size(); j++)
        {
            // Setting whole segment to not visited if any of its subsegments were not visited yet
            if (vHistory.collisions[i][j] == NOTVISITED)
            {
                add = NOTVISITED;
                break;
            }

            if (vHistory.collisions[i][j] > 0)
            {
                add = 1;
                break;
            }

            add = 0;
        }

        segCollisions.push_back(add);
        // :)
        // isCollided? segCollisions.push_back(1) : segCollisions.push_back(0);
    }

    // std::cout << " | ";
    // for (size_t i = 0; i < segCollisions.size(); i++)
    // {
    //     std::cout << segCollisions[i] << " ";
    // }
    // std::cout << std::endl;

    for (size_t i = 0; i < stateVarRelationships->size(); i++)
    {
        if (segCollisions[(*stateVarRelationships)[i][0]] != NOTVISITED && segCollisions[(*stateVarRelationships)[i][1]] != NOTVISITED)
        {
            if (segCollisions[(*stateVarRelationships)[i][0]] != segCollisions[(*stateVarRelationships)[i][1]])
            {
                int index = (int) pow(2, (*stateVarRelationships)[i][0]) + pow(2, (*stateVarRelationships)[i][1]);
                if (! std::count(already.begin(), already.end(), index))
                {
                    already.push_back(index);
                    nodes = std::make_pair((*stateVarRelationships)[i][0], (*stateVarRelationships)[i][1]);

                    return true;
                }
            }
        }
    }
    
    return false;
} 

bool VELOCITYREGULATION::CheckReplayDiscrepancy(STATE &state, VHISTORY &vHistory, VHISTORY &sVerbose, int depth) const
{
    return false;

    VELOCITYREGULATION_STATE& bmState = safe_cast<VELOCITYREGULATION_STATE&>(state);
    
    VELOCITYREGULATION_VHISTORY& verboseHistory = safe_cast<VELOCITYREGULATION_VHISTORY&>(vHistory);
    VELOCITYREGULATION_VHISTORY& stateVerbose = safe_cast<VELOCITYREGULATION_VHISTORY&>(sVerbose);

    // Save info of current particle, current episode
    // replayMap[state.id].push_back(stateVerbose.replayCollisions.back());

    // Once a segment is finished, find mode and used that value to delete particle if not in accordance with current verbose history

    if (stateVerbose.replayCollisions.back() != verboseHistory.replayCollisions[depth])
    {
        return true;
    }

    // if (depth == nSubSegs[replayStep])
    // {
        // int vals[3] = {0,0,0};
        // for (auto it = replayMap[state.id].begin(); it != replayMap[state.id].end(); it++)
        // {
        //     switch (*it)
        //     {
        //     case 0: vals[0]++; break;
        //     case 1: vals[1]++; break;
        //     case 2: vals[2]++; break;
            
        //     default: std::cerr << "Should not happen." << *it << std::endl; exit(-1); break;
        //     }
        // }
            
        // Assume the overall observation is zero
        // int segment_obsevation = 0;
        // for (size_t i = 1; i < 3; i++)
        // {
        //     // If same counter, assume worse
        //     if (vals[i] >= vals[i-1])
        //     {
        //         segment_obsevation = i;
        //     }
        // }

        // Sum collisions 



    //     int historyCollisions = 0;
    //     int currentCollisions = 0;
    //     for (size_t i = 0; i < stateVerbose.collisions; i++)
    //     {
    //         historyCollisions += verboseHistory.collisions[replayStep][i];
    //     }

    //     for (size_t i = 0; i < verboseHistory.collisions[replayStep].size(); i++)
    //     {
    //         historyCollisions += verboseHistory.collisions[replayStep][i];
    //     }
        

    //     if (historyCollisions > 1 && segment_obsevation != 2)
    //     {
    //         return true;
    //     }

    //     // Are the vectors erased?
    //     replayMap.clear();
    //     replayStep++;
    // }
    
    return false;
}