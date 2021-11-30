#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <stdlib.h>
#include <math.h>
#include <assert.h>
#include "coord.h"
#include "memorypool.h"
#include <algorithm>
#include <iostream>
#include <random>
#include <fstream>

#define pLargeInteger 1000000
#define pInfinity 1e+10
#define pTiny 1e-10

#ifdef DEBUG
#define safe_cast dynamic_cast
#else
#define safe_cast static_cast
#endif

#define RNDGENINTERFACE

namespace UTILS
{

// https://stackoverflow.com/questions/8870121/c-template-for-conversion-between-decimal-and-arbitrary-base
inline std::string to_base(unsigned int n, unsigned int base)
{
    static const char *alphabet = "0123456789ABCDEFGHI";
    std::string result;
    while(n) { result += alphabet[n % base]; n /= base; }
    return std::string(result.rbegin(), result.rend());
}


class RND
{
public:
    bool verbose;
    
    RND(int seed, int runs, std::string filepath, bool verbose=true)
    :   cnt(0),
        checkpoint(0),
        run(0),
        seed(seed),
        runs(runs),
        engine(seed),
        verbose(verbose),
        filepath(filepath)
    {
        seeds.push_back(seed);
        for (size_t i = 1; i < runs; i++)
        {
            seeds.push_back(seed + i);
        }
    }

    void SetCheckpoint()
    {
        std::cout << "Starting random generator checkpoint" << std::endl;
        checkpoint = cnt;
    }

    void RetrieveCheckpoint()
    {
        std::cout << "Retrieving random generator checkpoint" << std::endl;
        
        engine.seed(seed);
        engine.discard(checkpoint);
        cnt = checkpoint;
    }

    void NextSeed()
    {
        run++;
        std::cout << "Setting new seed: " << seed << " (" << run+1 << "/" << runs << ")" << std::endl;
        
        // Possible segmentation fault :)
        seed = seeds[run];

        engine.seed(seed);
        cnt = 0;
        checkpoint = 0;
    }

    void Log()
    {
        std::ofstream tmp;
        tmp.open (filepath, std::ios::app);
        tmp << Random(100) << ", " << Random(100) << ", " << Random(100) << std::endl;
        tmp.close();
    }

    void PrintSeed()
    {
        std::ofstream tmp;
        tmp.open (filepath, std::ios::app);
        tmp << run+1 << ": " << seeds[run] << std::endl;
        tmp << "-----------------------------------" << std::endl;
        tmp.close();
    }

    // -------------- GET RANDOM ---------------------------
    int Random(int max)
    {
        cnt++;
        return engine() % max;
    }

    int Random(int min, int max)
    {
        cnt++;
        return engine() % (max - min) + min;
    }

    double RandomDouble(double min, double max)
    {
        cnt++;
        return (double) engine() / RAND_MAX * (max - min) + min;
    }

    bool Bernoulli(double p)
    {
        cnt++;
        return engine() < p * RAND_MAX;
    }
    // -------------- GET RANDOM ---------------------------

private:
    unsigned long long cnt;
    unsigned long long checkpoint;
    int run;
    int seed;
    int runs;
    std::default_random_engine engine;

    std::string filepath;

    std::vector<int> seeds;
};

inline int Sign(int x)
{
    return (x > 0) - (x < 0);
}

inline int Random(int max)
{
    return rand() % max;
}

inline int Random(int min, int max)
{
    return rand() % (max - min) + min;
}

inline double RandomDouble(double min, double max)
{
    return (double) rand() / RAND_MAX * (max - min) + min;
}

inline void RandomSeed(int seed)
{
    srand(seed);
}

inline bool Bernoulli(double p)
{
    return rand() < p * RAND_MAX;
}

inline bool Near(double x, double y, double tol)
{
    return fabs(x - y) <= tol;
}

inline bool CheckFlag(int flags, int bit) { return (flags & (1 << bit)) != 0; }

inline void SetFlag(int& flags, int bit) { flags = (flags | (1 << bit)); }

template<class T>
inline bool Contains(std::vector<T>& vec, const T& item)
{
    return std::find(vec.begin(), vec.end(), item) != vec.end();
}

void UnitTest();

// ------------------------------- ADDED -------------------------------------------

inline void visit(int nodeI, bool* markedNodes, int* nMarkedNodes, std::vector<double*>* stateVarRelationships, std::vector<int>* connComp)
{
    // Mark nodeI
    // markedNodes[nodeI]=1;
    //std::cout << "In visit" << std::endl;
    connComp->push_back(nodeI);
    //std::cout << "P1" << std::endl;
    (*nMarkedNodes)++;

    // Search the first instance of node nodeI in stateVarRelationships
    std::vector<int> adjNodes; // Adjacents to nodeI
    for(std::vector<double*>::iterator it = stateVarRelationships->begin() ; it != stateVarRelationships->end(); ++it){

        if((*it)[0] == nodeI && markedNodes[(int)((*it)[1])] == 0)
        {
            // 0=nodeI, 1=not marked
            adjNodes.push_back((int)((*it)[1]));
            markedNodes[(int)((*it)[1])]=1;
        }

        if((*it)[1] == nodeI && markedNodes[(int)((*it)[0])] == 0)
        {
            // 1=nodeI, 0=not marked
            adjNodes.push_back((int)((*it)[0]));
            markedNodes[(int)((*it)[0])]=1;
        }
    }

    //std::cout << "P2" << std::endl;
    // Here adjNodes contains all adjacent not marked of nodeI
    for(std::vector<int>::iterator it = adjNodes.begin() ; it != adjNodes.end(); ++it){
        visit((*it), markedNodes, nMarkedNodes, stateVarRelationships, connComp);
    }
}

inline std::vector<std::vector<int>*>* computeConnComp(std::vector<double*>* stateVarRelationships, int nNodes)
{
    //std::cout << "In computeConnComp" << std::endl;
    int nConnComp = 0;
    bool* markedNodes = new bool[nNodes];
    for(int i = 0; i < nNodes; i++){
        markedNodes[i] = 0;
    }
    std::vector<std::vector<int>*>* connComps = new std::vector<std::vector<int>*>();
    int nMarkedNodes = 0;
    int nodeI;

    while(nMarkedNodes<nNodes)
    {
        nConnComp++;    // Every cycle is a component
        connComps->push_back(new std::vector<int>());    // Initialize vector
        nodeI = 0;

        while(markedNodes[nodeI] == 1) // Find the first node not visited
            nodeI++;

        markedNodes[nodeI] = 1;
        visit(nodeI,markedNodes,&nMarkedNodes,stateVarRelationships,(*connComps)[nConnComp-1]); // Visit it and its connected component
    }

    return connComps;
}

inline void printConnectedComponents(std::vector<std::vector<int>*>* connComps)
{
    std::cout << "Printing connected components " << std::endl;
    int cn = 0;

    std::cout << connComps->size() << std::endl;
    std::cout << connComps[0].size() << std::endl;

    for(std::vector<std::vector<int>*>::iterator it = connComps->begin() ; it != connComps->end(); ++it){
        std::cout << "Component: " << cn << std::endl;

        for(std::vector<int>::iterator it1 = (*it)->begin() ; it1 != (*it)->end(); ++it1){
            std::cout << (*it1) << " ";
        }

        cn++;
        std::cout << std::endl;
    }


}
// ------------------------------- ADDED -------------------------------------------

}

#endif // UTILS_H
