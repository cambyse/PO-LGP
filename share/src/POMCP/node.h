#ifndef NODE_H
#define NODE_H

//#include "beliefstate.h"
//#include "utils.h"
#include <iostream>
#include "../MCTS/env_marc.h"
#include "memorypool.h"
#include "statistic.h"

class HISTORY;
class QNODE;
class VNODE;

//-----------------------------------------------------------------------------
// Efficient computation of value from alpha vectors
// Only used for explicit POMDPs
struct ALPHA
{
    std::vector<double> AlphaSum;
    double MaxValue;
};

//-----------------------------------------------------------------------------

template<class COUNT>
class VALUE
{
public:

    void Set(double count, double value)
    {
        Count = count;
        Total = value * count;
    }

    void Add(double totalReward)
    {
        Count += 1.0;
        Total += totalReward;
    }

    void Add(double totalReward, COUNT weight)
    {
        Count += weight;
        Total += totalReward * weight;
    }

    double GetValue() const
    {
        return Count == 0 ? Total : Total / Count;
    }

    COUNT GetCount() const
    {
        return Count;
    }

private:

    COUNT Count;
    double Total;
};

//-----------------------------------------------------------------------------

class QNODE
{
public:

    VALUE<int> Value;
    VALUE<double> AMAF;

    void Initialise();

    VNODE*& Child(int index) { return Children[index]; }
    VNODE* Child(int index) const { return Children[index]; }


    ALPHA& Alpha() { return AlphaData; }
    const ALPHA& Alpha() const { return AlphaData; }

    void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;


    //for lazy initialization
    int NumChildren;
    int findIndex(MCTS_Environment::Handle& temp);
    int findIndex(MCTS_Environment::Handle& temp) const;

    void Add(MCTS_Environment::Handle& temp);
        ///////

private:

    std::vector<VNODE*> Children;
    std::vector<MCTS_Environment::Handle> Observations;//match each child with an observation
    ALPHA AlphaData;

friend class VNODE;
};

//-----------------------------------------------------------------------------

class VNODE : public MEMORY_OBJECT {

public:

    VALUE<int> Value;

    void Initialise();
    static VNODE* Create();
    static void Free(VNODE* vnode);
    static void FreeAll();

    QNODE& Child(int c) { return Children[c]; }
    const QNODE& Child(int c) const { return Children[c]; }
    //BELIEF_STATE& Beliefs() { return BeliefState; }
    //const BELIEF_STATE& Beliefs() const { return BeliefState; }

    void SetChildren(int count, double value);

    void DisplayValue(HISTORY& history, int maxDepth, std::ostream& ostr) const;
    void DisplayPolicy(HISTORY& history, int maxDepth, std::ostream& ostr) const;

    static int NumChildren;


private:

    std::vector<QNODE> Children;
    static MEMORY_POOL<VNODE> VNodePool;
    //BELIEF_STATE BeliefState;


};

#endif // NODE_H
