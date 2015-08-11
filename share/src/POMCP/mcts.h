#ifndef MCTS_H
#define MCTS_H

#include "node.h"
#include "statistic.h"
#include "../../include/MCTS_Environment/AbstractEnvironment.h"

class MCTS
{
public:

    struct PARAMS
    {
        PARAMS();

        int Verbose;
        int MaxDepth;
        int NumSimulations;
        int NumStartStates;
        bool UseTransforms;
        int NumTransforms;
        int MaxAttempts;
        int ExpandCount;
        double ExplorationConstant;
        bool UseRave;
        double RaveDiscount;
        double RaveConstant;
        bool DisableTree;
    };

   // MCTS(const SIMULATOR& simulator, const PARAMS& params);
    MCTS( std::shared_ptr<AbstractEnvironment> world, const PARAMS& params);
    ~MCTS();

    int SelectAction();
    //bool Update(int action, int observation, double reward);

    void UCTSearch();
    void RolloutSearch();

    double Rollout();

    static void InitFastUCB(double exploration);

    void SetDiscount(double discount){Discount = discount;}
    double GetDiscount()             {return Discount;}

private:

    double Discount;

    //const SIMULATOR& Simulator;
    //MCTS_Environment& World;
    std::shared_ptr<AbstractEnvironment> World;

    int TreeDepth, PeakTreeDepth;
    PARAMS Params;
    VNODE* Root;

    int GreedyUCB(VNODE* vnode, bool ucb) const;
    //int SelectRandom() const;
    double SimulateV(VNODE* vnode);
    double SimulateQ(QNODE& qnode, int action);
    int Random(int max) const;


    VNODE* ExpandNode();

    // Fast lookup table for UCB
    static const int UCB_N = 10000, UCB_n = 100;
    static double UCB[UCB_N][UCB_n];
    static bool InitialisedFastUCB;

    double FastUCB(int N, int n, double logN) const;

    std::vector<AbstractEnvironment::action_handle_t> Actions;// = get_actions();


};

#endif // MCTS_H
