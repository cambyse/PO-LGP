#include "mcts.h"
#include <math.h>
#include <tuple>
#include <algorithm>

using namespace std;


//-----------------------------------------------------------------------------

MCTS::PARAMS::PARAMS()
:   Verbose(0),
    MaxDepth(1000),
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

MCTS::MCTS(std::shared_ptr<AbstractEnvironment> world, const PARAMS& params)
:   World(world),
    Params(params),
    TreeDepth(0)
{
    /*/
    VNODE::NumChildren = Simulator.GetNumActions();
    //QNODE::NumChildren = Simulator.GetNumObservations();

    STATE * state = Simulator.CreateStartState();
    Root = ExpandNode(state);
    Root->MDPState.clear();
    Root->MDPState.push_back(state); //starting state (it is random)

    cout<<"simulation "<<Params.NumSimulations <<endl;
    cout<<"MaxDepth "<<Params.MaxDepth <<endl<<endl;
    /*/
    //std::cout<<"simulation "<<Params.NumSimulations <<std::endl;
    //std::cout<<"MaxDepth "<<Params.MaxDepth <<endl<<std::endl;


    Actions = World->get_actions();
    VNODE::NumChildren  = Actions.size();
    //STATE * state = 0;// Simulator.CreateStartState();
    Root = ExpandNode();
    //cout<< Actions[0] <<endl;

    // make sure we can to standard rollouts to terminal state
    assert(World->has_terminal_state());
    InitFastUCB(Params.ExplorationConstant);
}

MCTS::~MCTS()
{
    VNODE::Free(Root);
    VNODE::FreeAll();
}
/*/
bool MCTS::Update(int action, int observation, double reward)
{
    History.Add(action, observation);
        // Find matching vnode from the rest of the tree
    QNODE& qnode = Root->Child(action);
    VNODE* vnode = qnode.ChildviaObs(observation);

    STATE* state = Simulator.CreateStartState(observation);

    // Delete old tree and create new root
    VNODE::Free(Root, Simulator);
    VNODE* newRoot = ExpandNode(state);
    //newRoot->Beliefs() = beliefs;
    newRoot->MDPState.clear();
    newRoot->MDPState.push_back(state);


    Root = newRoot;

    return true;
}
/*/
int MCTS::SelectAction()
{

    VNODE::Free(Root);
    Root = ExpandNode();


    UCTSearch();

    return GreedyUCB(Root, false);
}

void MCTS::RolloutSearch()
{
    /*/
	std::vector<double> totals(Simulator.GetNumActions(), 0.0);
	int historyDepth = History.Size();
    std::vector<int> legal;
    legal.clear();
    for(int i=0;i<Simulator.GetNumActions();i++)
        legal.push_back(i);

    random_shuffle(legal.begin(), legal.end());

	for (int i = 0; i < Params.NumSimulations; i++)
	{
        int action = legal[i % Simulator.GetNumActions()];
        STATE* state = Simulator.Copy(*Root->MDPState[0]);
        //Simulator.Validate(*state);

		int observation;
		double immediateReward, delayedReward, totalReward;
		bool terminal = Simulator.Step(*state, action, observation, immediateReward);

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
    /*/
}
/*/
/*/
void MCTS::UCTSearch()
{

    for (int n = 0; n < Params.NumSimulations; n++)
    {
        //cout<<" ROLLOUT Starting: "<<endl;
        World->reset_state();
        //AbstractEnvironment::observation_handle_t

        TreeDepth = 0;
        PeakTreeDepth = 0;
        double totalReward = SimulateV(Root);        
    }

}


double MCTS::SimulateV(VNODE *vnode)
{

    int action = GreedyUCB(vnode, true);

    PeakTreeDepth = TreeDepth;

    if (TreeDepth >= Params.MaxDepth) // search horizon reached
        return 0;

    //if (TreeDepth == 1)
    //    AddSample(vnode, state);

    QNODE& qnode = vnode->Child(action);
    double totalReward = SimulateQ(qnode, action);
    vnode->Value.Add(totalReward);
    //AddRave(vnode, totalReward);
    return totalReward;
}

double MCTS::SimulateQ(QNODE &qnode, int action)
{

    //int observation;
    double immediateReward, delayedReward = 0;

    //if (Simulator.HasAlpha())
    //    Simulator.UpdateAlpha(qnode, state);
    AbstractEnvironment::observation_reward_pair_t SAOR = World->transition(Actions[action]);


    immediateReward = std::get<1>(SAOR);
    bool terminal = World->is_terminal_state();//= Simulator.Step(state, action, observation, immediateReward);
    //assert(observation >= 0 && observation < Simulator.GetNumObservations());

    int index = qnode.findIndex(std::get<0>(SAOR));
    if(index<0){
       qnode.Add(std::get<0>(SAOR));
       index = qnode.NumChildren-1;
    }

    VNODE*& vnode = qnode.Child(index);

    //VNODE*& vnode = qnode.Child(observation);
    if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount){
        vnode = ExpandNode();
    }

    if (!terminal)
    {
        TreeDepth++;
        if (vnode)
            delayedReward = SimulateV(vnode);
        else
            delayedReward = Rollout();
        TreeDepth--;
    }

    double totalReward = immediateReward + GetDiscount() * delayedReward;
    qnode.Value.Add(totalReward);
    return totalReward;
}


VNODE* MCTS::ExpandNode()
{
    VNODE* vnode = VNODE::Create();
    vnode->Value.Set(0, 0);

    return vnode;
}


int MCTS::GreedyUCB(VNODE* vnode, bool ucb) const
{
    static vector<int> besta;
    besta.clear();
    double bestq = -Infinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);

    for (int action = 0; action < Actions.size(); action++)
    {
        double q;
        int n;

        QNODE& qnode = vnode->Child(action);
        q = qnode.Value.GetValue();
        n = qnode.Value.GetCount();


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
    return besta[Random(besta.size())];
}

double MCTS::Rollout()
{

    double totalReward = 0.0;
    double discount = 1.0;
    bool terminal = false;
    int numSteps;
    for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
    {
        int observation;
        double reward;

        int action = Random(Actions.size());
        //std::pair<MCTS_Environment::Handle, double> SAOR = World.transition(Actions[action]);
        AbstractEnvironment::observation_reward_pair_t SAOR = World->transition(Actions[action]);
        reward = std::get<1>(SAOR);
        terminal = World->is_terminal_state();

        totalReward += reward * discount;
        discount *= GetDiscount();
    }


    return totalReward;
}

double MCTS::UCB[UCB_N][UCB_n];
bool MCTS::InitialisedFastUCB = true;

void MCTS::InitFastUCB(double exploration)
{
    //cout << "Initialising fast UCB table... ";
    for (int N = 0; N < UCB_N; ++N)
        for (int n = 0; n < UCB_n; ++n)
            if (n == 0)
                UCB[N][n] = Infinity;
            else
                UCB[N][n] = exploration * sqrt(log(N + 1) / n);
    //cout << "done" << endl;
    InitialisedFastUCB = true;
}

inline double MCTS::FastUCB(int N, int n, double logN) const
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
        return UCB[N][n];

    if (n == 0)
        return Infinity;
    else
        return Params.ExplorationConstant * sqrt(logN / n);
}

int MCTS::Random(int max) const
{
    return rand() % max;
}

