#include <Core/array.h>
#include <Ors/ors.h>

template<class NodeT>
mlr::Array<NodeT*> backtrack(const mlr::Array<NodeT*>& T, const NodeT *leaf);
struct State;
struct Action;
struct SearchNode;
extern Action& NoAction;
extern SearchNode& NoSearchNode;

//===========================================================================

enum ActionPredicate : uint{ create_=0, break_, _N_ActionPredicate };
const char* actionPredicateString(ActionPredicate);

//===========================================================================

struct Domain{
  uint numObjects();
  void getInitialState(State& s);
};

//===========================================================================

struct Action{
  ActionPredicate a;
  mlr::String p;
  uint i,j;
  Action():a(create_), i(0), j(0){}
  void setRandom(uint n);
  void write(ostream& os) const{ os <<actionPredicateString(a) <<p <<' ' <<i <<' ' <<j; }
};
stdOutPipe(Action)
typedef mlr::Array<Action*> ActionL_;


//===========================================================================

struct Pose{
  mlr::Transformation mean;
  mlr::Vector min,max; //< min and max positions (translational freedom)
  mlr::Vector rotRange; //< rotational pose range
  void write(ostream& os) const{ os <<mean <<" [" <<min <<',' <<max <<"] [" <<rotRange <<"]"; }
};
stdOutPipe(Pose)
typedef mlr::Array<Pose*> PoseL;

//===========================================================================

struct State{
  Graph G;
  Action preAction;

  void compControllable();
  void expandReachable();
  bool testAction(const Action& a, bool apply);
  void write(ostream& os) const;
};
stdOutPipe(State)

//===========================================================================

struct SearchNode;
typedef mlr::Array<SearchNode*> SearchNodeL;

struct SearchNode{
  SearchNodeL* container;
  const SearchNode* preNode;
  State state;

  SearchNode(SearchNodeL& container_);
  SearchNode(const SearchNode& preNode_, const Action& preAction);

//  ActionL_ getFeasibleActions();
  Action getRandomFeasibleAction();
  const State& getState(){ return state; }
//  void append(const State& s);
  const Action& getPreAction(){ return state.preAction; }
  void write(ostream& os) const{ os <<state; }
};
stdOutPipe(SearchNode)

//===========================================================================






template<class NodeT>
mlr::Array<NodeT*> backtrack(const mlr::Array<NodeT*>& T, const NodeT *leaf){
  mlr::Array<NodeT*> path;
  path.memMove=true;
  const NodeT *n = leaf;
  for(;;){
    path.insert(0,(NodeT*)n);
    n = n->preNode;
    if(!n) break;
  }
  return path;
}
