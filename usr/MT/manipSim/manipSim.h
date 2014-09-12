#include <Core/array.h>
#include <Ors/ors.h>

template<class NodeT>
MT::Array<NodeT*> backtrack(const MT::Array<NodeT*>& T, const NodeT *leaf);
struct State;
struct Action;
struct SearchNode;
extern Action& NoAction;
extern SearchNode& NoSearchNode;

//===========================================================================

enum Predicate : uint{ isFree=0, rigid, trans2DPhi, noPredicate_, _N_Predicate };
const char* predicateString(Predicate);

enum ActionPredicate : uint{ create_=0, break_, noActionPredicate_ , _N_ActionPredicate };
const char* actionPredicateString(ActionPredicate);

//===========================================================================

struct Domain{
  uint numObjects();
  bool isDirectlyControllable(uint i);
  void getInitialState(State& s);
  void getNewState(State& new_s, const State& s, const Action& a);
};

//===========================================================================
struct Relation{
  Predicate p;
  uint i,j;
  Relation(Predicate _p, uint _i, uint _j):p(_p), i(_i), j(_i){}
  void write(ostream& os) const{ os <<predicateString(p) <<' ' <<i <<' ' <<j; }
};
stdOutPipe(Relation)
typedef MT::Array<Relation*> RelationL;

//===========================================================================

struct Action{
  ActionPredicate a;
  Predicate p;
  uint i,j;
  Action():a(noActionPredicate_), p(noPredicate_), i(0), j(0){}
  void write(ostream& os) const{ os <<actionPredicateString(a) <<predicateString(p) <<' ' <<i <<' ' <<j; }
};
stdOutPipe(Action)
typedef MT::Array<Action*> ActionL_;


//===========================================================================

struct Pose{
  ors::Transformation mean;
  ors::Vector min,max; //< min and max positions (translational freedom)
  ors::Vector rotRange; //< rotational pose range
  void write(ostream& os) const{ os <<mean <<" [" <<min <<',' <<max <<"] [" <<rotRange <<"]"; }
};
stdOutPipe(Pose)
typedef MT::Array<Pose*> PoseL;

//===========================================================================

struct State{
  RelationL R;
  PoseL P;
  boolA controllable;
  KeyValueGraph G;
  Action preAction;

  void compControllable();
  void expandReachable();
  void write(ostream& os) const;
};
stdOutPipe(State)

//===========================================================================

struct SearchNode;
typedef MT::Array<SearchNode*> SearchNodeL;

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
MT::Array<NodeT*> backtrack(const MT::Array<NodeT*>& T, const NodeT *leaf){
  MT::Array<NodeT*> path;
  const NodeT *n = leaf;
  for(;;){
    path.insert(0,(NodeT*)n);
    n = n->preNode;
    if(!n) break;
  }
  return path;
}
