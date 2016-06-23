#pragma once

#include <MCTS/environment.h>
#include <Core/array.h>
#include <Core/graph.h>

struct FOL_World:MCTS_Environment{
  struct Decision:SAO{
    bool waitDecision;
    Node *rule;
    NodeL substitution;
    int id;
    Decision(bool waitDecision, Node *rule, const NodeL& substitution, int id):waitDecision(waitDecision),rule(rule), substitution(substitution), id(id) {}
    bool operator==(const SAO & other) const {
      auto decision = dynamic_cast<const Decision *>(&other);
      if(decision==nullptr) return false;
      if(decision->waitDecision!=waitDecision) return false;
      if(decision->rule!=rule) return false;
      if(decision->substitution!=substitution) return false;
      return true;
    }
    void write(ostream&) const;
    virtual size_t get_hash() const {
      return std::hash<int>()(id);
    }
  };
  struct Observation:SAO{
    int id;
    Observation(int id): id(id) {}
    bool operator==(const SAO & other) const {
      auto ob = dynamic_cast<const Observation *>(&other);
      return ob!=nullptr && ob->id==id;
    }
    void write(ostream& os) const { os <<id; }
    virtual size_t get_hash() const {
      return std::hash<int>()(id);
    }
  };
  struct State:SAO {};

  uint T_step, start_T_step; ///< discrete "time": decision steps so far
  double T_real, start_T_real;///< real time so far;
  double R_total;

  //-- parameters
  bool hasWait;
  double gamma, stepCost, timeCost, deadEndCost;

  bool deadEnd, successEnd;
  Graph KB;     ///< current knowledge base
  Graph *start_state; ///< the start-state within the KB (is a subgraph item of KB)
  Graph *state; ///< the dynamic/fluent state within the KB (is a subgraph item of KB, created within the constructor)
  NodeL worldRules;     ///< rules within the KB (each is a subgraph item of the KB)
  NodeL decisionRules;  ///< rules within the KB (each is a subgraph item of the KB)
  Node *lastDecisionInState; ///< the literal that represents the last decision in the state
  Graph *rewardFct; ///< the reward function within the KB (is a subgraph item of KB)
  Graph *tmp;   ///< a tmp subgraph of the KB (private, created within the constructor)
  Node *Terminate_keyword, *Quit_keyword, *Wait_keyword, *Quit_literal;
  int verbose;
  int verbFil;
  ofstream fil;
  bool generateStateTree;

  double lastStepDuration;
  double lastStepProbability;
  long count;

  FOL_World();
  FOL_World(istream& fil);
  virtual ~FOL_World();
  void init(istream& fil);

  virtual std::pair<Handle, double> transition(const Handle& action);
  virtual const std::vector<Handle> get_actions();
  virtual const Handle get_state();
  virtual bool is_terminal_state() const;
  virtual void make_current_state_default();
  virtual void reset_state();

  virtual bool get_info(InfoTag tag) const;
  virtual double get_info_value(InfoTag tag) const;
  void write_state(ostream&);
  void set_state(mlr::String&);

  //-- helpers
  void addFact(const StringA& symbols);
  void addAgent(const char* name);
  void addObject(const char* name);

  //-- internal access
  Graph* getState();
  void setState(Graph*);

  void write(std::ostream& os) const{ os <<KB; }
};
stdOutPipe(FOL_World)
