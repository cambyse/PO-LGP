#pragma once

#include <MCTS/env_marc.h>
//#include <FOL/fol.h>

struct Node;
struct Graph;
typedef MT::Array<Node*> NodeL;

struct FOL_World:MCTS_Environment{
  struct Decision:SAO{
    bool waitDecision;
    Node *rule;
    NodeL substitution;
    Decision(bool waitDecision, Node *rule, const NodeL& substitution):waitDecision(waitDecision),rule(rule), substitution(substitution){}
    bool operator==(const SAO & other) const{ NIY; }
    void write(ostream&) const;
  };


  uint T_step; ///< discrete "time": decision steps so far
  double T_real;///< real time so far;
  Graph& KB;     ///< current knowledge base
  Graph *start_state; ///< the start-state within the KB (is a subgraph item of KB)
  Graph *state; ///< the dynamic/fluent state within the KB (is a subgraph item of KB, created within the constructor)
  NodeL rules;  ///< rules withing the KB (each is a subgraph item of the KB)
  NodeL constants;///< constants withing the KB (each is an item of the KB)
  Graph *tmp;   ///< a tmp subgraph of the KB (private, created within the constructor)
  Graph *terminal; //TODO: replace
  Node *Terminate_keyword;
  int verbose;


  FOL_World(const char* KB_file);
  virtual ~FOL_World() = default;

  virtual std::pair<Handle, double> transition(const Handle& action);
  virtual const std::vector<Handle> get_actions();
  virtual const Handle get_state();
  virtual bool is_terminal_state() const;
  virtual double get_terminal_reward() const;
  virtual void set_state(const Handle& state);
  virtual void reset_state();

  virtual bool get_info(InfoTag tag) const;
  virtual double get_info_value(InfoTag tag) const;
};
