#pragma once

#include <MCTS/env_marc.h>
//#include <FOL/fol.h>
#include <Core/array.h>

struct Node;
struct Graph;
typedef MT::Array<Node*> NodeL;

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
    virtual size_t get_hash() const override {
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
    virtual size_t get_hash() const override {
      return std::hash<int>()(id);
    }
  };
  struct State:SAO {};

  uint T_step; ///< discrete "time": decision steps so far
  double T_real;///< real time so far;
  double R_total;
  uint Ndecisions;
  bool deadEnd, successEnd;
  Graph& KB;     ///< current knowledge base
  Graph *start_state; ///< the start-state within the KB (is a subgraph item of KB)
  Graph *state; ///< the dynamic/fluent state within the KB (is a subgraph item of KB, created within the constructor)
  NodeL decisionRules;  ///< rules withing the KB (each is a subgraph item of the KB)
  NodeL constants;///< constants withing the KB (each is an item of the KB)
  Graph *tmp;   ///< a tmp subgraph of the KB (private, created within the constructor)
  Graph *terminal; //TODO: replace
  Node *Terminate_keyword;
  int verbose;
  ofstream fil;

  FOL_World(const char* KB_file);
  virtual ~FOL_World();

  virtual std::pair<Handle, double> transition(const Handle& action);
  virtual const std::vector<Handle> get_actions();
  virtual const Handle get_state();
  virtual bool is_terminal_state() const;
  virtual void make_current_state_default() override;
  virtual void reset_state();

  virtual bool get_info(InfoTag tag) const;
  virtual double get_info_value(InfoTag tag) const;
};
