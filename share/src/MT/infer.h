/*  ---------------------------------------------------------------------
    Copyright 2012 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


/* Copyright (C) 2000, 2006  Marc Toussaint (mtoussai@inf.ed.ac.uk)
under the terms of the GNU LGPL (http://www.gnu.org/copyleft/lesser.html)
see the `util.h' file for a full copyright statement  */

#ifndef MT_inference_h
#define MT_inference_h

#include "array.h"
#include "util.h"
#include "keyValueGraph.h"
#include <map>
//#include <set>



//===========================================================================
//
// basic data structures: variables, factors, message pairs
//

/* Note: since variables, factors and message pairs know how they are
   linked (they store lists of adjoint objects), these types are
   already sufficient to define arbitrary models. The factor graph
   type below does nothing more than storing all variables, factors
   and messages together and simplify building connected models
   automatically */

namespace infer {
struct Variable;
struct Factor;
struct MessagePair;

//simple ``list'' types
typedef MT::Array<Variable*> VariableList;
typedef MT::Array<Factor*>   FactorList;
typedef MT::Array<MessagePair*> MessagePairList;

enum SpecialFactorType { NONE=0, AND, OR, WTA };

extern uint VarCount;
}

namespace infer {
/** a discrete random variable */
struct Variable {
  //core defining properties
  uint id;          ///< unique identifyer
  uint dim;         ///< cardinality
  
  // auxilliary & connectivity
  MT::String name;  ///< up to you...
  infer::FactorList factors;    ///< each variable knows all factors it is part of
  MessagePairList messages;  ///< each variable knows all the messages it directly connects to
  KeyValueGraph ats; //any convenience information (e.g. for dot);
  
  Variable();
  Variable(uint _dim, const char *_name);
  Variable(uint _dim, const char *_name, uint _id);
  ~Variable();
  operator uint() const { return id; } //TODO: REMOVE
  void write(ostream& os = std::cout) const;
};
}

namespace infer {
/** a factor (=probability table) over a tuple of variable; a list of
    factors readily defines a proper factored joint distribution

    Probability tables are multi-dimensional arrays (tensors), per
    variable a separate dimension (boolean variables: first false,
    then true). */
struct Factor {
  //core defining properties
  uintA varIds; ///< f=f(x_1, x_3, x_7) => id=[1, 3, 7]; array of variables (their id's) this factor depends on
  uintA dim;    ///< f=f(x_1, x_3, x_7) => dim=[dim(x_1), dim(x_3), dim(x_7)];  array of dimensionalities of the variables this factor depends on
  arr P;        ///< the (probability) table
  double logP;  ///< the log-scaling of the table, such that true_factor = exp(logP) * P
  
  // auxilliary & connectivity
  MT::String name;
  SpecialFactorType specialType;
  infer::VariableList variables;
  infer::FactorList factors;
  MessagePairList messages;          ///< each factor knows all the msg_pairs it connects to
  KeyValueGraph ats; //any convenience information (e.g. for dot);
  
  Factor();
  ~Factor();
  Factor(const infer::VariableList& variables, const char *_name=NULL);
  Factor(const infer::VariableList& variables, const arr& q, const char *_name=NULL);
  void init(const infer::VariableList& variables);
  void relinkTo(const infer::VariableList& variables);
  void operator=(const infer::Factor& q);
  void setP(const arr& q);           ///< f(x) = q(x)
  void setText(const char* text);    ///< f(x) = q(x)
  void setOne();                     ///< f(x) = 1
  void setUniform();                 ///< sum(x_i) f(x_i) = 1  only if factor.P is set by hand (as a conditional)
  void setRandom();                  ///< randomize f (e.g., for automatic testing)
  void setEvidence(uint e);          ///< f(e) = 1,  f(x!=e) = 0
  bool operator==(infer::Factor& q);        ///< check if f==q (also checks for logP)
  void getP(arr& p) const;           ///< p = exp(logP)*P
  void normalize(){ lognormScale(P, logP); logP=0.; }
  void write(std::ostream& os = std::cout, bool brief=false) const;
  void writeNice(std::ostream& os = std::cout) const;
  void writeExtremelyNice(std::ostream& os = std::cout) const;
  
  void makeLogZero(){ P *= ::exp(logP); logP=0.; }
  
  double entry(uint i);              ///< returns logP * P.elem(i)
  void checkCondNormalization(uint left=1, double tol=1e-10);
  uint numberNonZeroEntries();
};
}

namespace infer {
/** a pair of messages (fwd and bwd) which connects two
    factors. Depending on the context this can be a separator (JTA), a
    link from factor to a single-variable-factor (bi-partite factor
    graph), or a links between arbitrary factors (loopy BP) */
struct MessagePair {
  //core defining properties
  infer::Factor m12, m21;      ///< the forward and backward message
  infer::Factor   *f1, *f2;    ///< the first and second factor it is attached to (if at all..)
  infer::Variable *v1, *v2;    ///< the first and second variable it is attached to (if at all..)
  infer::Factor   *v_to_v_fac; ///< in case of variable-to-variable message: the factor it is one-to-one associated with
  infer::VariableList variables;       ///< the variables the messages are defined over
  
  MessagePair();
  MessagePair(infer::Factor *_f1, infer::Factor *_f2);  //factor-to-factor message (e.g., separate in JTA)
  MessagePair(infer::Variable *_v1, infer::Variable *_v2, infer::Factor *_v_to_v_fac); //var-to-var message (pair-wise net)
  MessagePair(infer::Factor *_f1, infer::Variable *_v2); //factor-to-var message (e.g., bi-partite graph)
  ~MessagePair();
  void init(infer::Factor *_f1, infer::Factor *_f2);
  void init(infer::Variable *_v1, infer::Variable *_v2, infer::Factor *_v_to_v_fac);
  void init(infer::Factor *_f1, infer::Variable *_v2);
  void operator=(const MessagePair& s){ variables=s.variables; f1=s.f1; f2=s.f2; m12=s.m12; m21=s.m21; }
  void write(std::ostream& os) const;
  void writeIds(std::ostream& os) const;
};
}


//===========================================================================
//
// basic operations on variables, factors, & messages
//

namespace infer {
//-- operations on factors
void tensorProduct(infer::Factor& c, const infer::Factor& a, const infer::Factor& b);
//[[specify remainingVars and guarantee that the output has right order!]]
void tensorProductMarginal(infer::Factor& c, const infer::Factor& a, const infer::Factor& b, const infer::VariableList& eliminateVars);
void tensorMarginal(infer::Factor& m, const infer::Factor& f, const infer::VariableList& marginalVars);    //marginalVars==remaining Vars
void tensorMaxMarginal(infer::Factor& m, const infer::Factor& f, const infer::VariableList& marginalVars);
void tensorMultiply(infer::Factor& f, const infer::Factor& m);
void tensorDivide(infer::Factor& f, const infer::Factor& m);
void tensorAdd(infer::Factor& f, const infer::Factor& m);
void tensorInvertMultiply(infer::Factor& f, const infer::Factor& m);
void tensorWeightedAdd(infer::Factor& f, double w, const infer::Factor& m);
void checkConsistent(const infer::Factor &f);

//-- helpers for variables lists
inline uintA ids(const infer::VariableList& vars){ uintA id(vars.N); for(uint i=0; i<id.N; i++) id(i)=vars(i)->id; return id; }

//-- operations on pure factor lists
void getJoint(infer::Factor& joint, const infer::FactorList& factors);
void computeEliminationOrder(infer::VariableList& elimOrder, const infer::FactorList& factors, const infer::VariableList& elimVars);
void eliminateVariable(infer::FactorList& factors, infer::FactorList& newed_factors, infer::Variable *var);
void eliminationAlgorithm(infer::Factor& post, const infer::FactorList& factors, const infer::VariableList& remaining_vars);
void checkConsistent(const infer::FactorList& F);

//-- operations on single messages
void collectBelief(infer::Factor& belief, const infer::Factor& f, const MessagePair *exclude);
void collectBelief(infer::Factor& belief, infer::Variable *v, const MessagePair *exclude);
void recomputeMessage_12(MessagePair& sep);
void recomputeMessage_21(MessagePair& sep);
void recomputeBatchOfMessages(MessagePairList& msgs, bool invert_order=false);
void recomputeBatchOfMessages(MessagePairList &msgs, const boolA &msgFlips, bool invert_order);
bool checkConsistency(const MessagePair& sep);
bool checkConsistencyBatch(const MessagePairList& msgs);

//-- operations on structures (after factors have been linked to messages)
void constructTreeMessageOrder(MessagePairList& msgs, boolA &msgFlips, const infer::Factor *root);
void treeInference(const infer::Factor *root, bool checkConsistency);

//-- LoopyBP engine
struct LoopyBP {
  ~LoopyBP();
  MessagePairList msgs;
  infer::VariableList    vars;
  infer::FactorList      facs;
  void clear();
  void initBipartite(const infer::VariableList& vars, const infer::FactorList& facs);
  void initPairwise(const infer::VariableList& vars, const infer::FactorList& facs);
  void getVarBeliefs(MT::Array<infer::Factor>& beliefs, bool normalized=true);
  void getVarBelief(infer::Factor& belief, infer::Variable *v, bool normalized=true);
  void step();
  void step_meanfield();
  //void loopyBP_pairwise(const infer::VariableList& vars, const infer::FactorList& facs);
  //void loopyBP_bipartite(const infer::VariableList& vars, const infer::FactorList& facs);
};
void connectThemUp(infer::VariableList& V, infer::FactorList& F);
void getVariableBeliefs(MT::Array<arr>& post, const infer::VariableList& vars);

//-- pipes
//inline ostream& operator<<(ostream& os, const iSpace& s)      { s.write(os); return os; }
inline ostream& operator<<(ostream& os, const Variable& v)    { v.write(os); return os; }
inline ostream& operator<<(ostream& os, const Factor& f)      { f.write(os); return os; }
inline ostream& operator<<(ostream& os, const MessagePair& s) { s.write(os); return os; }
inline ostream& operator<<(ostream& os, const VariableList& list)   { listWrite(list, os, "\n"); return os; }
inline ostream& operator<<(ostream& os, const FactorList& list)     { listWrite(list, os, "\n"); return os; }
inline ostream& operator<<(ostream& os, const MessagePairList& list){ listWrite(list, os, "\n"); return os; }
}

// =======================================================================
//
//  inference for mixture length DBNs
//

namespace infer {
void inferMixLengthUnstructured(
  arr& alpha, arr& beta, arr& PT, double& PR, double& ET,
  const arr& S, const arr& R, const arr& P, double gamma, uint Tmax,
  bool updateMode=false);
  
void inferMixLengthStructured(
  infer::Factor& alpha, infer::Factor& beta, arr& PT, double& PR, double& ET,
  const infer::VariableList& headVars, const infer::VariableList& tailVars,
  const infer::FactorList& S, const infer::FactorList& R, const infer::FactorList& P, double gamma, uint Tmax,
  bool updateMode=false);
}


//=======================================================================
//
//  inference on trees
//

struct TreeNode {
  int parent;
  uint dim;
  arr P;
};
typedef MT::Array<TreeNode> Tree;

void write(Tree& tree);
void treeInference(MT::Array<arr>& posteriors, const Tree& tree);
void treeInference(MT::Array<arr>& posteriors, const Tree& forest, uintA& roots);
void randomTree(Tree& tree, uint N, uint K, uint roots=1);
std::ostream& operator<<(std::ostream& os, const TreeNode& t);


//===========================================================================
//
// infer::Factor Graph (obsolete!!) (the new way is simply a list of variables and factors! (see LoopyBP example above))
//
namespace infer {
struct FactorGraph {
  infer::VariableList V;
  infer::FactorList F;   // original factors (over cliques; kept constant)
  infer::FactorList F_v; // dummies for variable factors; only used in case of true factor graph; all 1
  MessagePairList messages;  // point to F / F_v
  infer::FactorList B_c; // beliefs over clique factors (only for saving; optional)
  infer::FactorList B_v; // beliefs over variable factors (only for saving; optional)
  
  ~FactorGraph(){ deleteAll(); }
  FactorGraph& operator=(const FactorGraph& M){
    MT_MSG("das kopiert nur die pointer, erzeugt keinen neuen infer::Factor graphen!");
    B_c=M.B_c;
    B_v=M.B_v;
    messages=M.messages;
    F=M.F;
    F_v=M.F_v;
    return *this;
  }
  
  // deletes beliefs and message pairs
  void deleteAll();
  
  void setCliqueBeliefs(const infer::FactorList& fs_orig);
  void resetCliqueBeliefs(){setCliqueBeliefs(F);} // B_c
  void resetMessages();  // msg_pairs
  void resetVariableFactors();  // F_v
  void resetVariableBeliefs();  // B_v
  
  void checkCondNormalization_B_c(double tol=1e-10);
  
  double computeBeliefs();
  
  infer::Factor* getBelief(infer::Factor* f_orig);
  
  // deprecated -- nicht mehr benutzt, da auf den beliefs nicht mehr gerechnet wird
//   void checkFaithfulness(); // for whole graph; based on Marc's eq (2) / (3)

  void write(std::ostream& os = cout, bool writeBeliefs = true, bool writeMessages = true) const;
  void writeNice(std::ostream& os = cout) const;
  void writeMessagePairs(std::ostream& os) const;
  void writeVariableBeliefs(std::ostream& os = cout) const;
  
  // EFFICIENCY helpers
  
  // in case lookup for beliefs needs to be fast
  std::map<infer::Factor*, infer::Factor*> F2B;
  infer::Factor* getBelief_fast(infer::Factor* f_orig);
  void setF2Bmap();
  void addF2Bmap(infer::Factor* f, infer::Factor* b);
  
  // factors where variable is first (i.e., the conditioned variable)
  std::map<uint, infer::FactorList > V2F;
  std::map<uint, infer::Factor*> V2Fv;
  void setV2F();
  void addV2Fmap(infer::Factor* f);
  void setV2Fv();
};
stdOutPipe(FactorGraph);
}


//===========================================================================
//
// some helper functions
//

void write(const infer::FactorList& facs, ostream& os = cout);
void writeNice(const infer::VariableList& vars);
void writeNice(const infer::FactorList& individual_factors);
void writeEdges(const infer::FactorList& individual_factors, bool withProbs = false);
void writeExtremelyNice(const infer::FactorList& facs);


// calculates marginal for given variables
void getMarginal(infer::Factor& marginal, const uintA& marginalVars, infer::FactorGraph& fg);
void getMarginal(infer::Factor& marginal, const infer::VariableList& marginalVars, infer::FactorGraph& fg);



//===========================================================================
//
// Junction Tree methods
//

namespace infer {
namespace JunctionTree {
/** triangulates graph based on factors; ensures that resulting factors are max cliques;
corresponds to UNDIRECTED_GRAPH_ELIMINATE Jordan, Chapter 3, p. 13 */
void buildTriangulatedCliques(const infer::FactorList& factors, infer::FactorList& triangulatedCliques);

/** Builds max spanning tree (weights of edges according to size of set of MessagePair variables */
void buildMaxSpanningTree(infer::FactorList& factors, const infer::VariableList& vars, FactorGraph& cliqueTree);

// main method
/** Constructs a junction tree: triangulates original graph, builds max spanning tree
and updates probabilities &*/
void constructJunctionTree(FactorGraph& junctionTree, const infer::FactorList& factors, const infer::VariableList& vars);

/** Update prob dist on graph by passing messages */
void collectAndDistributeInference(FactorGraph& junctionTree);

void junctionTreeInference(FactorGraph& junctionTree, const infer::FactorList& factors, const infer::VariableList& vars);

void checkJunctionTreeProperty(FactorGraph& junctionTree);

/** adds evidence node to graph !*/
void addEvidence(FactorGraph& junctionTree, infer::Factor& evid);
}
}


//===========================================================================
//
//   Loopy BP
//

enum MsgCalc { WITH_DIV, NO_DIV };

namespace infer {
namespace LoopyBP_obsolete {

// with variable factors
void constructBipartiteFactorGraph(FactorGraph& fg, const infer::FactorList& factors);

enum PassType { PARALLEL };

double passAllEdges_parallel(FactorGraph& fg);
double passAllEdges(FactorGraph& fg, PassType type);
/** computes all outgoing messages of belief */
void shoutMessages(infer::Factor& f, MsgCalc calcMsgType = NO_DIV);

// main method
void loopy_belief_propagation(FactorGraph& fg, const infer::FactorList& factors);
}
}










#if 1

// OLD STUFF COPIED FROM MARC'S OLD CODE
/** automatically allocate message factors for mu_fwd and mu_bwd at each edge with the
correct factor types (derived from the nodes) \ingroup infer2 */
void allocateEdgeFactors(infer::FactorGraph& G);
/** test the factor graph for equilibrium (which means converged inference) \ingroup infer2 */
bool checkEquilibrium(infer::FactorGraph& G);
void getIndices(uintA& list, const uintA& id, const uintA& dim, const uintA& mid);
void getPick(uintA& pick, const uintA& id, const uintA& mid);
void fb(infer::FactorGraph& G);
/** calls message passing for a certain selection of edges.
seq gives the indices of the edges in the first time slice.
Those indices are extrapolated over a whole DBN of time length T with Mod
edges in each time slice. A negative indes means that the esge passes backward.
dir=BACKWARD means that one starts with the last time slice of the DBN going
towards the first. \ingroup infer2 */
//void passCertainEdges(infer::FactorGraph& G, intA seq, uint T, uint Mod, FwdBwd dir);
/** \ingroup infer2 */
//void passLabeledEdges(infer::FactorGraph& G, int label, FwdBwd dir);
void clearLabeledEdges(infer::FactorGraph& G, int label);
void resetLabeledNodes(infer::FactorGraph& G, const char *name);
void resetCertainNodes(infer::FactorGraph& G, intA seq);
void resetAllNodes(infer::FactorGraph& G);
void uniformCertainNodes(infer::FactorGraph& G, intA seq, uint T, uint Mod);
/// \ingroup infer2


//////////////////////////////////////////////

// For each var there is exactly one factor with the variable as first var.
void check_exactlyOneConditional(infer::VariableList& vars, infer::FactorList& facs);
void check_atLeastOneConditional(infer::VariableList& vars, infer::FactorList& facs);

void write(const MT::Array< infer::Variable* > vars, ostream& os = cout);

//[mt] only checks the registries - not the factors
void checkMessagePairConsistency(infer::MessagePairList msg_pairs);
infer::MessagePair* getMessagePair(const infer::Factor* f1, const infer::Factor* f2);
infer::Factor* getMessage(const infer::Factor* f_from, const infer::Factor* f_to);
void writeMessage(const infer::Factor* f_from, const infer::Factor* f_to, ostream& os = cout);
void sample(infer::Factor& f, uintA& samples);///< sample (s=discrete, x=continuous)

// Accessing global objects [mt] shouldn't be visible anymore
//iSpace* get_global_space();
void get_global_vars(infer::VariableList& vars);
void print_global_vars(uintA ids);


//===========================================================================
// Algorithms on factors and message pairs and graphs

namespace infer {
// 2 types of calculating messages
/** calculate new message */
void computeMessage_withDiv(infer::Factor& f_from, infer::Factor& f_to); // based on Marc's eq (5) / (7)
void computeMessage_noDiv(infer::Factor& f_from, infer::Factor& f_to); // based on Marc's eq (4) / (6)
// using already calculated belief of f_from
void computeMessage_noDiv(infer::Factor& f_from, infer::Factor& b_from, infer::Factor& f_to);

/** computes all incoming messages of belief */
void askForMessages(infer::Factor& f, MsgCalc calcMsgType = NO_DIV); //[mt] similar to collectBelief?
// BELIEF BASED
// --> using belief factors for storage

/** Updates belief according to original factor and incoming messages */
void collectBelief(infer::Factor& belief, const infer::Factor& f, const MessagePair *exclude);

/** pass message from f_from to f_to and write it into b_to*/
// based on Marc's eq (2) / (3)
// if calcMsgType==with_division, the incoming msgs to f_from are not used to calc the message.
// --> important if we do a mixture of belief propagation and setting certain factors inbetween by hand
double passMessage(infer::Factor& f_from, infer::Factor& f_to, infer::Factor& b_to, MsgCalc calcMsgType);

/** computes messages to all neighbors and updates these accordingly; should only
be used in case of loopy BP (for efficiency reasons) */
// NIY
// double distributeMessages(FactorGraph& fg, infer::Factor& f, MsgCalc calcMsgType = NO_DIV);

/** Calculates posterior over the variables given in "post" using the elimination algorithm. */
//void posteriorByElimination(infer::FactorList& factors, infer::Factor& post);
}
#endif

//===========================================================================
//
// coda
//

#ifdef  MT_IMPLEMENTATION
#  include "infer.cpp"
#endif

#endif



