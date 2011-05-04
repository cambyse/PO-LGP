/* Copyright (C) 2000, 2006  Marc Toussaint (mtoussai@inf.ed.ac.uk)
under the terms of the GNU LGPL (http://www.gnu.org/copyleft/lesser.html)
see the `util.h' file for a full copyright statement  */

#ifndef MT_inference_h
#define MT_inference_h

#include "array.h"
#include "util.h"
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

namespace infer{
struct Variable;
struct Factor;
struct MessagePair;
}

//simple ``list'' types
typedef MT::Array<infer::Variable*> VariableList;
typedef MT::Array<infer::Factor*>   FactorList;
typedef MT::Array<infer::MessagePair*> MessagePairList;

#define VarL VariableList
#define FacL FactorList
#define MsgPairL MessagePairList

extern uint VarCount;

namespace infer{
/*! a discrete random variable */
struct Variable{
  //core defining properties
  uint id;          //!< unique identifyer
  uint dim;         //!< cardinality
  
  // auxilliary & connectivity
  MT::String name;  //!< up to you...
  FactorList factors;    //!< each variable knows all factors it is part of
  MessagePairList messages;  //!< each variable knows all the messages it directly connects to
  
  Variable();
  Variable(uint _dim,const char *_name);
  Variable(uint _dim,const char *_name,uint _id);
  ~Variable();
  operator uint() const { return id; }
  void write(ostream& os = std::cout) const;
};
}

namespace infer{
/*! a factor (=probability table) over a tuple of variable; a list of
    factors readily defines a proper factored joint distribution

    Probability tables are multi-dimensional arrays (tensors), per
    variable a separate dimension (boolean variables: first false,
    then true). */
struct Factor{
  //core defining properties
  uintA varIds; //!< f=f(x_1,x_3,x_7) => id=[1,3,7]; array of variables (their indices) this factor depends on
  uintA dim;    //!< f=f(x_1,x_3,x_7) => dim=[dim(x_1),dim(x_3),dim(x_7)];  array of dimensionalities of the variables this factor depends on
  arr P;        //!< the (probability) table
  double logP;  //!< the log-scaling of the table, such that true_factor = exp(logP) * P

  // auxilliary & connectivity
  MT::String name;
  VariableList variables;
  FactorList factors;
  MessagePairList messages;          //!< each factor knows all the msg_pairs it connects to
  
  Factor();
  ~Factor();
  Factor(const VariableList& variables,const char *_name=NULL);
  Factor(const VarL& variables,const arr& q);
  void relinkTo(const VarL& variables);
  void operator=(const Factor& q);
  void setP(const arr& q);           //!< f(x) = q(x)
  void setText(const char* text);    //!< f(x) = q(x)
  void setOne();                     //!< f(x) = 1
  void setUniform();                 //!< sum(x_i) f(x_i) = 1  only if factor.P is set by hand (as a conditional)
  void setRandom();                  //!< randomize f (e.g., for automatic testing)
  void setEvidence(uint e);          //!< f(e) = 1,  f(x!=e) = 0
  bool operator==(Factor& q);        //!< check if f==q (also checks for logP)
  void getP(arr& p) const;           //!< p = exp(logP)*P
  void write(std::ostream& os = std::cout,bool brief=false) const;
  void writeNice(std::ostream& os = std::cout) const;
  void writeExtremelyNice(std::ostream& os = std::cout) const;
  
  void makeLogZero(){ P *= ::exp(logP); logP=0.; }
  
  double entry(uint i);              //!< returns logP * P.elem(i)
  void checkCondNormalization(uint left=1,double tol=1e-10);
  uint numberNonZeroEntries();
};
}

namespace infer{
/*! a pair of messages (fwd and bwd) which connects two
    factors. Depending on the context this can be a separator (JTA), a
    link from factor to a single-variable-factor (bi-partite factor
    graph), or a links between arbitrary factors (loopy BP) */
struct MessagePair{
  //core defining properties
  Factor m12, m21;      //!< the forward and backward message
  Factor   *f1, *f2;    //!< the first and second factor it is attached to (if at all..)
  Variable *v1, *v2;    //!< the first and second variable it is attached to (if at all..)
  Factor   *v_to_v_fac; //!< in case of variable-to-variable message: the factor it is one-to-one associated with
  VarL variables;       //!< the variables the messages are defined over
  
  MessagePair();
  MessagePair(Factor *_f1,Factor *_f2);  //factor-to-factor message (e.g., separate in JTA)
  MessagePair(Variable *_v1,Variable *_v2,Factor *_v_to_v_fac); //var-to-var message (pair-wise net)
  MessagePair(Factor *_f1,Variable *_v2); //factor-to-var message (e.g., bi-partite graph)
  ~MessagePair();
  void init(Factor *_f1,Factor *_f2);
  void init(Variable *_v1,Variable *_v2,Factor *_v_to_v_fac);
  void init(Factor *_f1,Variable *_v2);
  void operator=(const MessagePair& s){ variables=s.variables; f1=s.f1; f2=s.f2; m12=s.m12; m21=s.m21; }
  void write(std::ostream& os) const;
  void writeIds(std::ostream& os) const;
};
}


//===========================================================================
//
// basic operations on variables, factors, & messages
//

namespace infer{
//-- operations on factors
void tensorProduct(Factor& c, const Factor& a, const Factor& b);
//[[specify remainingVars and guarantee that the output has right order!]]
void tensorProductMarginal(Factor& c, const Factor& a, const Factor& b, const VarL& eliminateVars);
void tensorMarginal(Factor& m,const Factor& f, const VarL& marginalVars);    //marginalVars==remaining Vars
void tensorMaxMarginal(Factor& m,const Factor& f, const VarL& marginalVars);
void tensorMultiply(Factor& f,const Factor& m);
void tensorDivide(Factor& f,const Factor& m);
void tensorAdd(Factor& f,const Factor& m);
void tensorInvertMultiply(Factor& f,const Factor& m);
void tensorWeightedAdd(Factor& f,double w,const Factor& m);
void checkConsistent(const Factor &f);

//-- helpers for variables lists
inline uintA ids(const VarL& vars){ uintA id(vars.N); for(uint i=0;i<id.N;i++) id(i)=vars(i)->id; return id; }

//-- operations on pure factor lists
void getJoint(Factor& joint,const FacL& factors);
void computeEliminationOrder(VarL& elimOrder, const FacL& factors, const VarL& elimVars);
void eliminateVariable(FacL& factors, FacL& newed_factors,Variable *var);
void eliminationAlgorithm(Factor& post,const FacL& factors, const VarL& remaining_vars);
void checkConsistent(const FacL& F);

//-- operations on single messages
void collectBelief(Factor& belief,const Factor& f,const MessagePair *exclude);
void collectBelief(Factor& belief,Variable *v,const MessagePair *exclude);
void recomputeMessage_12(MessagePair& sep);
void recomputeMessage_21(MessagePair& sep);
void recomputeBatchOfMessages(MessagePairList& msgs,bool invert_order=false);
void recomputeBatchOfMessages(MessagePairList &msgs,const boolA &msgFlips,bool invert_order);
bool checkConsistency(const MessagePair& sep);
bool checkConsistencyBatch(const MsgPairL& msgs);

//-- operations on structures (after factors have been linked to messages)
 void constructTreeMessageOrder(MessagePairList& msgs, boolA &msgFlips, const Factor *root);
void treeInference(const Factor *root,bool checkConsistency);

//-- LoopyBP engine
struct LoopyBP{
  ~LoopyBP();
  MessagePairList msgs;
  VariableList    vars;
  FactorList      facs;
  void initBipartite(const VariableList& vars,const FactorList& facs);
  void initPairwise(const VariableList& vars,const FactorList& facs);
  void getVarBeliefs(MT::Array<Factor>& beliefs);
  void getVarBelief(Factor& belief,Variable *v);
  void step();
  void step_meanfield();
  //void loopyBP_pairwise(const VariableList& vars,const FactorList& facs);
  //void loopyBP_bipartite(const VariableList& vars,const FactorList& facs);
};
void connectThemUp(VariableList& V,FactorList& F);
void getVariableBeliefs(MT::Array<arr>& post,const VarL& vars);
}

//-- pipes
//inline ostream& operator<<(ostream& os,const iSpace& s)      { s.write(os); return os; }
inline ostream& operator<<(ostream& os,const infer::Variable& v)    { v.write(os); return os; }
inline ostream& operator<<(ostream& os,const infer::Factor& f)      { f.write(os); return os; }
inline ostream& operator<<(ostream& os,const infer::MessagePair& s) { s.write(os); return os; }
inline ostream& operator<<(ostream& os,const VarL& list)     { listWrite(list,os,"\n"); return os; }
inline ostream& operator<<(ostream& os,const FacL& list)     { listWrite(list,os,"\n"); return os; }
inline ostream& operator<<(ostream& os,const MsgPairL& list) { listWrite(list,os,"\n"); return os; }

// =======================================================================
//
//  inference for mixture length DBNs
//

void inferMixLengthUnstructured(
    arr& alpha, arr& beta, arr& PT, double& PR, double& ET,
    const arr& S, const arr& R, const arr& P, double gamma, uint Tmax,
    bool updateMode=false);

void inferMixLengthStructured(
    infer::Factor& alpha, infer::Factor& beta, arr& PT, double& PR, double& ET,
    const VarL& headVars, const VarL& tailVars,
    const FacL& S, const FacL& R, const FacL& P, double gamma, uint Tmax,
    bool updateMode=false);



//=======================================================================
//
//  inference on trees
//

struct TreeNode{
  int parent;
  uint dim;
  arr P;
};
typedef MT::Array<TreeNode> Tree;

void write(Tree& tree);
void treeInference(MT::Array<arr>& posteriors,const Tree& tree);
void treeInference(MT::Array<arr>& posteriors,const Tree& forest, uintA& roots);
void randomTree(Tree& tree,uint N,uint K, uint roots=1);
std::ostream& operator<<(std::ostream& os,const TreeNode& t);


//===========================================================================
//
// Factor Graph (obsolete!!)
//
namespace infer{
struct FactorGraph {
  VarL V;
  FacL F;   // original factors (over cliques; kept constant)
  FacL F_v; // dummies for variable factors; only used in case of true factor graph; all 1
  MessagePairList messages;  // point to F / F_v
  FacL B_c; // beliefs over clique factors (only for saving; optional)
  FacL B_v; // beliefs over variable factors (only for saving; optional)
  
  ~FactorGraph(){ deleteAll(); }
  FactorGraph& operator=(const FactorGraph& M){
    MT_MSG("das kopiert nur die pointer, erzeugt keinen neuen Factor graphen!");
    B_c=M.B_c;
    B_v=M.B_v;
    messages=M.messages;
    F=M.F;
    F_v=M.F_v;
    return *this;
  }
  
  // deletes beliefs and message pairs
  void deleteAll();

  void setCliqueBeliefs(const FacL& fs_orig);
  void resetCliqueBeliefs() {setCliqueBeliefs(F);}  // B_c
  void resetMessages();  // msg_pairs
  void resetVariableFactors();  // F_v
  void resetVariableBeliefs();  // B_v
  
  void checkCondNormalization_B_c(double tol=1e-10);
  
  double computeBeliefs();
  
  Factor* getBelief(Factor* f_orig);
  
  // deprecated -- nicht mehr benutzt, da auf den beliefs nicht mehr gerechnet wird
//   void checkFaithfulness(); // for whole graph; based on Marc's eq (2) / (3)
  
  void write(std::ostream& os = cout, bool writeBeliefs = true, bool writeMessages = true) const;
  void writeNice(std::ostream& os = cout) const;
  void writeMessagePairs(std::ostream& os) const;
  void writeVariableBeliefs(std::ostream& os = cout) const;
  
  // EFFICIENCY helpers
  
  // in case lookup for beliefs needs to be fast
  std::map<Factor*, Factor*> F2B;
  Factor* getBelief_fast(Factor* f_orig);
  void setF2Bmap();
  void addF2Bmap(Factor* f, Factor* b);
  
  // factors where variable is first (i.e., the conditioned variable)
  std::map<uint, FacL > V2F;
  std::map<uint, Factor*> V2Fv;
  void setV2F();
  void addV2Fmap(Factor* f);
  void setV2Fv();
};
}


//===========================================================================
//
// some helper functions
//

void write(const FacL& facs, ostream& os = cout);
void writeNice(const VarL& vars);
void writeNice(const FacL& individual_factors);
void writeEdges(const FacL& individual_factors, bool withProbs = false);
void writeExtremelyNice(const FacL& facs);

inline ostream& operator<<(ostream& os,const infer::FactorGraph& fg){ fg.write(os); return os; }

// calculates marginal for given variables
void getMarginal(infer::Factor& marginal, const uintA& marginalVars, infer::FactorGraph& fg);



//===========================================================================
//
// Junction Tree methods
//

namespace infer{
namespace JunctionTree {
  /*! triangulates graph based on factors; ensures that resulting factors are max cliques;
  corresponds to UNDIRECTED_GRAPH_ELIMINATE Jordan, Chapter 3, p. 13 */
  void buildTriangulatedCliques(const FacL& factors, FacL& triangulatedCliques);
  
  /*! Builds max spanning tree (weights of edges according to size of set of MessagePair variables */
  void buildMaxSpanningTree(FacL& factors, const VarL& vars, FactorGraph& cliqueTree);
  
  // main method
  /*! Constructs a junction tree: triangulates original graph, builds max spanning tree
  and updates probabilities &*/
  void constructJunctionTree(FactorGraph& junctionTree, const FacL& factors, const VarL& vars);
  
  /*! Update prob dist on graph by passing messages */
  void collectAndDistributeInference(FactorGraph& junctionTree);

  void junctionTreeInference(FactorGraph& junctionTree, const FacL& factors, const VarL& vars);

  void checkJunctionTreeProperty(FactorGraph& junctionTree);
  
  /*! adds evidence node to graph !*/
  void addEvidence(FactorGraph& junctionTree, Factor& evid);
}
}


//===========================================================================
//
//   Loopy BP
//

enum MsgCalc{ WITH_DIV, NO_DIV };

namespace infer{
namespace LoopyBP_obsolete{

  // with variable factors
  void constructBipartiteFactorGraph(FactorGraph& fg, const FacL& factors);

  enum PassType{ PARALLEL };
  
  double passAllEdges_parallel(FactorGraph& fg);
  double passAllEdges(FactorGraph& fg, PassType type);
  /*! computes all outgoing messages of belief */
  void shoutMessages(Factor& f, MsgCalc calcMsgType = NO_DIV);
  
  // main method
  void loopy_belief_propagation(FactorGraph& fg, const FacL& factors);
}
}










#if 1

// OLD STUFF COPIED FROM MARC'S OLD CODE
/*! automatically allocate message factors for mu_fwd and mu_bwd at each edge with the
correct factor types (derived from the nodes) \ingroup infer2 */
void allocateEdgeFactors(infer::FactorGraph& G);
/*! test the factor graph for equilibrium (which means converged inference) \ingroup infer2 */
bool checkEquilibrium(infer::FactorGraph& G);
void getIndices(uintA& list,const uintA& id,const uintA& dim,const uintA& mid);
void getPick(uintA& pick,const uintA& id,const uintA& mid);
void fb(infer::FactorGraph& G);
/*! calls message passing for a certain selection of edges.
seq gives the indices of the edges in the first time slice.
Those indices are extrapolated over a whole DBN of time length T with Mod
edges in each time slice. A negative indes means that the esge passes backward.
dir=BACKWARD means that one starts with the last time slice of the DBN going
towards the first. \ingroup infer2 */
//void passCertainEdges(infer::FactorGraph& G,intA seq,uint T,uint Mod,FwdBwd dir);
/*! \ingroup infer2 */
//void passLabeledEdges(infer::FactorGraph& G,int label,FwdBwd dir);
void clearLabeledEdges(infer::FactorGraph& G,int label);
void resetLabeledNodes(infer::FactorGraph& G,const char *name);
void resetCertainNodes(infer::FactorGraph& G,intA seq);
void resetAllNodes(infer::FactorGraph& G);
void uniformCertainNodes(infer::FactorGraph& G,intA seq,uint T,uint Mod);
//! \ingroup infer2


//////////////////////////////////////////////

// For each var there is exactly one factor with the variable as first var.
void check_exactlyOneConditional(VarL& vars, FacL& facs);
void check_atLeastOneConditional(VarL& vars, FacL& facs);

void write(const MT::Array< infer::Variable* > vars, ostream& os = cout);

//[mt] only checks the registries - not the factors 
void checkMessagePairConsistency(MsgPairL msg_pairs);
infer::MessagePair* getMessagePair(const infer::Factor* f1, const infer::Factor* f2);
infer::Factor* getMessage(const infer::Factor* f_from, const infer::Factor* f_to);
void writeMessage(const infer::Factor* f_from, const infer::Factor* f_to, ostream& os = cout);
void sample(infer::Factor& f,uintA& samples);//!< sample (s=discrete, x=continuous)

// Accessing global objects [mt] shouldn't be visible anymore
//iSpace* get_global_space();
void get_global_vars(VarL& vars);
void print_global_vars(uintA ids);


//===========================================================================
// Algorithms on factors and message pairs and graphs

namespace infer{
// 2 types of calculating messages
/*! calculate new message */
void computeMessage_withDiv(Factor& f_from, Factor& f_to); // based on Marc's eq (5) / (7)
void computeMessage_noDiv(Factor& f_from, Factor& f_to); // based on Marc's eq (4) / (6)
// using already calculated belief of f_from
void computeMessage_noDiv(Factor& f_from, Factor& b_from, Factor& f_to);

/*! computes all incoming messages of belief */
void askForMessages(Factor& f, MsgCalc calcMsgType = NO_DIV); //[mt] similar to collectBelief?
// BELIEF BASED
// --> using belief factors for storage

/*! Updates belief according to original factor and incoming messages */
void collectBelief(Factor& belief, const Factor& f, const MessagePair *exclude);

/*! pass message from f_from to f_to and write it into b_to*/
// based on Marc's eq (2) / (3)
// if calcMsgType==with_division, the incoming msgs to f_from are not used to calc the message.
// --> important if we do a mixture of belief propagation and setting certain factors inbetween by hand
double passMessage(Factor& f_from, Factor& f_to, Factor& b_to, MsgCalc calcMsgType);

/*! computes messages to all neighbors and updates these accordingly; should only
be used in case of loopy BP (for efficiency reasons) */
// NIY
// double distributeMessages(FactorGraph& fg, Factor& f, MsgCalc calcMsgType = NO_DIV);

/*! Calculates posterior over the variables given in "post" using the elimination algorithm. */
//void posteriorByElimination(FacL& factors, Factor& post);
}
#endif

//===========================================================================
//
// coda
//

#ifdef MT_IMPLEMENTATION
#  include"infer.cpp"
#endif

#endif



