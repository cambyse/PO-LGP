#pragma once

#include <Core/graph.h>

//### Level 0 routines

NodeL getLiteralsOfScope(Graph& KB);
NodeL getSymbolsOfScope(const Graph& KB);
NodeL getVariables(Node* literal, Graph* varScope);
uint getNumOfVariables(Node* literal, Graph* varScope);
Node *getFirstVariable(Node* literal, Graph* varScope);
//Node *getFirstSymbol(Node* literal, Graph* varScope);

//---------- checking equality of facts or literals

bool tuplesAreEqual(NodeL& tuple0, NodeL& tuple1);
bool valuesAreEqual(Node *fact0, Node *fact1, bool booleanMeansExistance);
bool factsAreEqual(Node *fact0, Node *fact1, bool checkAlsoValue);
bool factsAreEqual(Node *fact, Node *literal, const NodeL& subst, const Graph* subst_scope, bool checkAlsoValue, bool ignoreSubst=false);
bool getEqualFactInKB(Graph& facts, Node *fact, bool checkAlsoValue=true);
bool getEqualFactInKB(Graph& facts, Node *literal, const NodeL& subst, Graph* subst_scope, bool checkAlsoValue=true);
NodeL getPotentiallyEqualFactsInKB(Graph& facts, Node* tuple, const Graph& varScope, bool checkAlsoValue=true);
Node *getEqualFactInList(Node *fact, NodeL& facts, bool checkAlsoValue=true);
bool allFactsHaveEqualsInScope(Graph& KB, NodeL& facts, bool checkAlsoValue=true);

bool matchingFactsAreEqual(Graph& facts, Node *it1, Node *it2, const NodeL& subst, Graph* subst_scope);

//---------- finding possible variable substitutions

void removeInfeasibleSymbolsFromDomain(Graph& facts, NodeL& domain, Node *literal, Graph* varScope);
NodeL getSubstitutions2(Graph& facts, NodeL& relations, int verbose=0);
NodeL getRuleSubstitutions2(Graph& facts, Node *rule, int verbose=0);

//----------- adding facts

Node *createNewFact(Graph& facts, const NodeL& symbols);
Node *createNewSubstitutedLiteral(Graph& facts, Node* literal, const NodeL& subst, Graph* subst_scope);
bool applySubstitutedLiteral(Graph& facts, Node*  literal, const NodeL& subst, Graph* subst_scope, Graph& changes=NoGraph);
bool applyEffectLiterals    (Graph& facts, NodeL& effects, const NodeL& subst, Graph* subst_scope, Graph& changes=NoGraph);

//------------ fwd chaining

bool forwardChaining_FOL(Graph& state, NodeL& rules, Node* query=NULL, Graph& changes=NoGraph, int verbose=0, int* decisionObservation=NULL);
bool forwardChaining_FOL(Graph& KB, Graph& state, Node* query, Graph& changes=NoGraph, int verbose=0, int* decisionObservation=NULL);
bool forwardChaining_propositional(Graph& KB, Node* q);

//------------ functions

double evaluateFunction(Graph& func, Graph& state, int verbose=0);

//### Level 1 class

//struct FOL: Graph{
//  Graph& state;


//};
