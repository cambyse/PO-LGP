#pragma once

#include <Core/graph.h>

//### Level 0 routines

NodeL getLiteralsOfScope(Graph& KB);
NodeL getSymbolsOfScope(Graph& KB);
NodeL getVariables(Node* literal, Graph* varScope);
uint getNumOfVariables(Node* literal, Graph* varScope);
Node *getFirstVariable(Node* literal, Graph* varScope);
//Node *getFirstSymbol(Node* literal, Graph* varScope);

//---------- checking equality of facts or literals

bool factsAreEqual(Node *fact0, Node *fact1, bool checkAlsoValue=false);
bool factsAreEqual(Node *fact0, NodeL& fact1);
bool factsAreEqual(Node *fact, Node *literal, const NodeL& subst, Graph* subst_scope, bool checkAlsoValue=false, bool ignoreSubst=false);
Node *getEqualFactInKB(Graph& facts, Node *fact, bool checkAlsoValue=true);
Node *getEqualFactInKB(Graph& facts, NodeL& fact);
Node *getEqualFactInKB(Graph& facts, Node *literal, const NodeL& subst, Graph* subst_scope, bool checkAlsoValue=true);
NodeL getPotentiallyEqualFactInKB(Graph& facts, Node* literal, Graph* subst_scope, bool checkAlsoValue=true);
Node *getEqualFactInList(Node *fact, NodeL& facts);
bool allFactsHaveEqualsInScope(Graph& KB, NodeL& facts);

bool matchingFactsAreEqual(Graph& facts, Node *it1, Node *it2, const NodeL& subst, Graph* subst_scope);

//---------- finding possible variable substitutions

void removeInfeasibleSymbolsFromDomain(Graph& facts, NodeL& domain, Node *literal, Graph* varScope);
NodeL getSubstitutions(Graph& facts, NodeL& literals, NodeL& domain, int verbose=0);
NodeL getRuleSubstitutions(Graph& facts, Node *rule, NodeL& domain, int verbose=0);

//----------- adding facts

Node *createNewFact(Graph& facts, const NodeL& symbols);
Node *createNewSubstitutedLiteral(Graph& facts, Node* literal, const NodeL& subst, Graph* subst_scope);
bool applySubstitutedLiteral(Graph& facts, Node*  literal, const NodeL& subst, Graph* subst_scope, Graph& changes=NoGraph);
bool applyEffectLiterals    (Graph& facts, Graph& effects, const NodeL& subst, Graph* subst_scope, Graph& changes=NoGraph);

//------------ fwd chaining

bool forwardChaining_FOL(Graph& KB, Node* query, Graph& changes=NoGraph, int verbose=0);
bool forwardChaining_propositional(Graph& KB, Node* q);

//### Level 1 class

//struct FOL: Graph{
//  Graph& state;


//};
