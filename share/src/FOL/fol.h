#include <Core/graph.h>

//### Level 0 routines

ItemL getLiteralsOfScope(Graph& KB);
ItemL getSymbolsOfScope(Graph& KB);
ItemL getVariables(Item* literal, Graph* varScope);
uint getNumOfVariables(Item* literal, Graph* varScope);
Item *getFirstVariable(Item* literal, Graph* varScope);
//Item *getFirstSymbol(Item* literal, Graph* varScope);

//---------- checking equality of facts or literals

bool factsAreEqual(Item *fact0, Item *fact1, bool checkAlsoValue=false);
bool factsAreEqual(Item *fact0, ItemL& fact1);
bool factsAreEqual(Item *fact, Item *literal, const ItemL& subst, Graph* subst_scope, bool checkAlsoValue=false);
Item *getEqualFactInKB(Graph& facts, Item *fact, bool checkAlsoValue=true);
Item *getEqualFactInKB(Graph& facts, ItemL& fact);
Item *getEqualFactInKB(Graph& facts, Item *literal, const ItemL& subst, Graph* subst_scope, bool checkAlsoValue=true);
Item *getEqualFactInList(Item *fact, ItemL& facts);
bool allFactsHaveEqualsInScope(Graph& KB, ItemL& facts);

bool matchingFactsAreEqual(Graph& facts, Item *it1, Item *it2, const ItemL& subst, Graph* subst_scope);

//---------- finding possible variable substitutions

void removeInfeasibleSymbolsFromDomain(Graph& facts, ItemL& domain, Item *literal, Graph* varScope);
ItemL getSubstitutions(Graph& facts, ItemL& literals, ItemL& domain, bool verbose=false);
ItemL getRuleSubstitutions(Graph& facts, Item *rule, ItemL& domain, bool verbose=false);

//----------- adding facts

Item *createNewSubstitutedLiteral(Graph& facts, Item* literal, const ItemL& subst, Graph* subst_scope);
bool applyEffectLiterals(Graph& facts, Item* effectliterals, const ItemL& subst, Graph* subst_scope);

//------------ fwd chaining

bool forwardChaining_FOL(Graph& KB, Item* query, bool verbose=false);
bool forwardChaining_propositional(Graph& KB, Item* q);

//### Level 1 class

//struct FOL: Graph{
//  Graph& state;


//};
