#include <Core/graph.h>

//### Level 0 routines

ItemL getLiteralsOfScope(Graph& KB);
ItemL getVariablesOfScope(Graph& KB);
ItemL getVariables(Item *literal);
uint getNumOfVariables(Item* literal);

//---------- checking equality of facts or literals

bool factsAreEqual(Item *fact0, Item *fact1, bool checkAlsoValue=false);
bool factsAreEqual(Item *fact0, ItemL& fact1);
bool factsAreEqual(Item *fact, Item *literal, const ItemL& subst, Graph* subst_scope, bool checkAlsoValue=false);
Item *getEqualFactInKB(Graph& KB, Item *fact, bool checkAlsoValue=true);
Item *getEqualFactInKB(Graph& KB, ItemL& fact);
Item *getEqualFactInKB(Graph& KB, Item *literal, const ItemL& subst, Graph* subst_scope, bool checkAlsoValue=true);
Item *getEqualFactInList(Item *fact, ItemL& facts);
bool allFactsHaveEqualsInScope(Graph& KB, ItemL& facts);

bool matchingFactsAreEqual(Graph& KB, Item *it1, Item *it2, const ItemL& subst, Graph* subst_scope);

//---------- finding possible variable substitutions

void removeInfeasibleSymbolsFromDomain(Graph& KB, ItemL& domain, Item *literal);
ItemL getSubstitutions(Graph& KB, ItemL& literals, ItemL& symbols, bool verbose=false);
ItemL getRuleSubstitutions(Graph& KB, Item *rule, ItemL& symbols, bool verbose=false);

//----------- adding facts

Item *createNewSubstitutedLiteral(Graph& KB, Item* literal, const ItemL& subst, Graph* subst_scope);
bool applyEffectLiterals(Graph& KB, Item* effectliterals, const ItemL& subst, Graph* subst_scope);

//------------ fwd chaining

bool forwardChaining_FOL(Graph& KB, Item* query, bool verbose=false);
bool forwardChaining_propositional(Graph& KB, Item* q);

//### Level 1 class

//struct FOL: Graph{
//  Graph& state;


//};
