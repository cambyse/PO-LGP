#include <Core/keyValueGraph.h>

ItemL getLiteralsOfScope(Graph& KB);
ItemL getVariablesOfScope(Graph& KB);
ItemL getVariables(Item *literal);
uint getNumOfVariables(Item* literal);

bool match(Item *literal0, Item *literal1);

Item *getMatchInScope(Item *literal, Graph* scope);

bool checkAllMatchesInScope(ItemL& literals, Graph* scope);

ItemL getFactMatches(Item *literal, ItemL& literals);

bool match(Item *fact, Item *literal, const ItemL& subst, Graph* subst_scope, bool checkAlsoValue=false);

Item *createNewSubstitutedLiteral(Graph& KB, Item* literal, const ItemL& subst, Graph* subst_scope);

bool applyEffectLiterals(Graph& KB, Item* effectliterals, const ItemL& subst, Graph* subst_scope);

bool checkTruth(Item *literal, const ItemL& subst, Graph* subst_scope);

void removeInfeasible(ItemL& domain, Item *literal, bool checkAlsoValue=false);

ItemL getSubstitutions(ItemL& literals, ItemL& state, ItemL& constants, bool verbose=false);

ItemL getRuleSubstitutions(Item *rule, ItemL& state, ItemL& constants, bool verbose=false);


bool forwardChaining_FOL(KeyValueGraph& KB, Item* query, bool verbose=false);

bool forwardChaining_propositional(KeyValueGraph& KB, Item* q);
