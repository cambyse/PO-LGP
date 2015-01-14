#include <Core/keyValueGraph.h>

/// given a scope (a subKvg, e.g. the full KB, or a rule or so), return all literals (defined by degree>0)
ItemL getLiteralsOfScope(Graph& KB);

/// return all variables (defined by degree=0)
ItemL getVariablesOfScope(Graph& KB);

/// returns all variables of the literal
ItemL getVariables(Item *literal);

uint getNumOfVariables(Item* literal);

/// ONLY for a literal with one free variable: remove all infeasible values from the domain
/// this is meant to be used as basic 'constraint propagation' for order-1 constraints
void removeInfeasible(ItemL& domain, Item *literal);

/// check if these are literally equal (all arguments are identical, be they vars or consts)
bool match(Item *literal0, Item *literal1);

/// try to find a literal within 'scope' that is exactly equal to 'literal'
Item *getMatchInScope(Item *literal, Graph* scope);

/// check if all literals in 'literals' can be matched with one in scope
bool checkAllMatchesInScope(ItemL& literals, Graph* scope);

/// return the subset of 'literals' that matches with a fact (calling match(lit0, lit1))
ItemL getFactMatches(Item *literal, ItemL& literals);

/// check match, where all variables of literal are replaced by subst(var->index)
bool match(Item *fact, Item *literal, const ItemL& subst, Graph* subst_scope);

/// create a new literal by substituting all variables with subst(var->index) (if non-NULL)
/// add the new literal to KB
Item *createNewSubstitutedLiteral(Graph& KB, Item* literal, const ItemL& subst, Graph* subst_scope);

void applyEffectLiterals(Graph& KB, Item* literals, const ItemL& subst, Graph* subst_scope);

/// check if subst is a feasible substitution for a literal (by checking with all facts that have same predicate)
bool checkFeasibility(Item *literal, const ItemL& subst, Graph* subst_scope);

/// the list of literals is a conjunctive clause (e.g. precondition)
/// all literals must be in the same scope (element of the same subKvg)
/// we return all feasible substitutions of the literal's variables by constants
/// the return value is an array: for every item of the literal's scope:
/// if item=variable the array contains a pointer to the constant
/// if item=non-variable the arrach contains a NULL pointer
ItemL getSubstitutions(ItemL& literals, ItemL& state, ItemL& constants, bool verbose=false);

/// extracts the preconditions of the rule, then returns substitutions
ItemL getRuleSubstitutions(Item *rule, ItemL& state, ItemL& constants, bool verbose=false);


bool forwardChaining_FOL(KeyValueGraph& KB, Item* query);

// actually propositional logic:
bool forwardChaining_propositional(KeyValueGraph& KB, Item* q);
