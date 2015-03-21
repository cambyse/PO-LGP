#include "fol.h"

/// given a scope (a subKvg, e.g. the full KB, or a rule or so), return all literals (defined by degree>0)
ItemL getLiteralsOfScope(Graph& KB){
  ItemL state;
  state.anticipateMEM(KB.N);
  for(Item *i:KB) if(i->keys.N==0 && i->parents.N>0) state.append(i);
  return state;
}

/// return all variables (defined by degree=0)
ItemL getSymbolsOfScope(Graph& KB){
  ItemL vars;
  vars.anticipateMEM(KB.N);
  for(Item *i:KB) if(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool)) vars.append(i);
  return vars;
}

/// returns all variables of the literal
ItemL getVariables(Item* literal, Graph* varScope){
  ItemL vars;
  for(Item *i:literal->parents) if(&i->container==varScope){
    CHECK(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool),"");
    vars.append(i);
  }
  return vars;
}

uint getNumOfVariables(Item* literal, Graph* varScope){
  uint v=0;
  for(Item *i:literal->parents) if(&i->container==varScope){
    CHECK(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool),"");
    v++;
  }
  return v;
}

Item *getFirstVariable(Item* literal, Graph* varScope){
  for(Item *i:literal->parents) if(&i->container==varScope){
    CHECK(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool),"");
    return i;
  }
  return NULL;
}

//Item *getFirstSymbol(Item* literal, Graph* varScope){
//  for(Item *i:literal->parents) if(&i->container!=varScope){
//    CHECK(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool),"");
//    return i;
//  }
//  return NULL;
//}

/// check if these are literally equal (all arguments are identical, be they vars or consts)
bool factsAreEqual(Item* fact0, Item* fact1, bool checkAlsoValue){
  if(fact0->parents.N!=fact1->parents.N) return false;
  for(uint i=0;i<fact0->parents.N;i++){
    if(fact0->parents(i) != fact1->parents(i)) return false;
  }
  if(checkAlsoValue){
    if(fact0->getValueType()!=fact1->getValueType()) return false;
    if(!fact0->hasEqualValue(fact1)) return false;
  }
  return true;
}

/// check if these are literally equal (all arguments are identical, be they vars or consts)
bool factsAreEqual(Item *fact0, ItemL& fact1){
  if(fact0->parents.N!=fact1.N) return false;
  for(uint i=0;i<fact0->parents.N;i++){
    if(fact0->parents(i) != fact1(i)) return false;
  }
  return true;
}

/// check match, where all variables of literal are replaced by subst(var->index)
bool factsAreEqual(Item* fact, Item* literal, const ItemL& subst, Graph* subst_scope,bool checkAlsoValue){
  if(fact->parents.N!=literal->parents.N) return false;
  for(uint i=0;i<fact->parents.N;i++){
    Item *fact_arg = fact->parents(i);
    Item *lit_arg = literal->parents(i);
    if(&lit_arg->container==subst_scope){ //lit_arg is a variable -> check match of substitution
      if(subst(lit_arg->index)!=fact_arg) return false;
    }else if(lit_arg!=fact_arg) return false;
  }
  if(checkAlsoValue){
    if(fact->getValueType()!=literal->getValueType()) return false;
    if(!fact->hasEqualValue(literal)) return false;
  }
  return true;
}

/// try to find a literal within 'scope' that is exactly equal to 'literal'
Item *getEqualFactInKB(Graph& facts, Item *fact, bool checkAlsoValue){
  //first find the section of all facts that derive from the same symbols
  ItemL candidates=fact->parents(0)->parentOf;
//  for(uint p=1;p<fact->parents.N;p++)
//    candidates = setSection(candidates, fact->parents(p)->parentOf);
  //now check only these candidates
  for(Item *fact1:candidates) if(&fact1->container==&facts && fact1!=fact){
    if(factsAreEqual(fact, fact1, checkAlsoValue)) return fact1;
  }
  return NULL;
}

/// try to find a literal within 'scope' that is exactly equal to 'literal'
Item *getEqualFactInKB(Graph& facts, ItemL& fact){
  //first find the section of all facts that derive from the same symbols
  ItemL candidates=fact(0)->parentOf;
//  for(uint p=1;p<fact.N;p++)
//    candidates = setSection(candidates, fact->parents(p)->parentOf);
  //now check only these candidates
  for(Item *fact1:candidates) if(&fact1->container==&facts){
    if(factsAreEqual(fact1, fact)) return fact1;
  }
  return NULL;
}

/// check if subst is a feasible substitution for a literal (by checking with all facts that have same predicate)
Item *getEqualFactInKB(Graph& facts, Item* literal, const ItemL& subst, Graph* subst_scope, bool checkAlsoValue){
  ItemL candidates = literal->parents(0)->parentOf;
//  ItemL candidates = getLiteralsOfScope(KB);
  for(Item *fact:candidates) if(&fact->container==&facts && fact!=literal){
    if(factsAreEqual(fact, literal, subst, subst_scope, checkAlsoValue)) return fact;
  }
  return NULL;
}

/// return the subset of 'literals' that matches with a fact (calling match(lit0, lit1))
Item *getEqualFactInList(Item* fact, ItemL& facts){
//  ItemL candidates=facts;
//  for(Item *p:fact->parents) candidates = setSection(candidates, p->parentOf);
  for(Item *fact1:facts) if(factsAreEqual(fact, fact1)) return fact1; //matches.append(fact1);
  return NULL;
}

/// check if all literals in 'literals' can be matched with one in scope
bool allFactsHaveEqualsInScope(Graph& KB, ItemL& facts){
  for(Item *fact:facts){
    if(!getEqualFactInKB(KB, fact)) return false;
  }
  return true;
}

bool matchingFactsAreEqual(Graph& facts, Item *it1, Item *it2, const ItemL& subst, Graph* subst_scope){
  CHECK(&it1->container==&it2->container,"");
  if(it1->getValueType()!=it2->getValueType()) return false;
  if(it1->parents(0)!=it2->parents(0)) return false;

  Item *m1=getEqualFactInKB(facts, it1, subst, subst_scope, false);
  if(!m1) return false;
  Item *m2=getEqualFactInKB(facts, it2, subst, subst_scope, false);
  if(!m2) return false;

  if(m1==m2) return true;
  return m1->hasEqualValue(m2);
}



/// ONLY for a literal with one free variable: remove all infeasible values from the domain
/// this is meant to be used as basic 'constraint propagation' for order-1 constraints
void removeInfeasibleSymbolsFromDomain(Graph& facts, ItemL& domain, Item* literal, Graph *varScope){
  CHECK(getNumOfVariables(literal, varScope)==1," remove Infeasible works only for literals with one open variable!");
  Item *var = getFirstVariable(literal, varScope);
  Item *predicate = literal->parents(0);

  ItemL dom;
  dom.anticipateMEM(domain.N);
  for(Item *fact:predicate->parentOf) if(&fact->container==&facts){
    //-- check that all arguments are the same, except for var!
    bool match=true;
    Item *value=NULL;
    for(uint i=0;i<literal->parents.N;i++){
      Item *lit_arg = literal->parents(i);
      Item *fact_arg = fact->parents(i);
      if(lit_arg==var) value = fact_arg;
      else if(lit_arg!=fact_arg){ match=false; break; }
    }
    if(match && literal->getValueType()!=typeid(bool)){ //if the literal is boolean, we don't YET check the value (see below)
      if(fact->getValueType()!=literal->getValueType()) match=false;
      match = fact->hasEqualValue(literal);
    }
    if(match){
      CHECK(value && &value->container==&facts.isItemOfParentKvg->container,""); //the value should be a constant!
//      dom.ItemL::setAppendInSorted(value, ItemComp);
      dom.ItemL::append(value);
    }
  }

  //for a negative boolean literal, we REMOVE the matches instead of allowing for them
  if(literal->getValueType()==typeid(bool) && *((bool*)literal->getValueDirectly()) == false){
    setMinus(domain, dom);
  }else{
    domain = setSection(domain, dom);
  }
}



/// create a new literal by substituting all variables with subst(var->index) (if non-NULL)
/// add the new literal to KB
Item* createNewSubstitutedLiteral(Graph& facts, Item* literal, const ItemL& subst, Graph* subst_scope){
  Item *fact = literal->newClone(facts);
  for(uint i=0;i<fact->parents.N;i++){
    Item *arg=fact->parents(i);
    CHECK(&arg->container==subst_scope || &arg->container==&facts.isItemOfParentKvg->container,"the literal argument should be a constant (KB scope) or variable (1st level local scope)");
    if(&arg->container==subst_scope){ //is a variable, and subst exists
       CHECK(subst(arg->index)!=NULL,"a variable (=argument in local scope) requires a substitution, no?");
      //CHECK(arg->container.N==subst.N, "somehow the substitution does not fit the container of literal arguments");
      fact->parents(i) = subst(arg->index);
      arg->parentOf.removeValue(fact);
      fact->parents(i)->parentOf.append(fact);
    }
  }
//  cout <<*fact <<endl;
  return fact;
}

bool applySubstitutedLiteral(Graph& facts, Item* literal, const ItemL& subst, Graph* subst_scope){
  bool trueValue=true; //check if the literal is negated
  if(literal->getValueType()==typeid(bool)){
    if(*((bool*)literal->getValueDirectly()) == false) trueValue = false;
  }

  bool hasEffects=false;

  //first collect tuple matches
  ItemL matches;
  for(Item *fact:literal->parents(0)->parentOf) if(&fact->container==&facts){
    if(factsAreEqual(fact, literal, subst, subst_scope)) matches.append(fact);
  }

  if(trueValue){
    if(!matches.N){
      createNewSubstitutedLiteral(facts, literal, subst, subst_scope);
      hasEffects=true;
    }else{
      for(Item *m:matches){
        if(m->getValueType()==typeid(double)){ //TODO: very special HACK: double add up instead of being assigned
          *m->getValue<double>() += *literal->getValue<double>();
          hasEffects=true;
        }else{
          if(!m->hasEqualValue(literal)){
            m->copyValue(literal);
            hasEffects=true;
          }
        }
      }
    }
    //TODO: remove double!
  }else{
    //delete all matching facts!
    for(Item *fact:matches) delete fact;
    if(matches.N) hasEffects=true;
  }
  return hasEffects;
}

bool applyEffectLiterals(Graph& facts, Graph& effects, const ItemL& subst, Graph* subst_scope){
  bool hasEffects=false;
  for(Item *lit:effects){
    bool e = applySubstitutedLiteral(facts, lit, subst, subst_scope);
    hasEffects = hasEffects || e;
  }
  return hasEffects;
}


/// extracts the preconditions of the rule, then returns substitutions
ItemL getRuleSubstitutions(Graph& facts, Item *rule, ItemL& domain, bool verbose){
  //-- extract precondition
  if(verbose){ cout <<"Substitutions for rule " <<*rule <<endl; }
  Graph& Rule=rule->kvg();
  return getSubstitutions(facts, Rule.last(-2)->kvg(), domain, verbose);
}


/// the list of literals is a conjunctive clause (e.g. precondition)
/// all literals must be in the same scope (element of the same subKvg)
/// we return all feasible substitutions of the literal's variables by constants
/// the return value is an array: for every item of the literal's scope:
/// if item=variable the array contains a pointer to the constant
/// if item=non-variable the arrach contains a NULL pointer
ItemL getSubstitutions(Graph& facts, ItemL& literals, ItemL& domain, bool verbose){
  CHECK(literals.N,"");
  Graph& varScope = literals(0)->container.isItemOfParentKvg->container; //this is usually a rule (scope = subKvg in which we'll use the indexing)

  Item* EQ = facts["EQ"];
  ItemL vars = getSymbolsOfScope(varScope);

  if(verbose){
    cout <<"Substitutions for literals "; listWrite(literals, cout); cout <<" with variables '"; listWrite(vars, cout); cout <<'\'' <<endl;
//    cout <<"   with facts " <<facts <<" and domain "; listWrite(domain, cout); cout <<'\'' <<endl;
  }

  //-- initialize potential domains for each variable
  MT::Array<ItemL> domainOf(vars.N);
//  constants.sort(ItemComp);
  for(Item *v:vars) domainOf(v->index) = domain;

  if(verbose) cout <<"domains before 'constraint propagation':" <<endl;
  if(verbose) for(Item *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

  //-- grab open variables for each literal
  uintA lit_numVars(literals(0)->container.N);
  for(Item *literal:literals) lit_numVars(literal->index) = getNumOfVariables(literal, &varScope);

  //-- first pick out all precondition predicates with just one open variable and reduce domains directly
  for(Item *literal:literals){
    if(lit_numVars(literal->index)==1){
      Item *var = getFirstVariable(literal, &varScope);
      if(verbose) cout <<"checking literal '" <<*literal <<"'" <<flush;
      removeInfeasibleSymbolsFromDomain(facts, domainOf(var->index), literal, &varScope);
      if(verbose){ cout <<" gives remaining domain for '" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }
      if(domainOf(var->index).N==0){
        if(verbose) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
        return ItemL(); //early failure
      }
    }
  }

  if(verbose) cout <<"domains after 'constraint propagation':" <<endl;
  if(verbose) for(Item *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

  //-- for the others, create constraints
  ItemL constraints;
  for(Item *literal:literals){
    if(lit_numVars(literal->index)!=1 || (literal->parents.N && literal->parents(0)==EQ)){
      constraints.append(literal);
    }
  }

  if(verbose){ cout <<"remaining constraint literals:" <<endl; listWrite(constraints, cout); cout <<endl; }

  //-- naive CSP: loop through everything
  uint subN=0;
  ItemL substitutions;
  ItemL values(vars.N); values.setZero();
  {
    //-- using 'getIndexTuple' we can linearly enumerate all configurations of all variables
    uintA domainN(vars.N);
    for(uint i=0;i<vars.N;i++) domainN(i) = domainOf(i).N;  //collect dims/cardinalities of domains
    uint configurationsN = product(domainN); //number of all possible configurations
    for(uint config=0;config<configurationsN;config++){ //loop through all possible configurations
      uintA valueIndex = getIndexTuple(config, domainN);
      bool feasible=true;
      for(uint i=0;i<vars.N;i++) values(vars(i)->index) = domainOf(i)(valueIndex(i)); //assign the configuration
      //only allow for disjoint assignments
      for(uint i=0; i<values.N && feasible; i++) for(uint j=i+1; j<values.N && feasible; j++){
        if(values(i)==values(j)) feasible=false;
      }
      if(!feasible) continue;
      for(Item* literal:constraints){ //loop through all constraints
        if(literal->parents.N && literal->parents(0)==EQ){ //check equality of subsequent literals
          Item *it1 = literal->container(literal->index+1);
          Item *it2 = literal->container(literal->index+2);
          feasible = matchingFactsAreEqual(facts, it1, it2, values, &varScope);
        }else{
          feasible = getEqualFactInKB(facts, literal, values, &varScope);
          if(!feasible){ //when literal is a negative boolean literal and we don't find a match, we interpret this as feasible!
            if(literal->getValueType()==typeid(bool) && *((bool*)literal->getValueDirectly()) == false)
              feasible=true;
          }
        }
        if(verbose){ cout <<"checking literal '" <<*literal <<"' with args "; listWrite(values, cout); cout <<(feasible?" -- good":" -- failed") <<endl; }
        if(!feasible) break;
      }
      if(feasible){
        if(verbose){ cout <<"adding feasible substitution "; listWrite(values, cout); cout <<endl; }
        substitutions.append(values);
        subN++;
      }
    }
  }
  substitutions.reshape(subN,vars.N);

  if(verbose){
    cout <<"POSSIBLE SUBSTITUTIONS:" <<endl;
    for(uint s=0;s<substitutions.d0;s++){
      for(uint i=0;i<substitutions.d1;i++) if(substitutions(s,i)){
        cout <<varScope(i)->keys(0) <<" -> " <<substitutions(s,i)->keys(1) <<", ";
      }
      cout <<endl;
    }
    if(!substitutions.d0) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
  }
  return substitutions;
}


bool forwardChaining_FOL(Graph& KB, Item* query, bool verbose){
  ItemL rules = KB.getItems("Rule");
  ItemL constants = KB.getItems("Constant");
  Graph& state = KB.getItem("STATE")->kvg();

  for(;;){
    KB.checkConsistency();
    bool newFacts=false;
    for(Item *rule:rules){
      if(verbose) cout <<"Testing Rule " <<*rule <<endl;
      ItemL subs = getRuleSubstitutions(state, rule, constants, verbose);
      for(uint s=0;s<subs.d0;s++){
        Item *effect = rule->kvg().last();
        if(verbose){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs[s], cout); cout <<endl; }
        bool e = applyEffectLiterals(state, effect->kvg(), subs[s], &rule->kvg());
        if(verbose){
          if(e) cout <<"NEW STATE = " <<state <<endl;
          else cout <<"DID NOT CHANGE STATE" <<endl;
        }
        newFacts |= e;
        if(e && query){
          if(getEqualFactInKB(state, query)){
            cout <<"SUCCESS!" <<endl;
            return true;
          }
        }
      }
      if(!subs.d0){
        if(verbose) cout <<"NO NEW STATE for this rule" <<endl;
      }
    }
    if(!newFacts) break;
  }
  if(query) cout <<"FAILED" <<endl;
  return false;
}


/// actually propositional logic:
bool forwardChaining_propositional(Graph& KB, Item* q){
  KB.checkConsistency();
  uintA count(KB.N);     count=0;
  boolA inferred(KB.N);  inferred=false;
  ItemL clauses = KB.getItems("Clause");
  ItemL agenda;
  for(Item *clause:clauses){
    count(clause->index) = clause->kvg().N;
    if(!count(clause->index)){ //no preconditions -> facts -> add to 'agenda'
      agenda.append(clause->parents(0));
    }
  }
  cout <<count <<endl;

  while(agenda.N){
    Item *s = agenda.popFirst();
    if(!inferred(s->index)){
      inferred(s->index) = true;
      for(Item *child : s->parentOf){ //all objects that involve 's'
        Item *clause = child->container.isItemOfParentKvg; //check if child is a literal in a clause
        if(clause){ //yes: 's' is a literal in a clause
          CHECK(count(clause->index)>0,"");
          //          if(count(clause->index)>0){ //I think this is always true...
          count(clause->index)--;
          if(!count(clause->index)){ //are all the preconditions fulfilled?
            Item *newFact = clause->parents(0);
            if(newFact==q) return true;
            agenda.append(newFact);
          }
        }
        cout <<count <<endl;
      }
    }
  }
  return false;
}
