#include "fol.h"

/// given a scope (a subKvg, e.g. the full KB, or a rule or so), return all literals (defined by degree>0)
NodeL getLiteralsOfScope(Graph& KB){
  NodeL state;
  state.anticipateMEM(KB.N);
  for(Node *i:KB) if(i->keys.N==0 && i->parents.N>0) state.append(i);
  return state;
}

/// return all variables (defined by degree=0)
NodeL getSymbolsOfScope(Graph& KB){
  NodeL vars;
  vars.anticipateMEM(KB.N);
  for(Node *i:KB) if(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool)) vars.append(i);
  return vars;
}

Node *getFirstNonSymbolOfScope(Graph& KB){
  for(Node *i:KB) if( !(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool)) ) return i;
  return NULL;
}

/// returns all variables of the literal
NodeL getVariables(Node* literal, Graph* varScope){
  NodeL vars;
  for(Node *i:literal->parents) if(&i->container==varScope){
    CHECK(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool),"");
    vars.append(i);
  }
  return vars;
}

uint getNumOfVariables(Node* literal, Graph* varScope){
  uint v=0;
  for(Node *i:literal->parents) if(&i->container==varScope){
    CHECK(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool),"");
    v++;
  }
  return v;
}

Node *getFirstVariable(Node* literal, Graph* varScope){
  for(Node *i:literal->parents) if(&i->container==varScope){
    CHECK(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool),"");
    return i;
  }
  return NULL;
}

//Node *getFirstSymbol(Node* literal, Graph* varScope){
//  for(Node *i:literal->parents) if(&i->container!=varScope){
//    CHECK(i->keys.N>0 && i->parents.N==0 && i->getValueType()==typeid(bool),"");
//    return i;
//  }
//  return NULL;
//}

/// check if these are literally equal (all arguments are identical, be they vars or consts)
bool factsAreEqual(Node* fact0, Node* fact1, bool checkAlsoValue){
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

/// check if these are literally equal (all arguments are identical, be they vars or consts) -- fact1 is only a tuple, not an node of the graph
bool factsAreEqual(Node *fact0, NodeL& fact1){
  if(fact0->parents.N!=fact1.N) return false;
  for(uint i=0;i<fact0->parents.N;i++){
    if(fact0->parents(i) != fact1(i)) return false;
  }
  return true;
}

/// check match, where all variables of literal are replaced by subst(var->index)
bool factsAreEqual(Node* fact, Node* literal, const NodeL& subst, Graph* subst_scope, bool checkAlsoValue, bool ignoreSubst){
  if(fact->parents.N!=literal->parents.N) return false;
  for(uint i=0;i<fact->parents.N;i++){
    Node *fact_arg = fact->parents(i);
    Node *lit_arg = literal->parents(i);
    if(&lit_arg->container==subst_scope){ //lit_arg is a variable -> check match of substitution
      if(!ignoreSubst && subst(lit_arg->index)!=fact_arg) return false;
    }else if(lit_arg!=fact_arg) return false;
  }
  if(checkAlsoValue){
    if(fact->getValueType()!=literal->getValueType()) return false;
    if(!fact->hasEqualValue(literal)) return false;
  }
  return true;
}

/// try to find a literal within 'scope' that is exactly equal to 'literal'
Node *getEqualFactInKB(Graph& facts, Node *fact, bool checkAlsoValue){
  //first find the section of all facts that derive from the same symbols
  NodeL candidates=fact->parents(0)->parentOf;
//  for(uint p=1;p<fact->parents.N;p++)
//    candidates = setSection(candidates, fact->parents(p)->parentOf);
  //now check only these candidates
  for(Node *fact1:candidates) if(&fact1->container==&facts && fact1!=fact){
    if(factsAreEqual(fact, fact1, checkAlsoValue)) return fact1;
  }
  return NULL;
}

/// try to find a literal within 'scope' that is exactly equal to 'literal'
Node *getEqualFactInKB(Graph& facts, NodeL& fact){
  //first find the section of all facts that derive from the same symbols
  NodeL candidates=fact(0)->parentOf;
//  for(uint p=1;p<fact.N;p++)
//    candidates = setSection(candidates, fact->parents(p)->parentOf);
  //now check only these candidates
  for(Node *fact1:candidates) if(&fact1->container==&facts){
    if(factsAreEqual(fact1, fact)) return fact1;
  }
  return NULL;
}

/// check if subst is a feasible substitution for a literal (by checking with all facts that have same predicate)
Node *getEqualFactInKB(Graph& facts, Node* literal, const NodeL& subst, Graph* subst_scope, bool checkAlsoValue){
  //TODO: this should construct the tuple, call the above method, then additionally checkForValue -> write a method that checks value only
  NodeL candidates = literal->parents(0)->parentOf;
//  NodeL candidates = getLiteralsOfScope(KB);
  for(Node *fact:candidates) if(&fact->container==&facts && fact!=literal){
    if(factsAreEqual(fact, literal, subst, subst_scope, checkAlsoValue)) return fact;
  }
  return NULL;
}

/// check if subst is a feasible substitution for a literal (by checking with all facts that have same predicate)
// NodeL getPotentiallyEqualFactInKB(Graph& facts, Node* literal, Graph* subst_scope, bool checkAlsoValue){
//   Node *rarestSymbol=NULL;
//   uint rarestSymbolN=0;
//   for(Node *sym:literal) if(&sym->container!=subst_scope){ //loop through all grounded symbols, not variables
//     if(!rarestSymbol || sym->parentOf.N<rarestSymbolN){
//       rarestSymbol = sym;
//       rarestSymbolN = sym->parentOf.N;
//     }
//   }
//   NodeL candidates = rarestSymbol->parentOf;
//   NodeL matches;
//   for(Node *fact:candidates) if(&fact->container==&facts && fact!=literal){
//     if(factsAreEqual(fact, literal, {}, subst_scope, checkAlsoValue, true))
//       matches.append(fact);
//   }
//   return matches;
// }

/// return the subset of 'literals' that matches with a fact (calling match(lit0, lit1))
Node *getEqualFactInList(Node* fact, NodeL& facts){
//  NodeL candidates=facts;
//  for(Node *p:fact->parents) candidates = setSection(candidates, p->parentOf);
  for(Node *fact1:facts) if(factsAreEqual(fact, fact1)) return fact1; //matches.append(fact1);
  return NULL;
}

/// check if all facts can be matched with one in scope
//TODO: option value
bool allFactsHaveEqualsInScope(Graph& KB, NodeL& facts){
  for(Node *fact:facts){
    if(!getEqualFactInKB(KB, fact)) return false;
  }
  return true;
}

//TODO: it*->literal*
bool matchingFactsAreEqual(Graph& facts, Node *it1, Node *it2, const NodeL& subst, Graph* subst_scope){
  CHECK(&it1->container==&it2->container,"");
  if(it1->getValueType()!=it2->getValueType()) return false;
  if(it1->parents(0)!=it2->parents(0)) return false;

  Node *m1=getEqualFactInKB(facts, it1, subst, subst_scope, false);
  if(!m1) return false;
  Node *m2=getEqualFactInKB(facts, it2, subst, subst_scope, false);
  if(!m2) return false;

  if(m1==m2) return true;
  return m1->hasEqualValue(m2);
}

/// ONLY for a literal with one free variable: remove all infeasible values from the domain
/// this is meant to be used as basic 'constraint propagation' for order-1 constraints
void removeInfeasibleSymbolsFromDomain(Graph& facts, NodeL& domain, Node* literal, Graph *varScope){
  CHECK(getNumOfVariables(literal, varScope)==1," remove Infeasible works only for literals with one open variable!");
  Node *var = getFirstVariable(literal, varScope);
  Node *predicate = literal->parents(0);

  NodeL dom;
  dom.anticipateMEM(domain.N);
  for(Node *fact:predicate->parentOf) if(&fact->container==&facts){
    //-- check that all arguments are the same, except for var!
    bool match=true;
    Node *value=NULL;
    for(uint i=0;i<literal->parents.N;i++){
      Node *lit_arg = literal->parents(i);
      Node *fact_arg = fact->parents(i);
      if(lit_arg==var) value = fact_arg;
      else if(lit_arg!=fact_arg){ match=false; break; }
    }
    if(match && literal->getValueType()!=typeid(bool)){ //if the literal is boolean, we don't YET check the value (see below)
      if(fact->getValueType()!=literal->getValueType()) match=false;
      match = fact->hasEqualValue(literal);
    }
    if(match){
      CHECK(value && &value->container==&facts.isItemOfParentKvg->container,""); //the value should be a constant!
//      dom.NodeL::setAppendInSorted(value, ItemComp);
      dom.NodeL::append(value);
    }
  }

  //for a negative boolean literal, we REMOVE the matches instead of allowing for them
  if(literal->getValueType()==typeid(bool) && *((bool*)literal->getValueDirectly()) == false){
    setMinus(domain, dom);
  }else{
    domain = setSection(domain, dom);
  }
}


/// directly create a new fact
Node *createNewFact(Graph& facts, const NodeL& symbols){
  return new Node_typed<bool>(facts, {}, symbols, new bool(true), true);
}

/// create a new fact by substituting all variables with subst(var->index) (if non-NULL)
/// add the new literal to KB
Node* createNewSubstitutedLiteral(Graph& facts, Node* literal, const NodeL& subst, Graph* subst_scope){
  Node *fact = literal->newClone(facts);
  for(uint i=0;i<fact->parents.N;i++){
    Node *arg=fact->parents(i);
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

bool applySubstitutedLiteral(Graph& facts, Node* literal, const NodeL& subst, Graph* subst_scope, Graph& changes){
  bool trueValue=true; //check if the literal is negated
  if(literal->getValueType()==typeid(bool)){
    if(*((bool*)literal->getValueDirectly()) == false) trueValue = false;
  }

  bool hasEffects=false;

  //first collect tuple matches
  NodeL matches;
  for(Node *fact:literal->parents(0)->parentOf) if(&fact->container==&facts){
    if(factsAreEqual(fact, literal, subst, subst_scope)) matches.append(fact);
  }

  if(trueValue){
    if(!matches.N){
      Node *newNode = createNewSubstitutedLiteral(facts, literal, subst, subst_scope);
      hasEffects=true;
      if(&changes) newNode->newClone(changes);
    }else{
      for(Node *m:matches){
        if(m->getValueType()==typeid(double)){ //TODO: very special HACK: double add up instead of being assigned
          *m->getValue<double>() += *literal->getValue<double>();
          hasEffects=true;
          if(&changes) m->newClone(changes);
        }else{
          if(!m->hasEqualValue(literal)){
            m->copyValue(literal);
            hasEffects=true;
            if(&changes) m->newClone(changes);
          }
        }
      }
    }
    //TODO: remove double!
  }else{
    //delete all matching facts!
    for(Node *fact:matches){
      hasEffects=true;
      if(&changes){ Node *it=fact->newClone(changes); if(it->getValueType()==typeid(bool)) it->V<bool>()=false; }
      delete fact;
    }
  }
  return hasEffects;
}

bool applyEffectLiterals(Graph& facts, Graph& effects, const NodeL& subst, Graph* subst_scope, Graph& changes){
  bool hasEffects=false;
  for(Node *lit:effects){
    bool e = applySubstitutedLiteral(facts, lit, subst, subst_scope, changes);
    hasEffects = hasEffects || e;
  }
  return hasEffects;
}


/// extracts the preconditions of the rule, then returns substitutions
NodeL getRuleSubstitutions(Graph& facts, Node *rule, NodeL& domain, int verbose){
  //-- extract precondition
  if(verbose>1){ cout <<"Substitutions for rule " <<*rule <<endl; }
  Graph& Rule=rule->graph();
  return getSubstitutions(facts, getFirstNonSymbolOfScope(Rule)->graph(), domain, verbose);
}


/// the list of literals is a conjunctive clause (e.g. precondition)
/// all literals must be in the same scope (element of the same subKvg)
/// we return all feasible substitutions of the literal's variables by constants
/// the return value is an array: for every item of the literal's scope:
/// if item=variable the array contains a pointer to the constant
/// if item=non-variable the arrach contains a NULL pointer
NodeL getSubstitutions(Graph& facts, NodeL& literals, NodeL& domain, int verbose){
  CHECK(literals.N,"");
  Graph& varScope = literals(0)->container.isItemOfParentKvg->container; //this is usually a rule (scope = subKvg in which we'll use the indexing)

  Node* EQ = facts["EQ"];
  NodeL vars = getSymbolsOfScope(varScope);

//  if(!vars.N){
//    cout <<"Substitutions for literals "; listWrite(literals, cout); cout <<" WITHOUT variables!" <<endl;
//    NodeL subs(1u,0u);
//    return subs;
//  }

  if(verbose>2){
    cout <<"Substitutions for literals "; listWrite(literals, cout); cout <<" with variables '"; listWrite(vars, cout); cout <<'\'' <<endl;
//    cout <<"   with facts " <<facts <<" and domain "; listWrite(domain, cout); cout <<'\'' <<endl;
  }

  //-- initialize potential domains for each variable
  MT::Array<NodeL> domainOf(vars.N);
//  constants.sort(ItemComp);
  for(Node *v:vars) domainOf(v->index) = domain;

  if(verbose>3) cout <<"domains before 'constraint propagation':" <<endl;
  if(verbose>3) for(Node *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

  //-- grab open variables for each literal
  uintA lit_numVars(literals(0)->container.N);
  for(Node *literal:literals) lit_numVars(literal->index) = getNumOfVariables(literal, &varScope);

  //-- first pick out all precondition predicates with just one open variable and reduce domains directly
  for(Node *literal:literals){
    if(lit_numVars(literal->index)==1){
      Node *var = getFirstVariable(literal, &varScope);
      if(verbose>3) cout <<"checking literal '" <<*literal <<"'" <<flush;
      removeInfeasibleSymbolsFromDomain(facts, domainOf(var->index), literal, &varScope);
      if(verbose>3){ cout <<" gives remaining domain for '" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }
      if(domainOf(var->index).N==0){
        if(verbose>2) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
        return NodeL(); //early failure
      }
    }
  }

  if(verbose>2) cout <<"domains after 'constraint propagation':" <<endl;
  if(verbose>2) for(Node *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

  //-- for the others, create constraints
  NodeL constraints;
  for(Node *literal:literals){
    if(lit_numVars(literal->index)!=1 || (literal->parents.N && literal->parents(0)==EQ)){
      constraints.append(literal);
    }
  }

  if(verbose>2){ cout <<"remaining constraint literals:" <<endl; listWrite(constraints, cout); cout <<endl; }

  //-- naive CSP: loop through everything
  uint subN=0;
  NodeL substitutions;
  NodeL values(vars.N); values.setZero();
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
      for(Node* literal:constraints){ //loop through all constraints
        if(literal->parents.N && literal->parents(0)==EQ){ //check equality of subsequent literals
          Node *it1 = literal->container(literal->index+1);
          Node *it2 = literal->container(literal->index+2);
          feasible = matchingFactsAreEqual(facts, it1, it2, values, &varScope);
        }else{
          feasible = getEqualFactInKB(facts, literal, values, &varScope);
          if(!feasible){ //when literal is a negative boolean literal and we don't find a match, we interpret this as feasible!
            if(literal->getValueType()==typeid(bool) && *((bool*)literal->getValueDirectly()) == false)
              feasible=true;
          }
        }
        if(verbose>3){ cout <<"checking literal '" <<*literal <<"' with args "; listWrite(values, cout); cout <<(feasible?" -- good":" -- failed") <<endl; }
        if(!feasible) break;
      }
      if(feasible){
        if(verbose>3){ cout <<"adding feasible substitution "; listWrite(values, cout); cout <<endl; }
        substitutions.append(values);
        subN++;
      }
    }
  }
  substitutions.reshape(subN,vars.N);

  if(verbose>1){
    cout <<"POSSIBLE SUBSTITUTIONS: " <<substitutions.d0 <<endl;
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


// NodeL getSubstitutions(Graph& facts, NodeL& literals, bool verbose){
//   CHECK(literals.N,"");
//   Graph& varScope = literals(0)->container.isItemOfParentKvg->container; //this is usually a rule (scope = subKvg in which we'll use the indexing)

//   NodeL vars = getSymbolsOfScope(varScope);

//   if(verbose){
//     cout <<"Substitutions for literals "; listWrite(literals, cout); cout <<" with variables '"; listWrite(vars, cout); cout <<'\'' <<endl;
//   }

//   //-- initialize potential domains for each variable
//   MT::Array<NodeL> domainOf(vars.N);
//   for(Node *l:literals){ //go through all (non-negated) literals...
//     if(l->getValueType()!=typeid(bool) || l->V<bool>()!=false){
//       NodeL matches = getPotentiallyEqualFactInKB(facts, l, true);
//       for(uint i=0;i<l->parents.N;i++){
//         Node *var = l->parents(i);
//         if(&var->container!=subs_scope) continue; //loop over variables only
//         for(Node *m:matches) domainOf(var->index).append(m->parents(i));
//       }
//     }
//   }

//   if(verbose) cout <<"domains before 'constraint propagation':" <<endl;
//   if(verbose) for(Node *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

//   //-- grab open variables for each literal
//   uintA lit_numVars(literals(0)->container.N);
//   for(Node *literal:literals) lit_numVars(literal->index) = getNumOfVariables(literal, &varScope);

//   //-- first pick out all precondition predicates with just one open variable and reduce domains directly
//   for(Node *literal:literals){
//     if(lit_numVars(literal->index)==1){
//       Node *var = getFirstVariable(literal, &varScope);
//       if(verbose) cout <<"checking literal '" <<*literal <<"'" <<flush;
//       removeInfeasibleSymbolsFromDomain(facts, domainOf(var->index), literal, &varScope);
//       if(verbose){ cout <<" gives remaining domain for '" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }
//       if(domainOf(var->index).N==0){
//         if(verbose) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
//         return NodeL(); //early failure
//       }
//     }
//   }

//   if(verbose) cout <<"domains after 'constraint propagation':" <<endl;
//   if(verbose) for(Node *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domainOf(var->index), cout); cout <<" }" <<endl; }

//   //-- for the others, create constraints
//   NodeL constraints;
//   for(Node *literal:literals){
//     if(lit_numVars(literal->index)!=1 || (literal->parents.N && literal->parents(0)==EQ)){
//       constraints.append(literal);
//     }
//   }

//   if(verbose){ cout <<"remaining constraint literals:" <<endl; listWrite(constraints, cout); cout <<endl; }

//   //-- naive CSP: loop through everything
//   uint subN=0;
//   NodeL substitutions;
//   NodeL values(vars.N); values.setZero();
//   {
//     //-- using 'getIndexTuple' we can linearly enumerate all configurations of all variables
//     uintA domainN(vars.N);
//     for(uint i=0;i<vars.N;i++) domainN(i) = domainOf(i).N;  //collect dims/cardinalities of domains
//     uint configurationsN = product(domainN); //number of all possible configurations
//     for(uint config=0;config<configurationsN;config++){ //loop through all possible configurations
//       uintA valueIndex = getIndexTuple(config, domainN);
//       bool feasible=true;
//       for(uint i=0;i<vars.N;i++) values(vars(i)->index) = domainOf(i)(valueIndex(i)); //assign the configuration
//       //only allow for disjoint assignments
//       for(uint i=0; i<values.N && feasible; i++) for(uint j=i+1; j<values.N && feasible; j++){
//         if(values(i)==values(j)) feasible=false;
//       }
//       if(!feasible) continue;
//       for(Node* literal:constraints){ //loop through all constraints
//         if(literal->parents.N && literal->parents(0)==EQ){ //check equality of subsequent literals
//           Node *it1 = literal->container(literal->index+1);
//           Node *it2 = literal->container(literal->index+2);
//           feasible = matchingFactsAreEqual(facts, it1, it2, values, &varScope);
//         }else{
//           feasible = getEqualFactInKB(facts, literal, values, &varScope);
//           if(!feasible){ //when literal is a negative boolean literal and we don't find a match, we interpret this as feasible!
//             if(literal->getValueType()==typeid(bool) && *((bool*)literal->getValueDirectly()) == false)
//               feasible=true;
//           }
//         }
//         if(verbose){ cout <<"checking literal '" <<*literal <<"' with args "; listWrite(values, cout); cout <<(feasible?" -- good":" -- failed") <<endl; }
//         if(!feasible) break;
//       }
//       if(feasible){
//         if(verbose){ cout <<"adding feasible substitution "; listWrite(values, cout); cout <<endl; }
//         substitutions.append(values);
//         subN++;
//       }
//     }
//   }
//   substitutions.reshape(subN,vars.N);

//   if(verbose){
//     cout <<"POSSIBLE SUBSTITUTIONS: " <<substitutions.d0 <<endl;
//     for(uint s=0;s<substitutions.d0;s++){
//       for(uint i=0;i<substitutions.d1;i++) if(substitutions(s,i)){
//         cout <<varScope(i)->keys(0) <<" -> " <<substitutions(s,i)->keys(1) <<", ";
//       }
//       cout <<endl;
//     }
//     if(!substitutions.d0) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
//   }
//   return substitutions;
// }


bool forwardChaining_FOL(Graph& KB, Node* query, Graph& changes, int verbose){
  NodeL rules = KB.getItems("Rule");
  NodeL constants = KB.getItems("Constant");
  Graph& state = KB.getItem("STATE")->graph();

  for(;;){
    KB.checkConsistency();
    bool newFacts=false;
    for(Node *rule:rules){
      if(verbose>1) cout <<"Testing Rule " <<*rule <<endl;
      NodeL subs = getRuleSubstitutions(state, rule, constants, verbose);
      for(uint s=0;s<subs.d0;s++){
        Node *effect = rule->graph().last();
        if(effect->getValueType()==typeid(arr)){ //TODO: THIS IS SAMPLING!!! SOMEHOW MAKE THIS CLEAR/transparent/optional or so
          arr p = effect->V<arr>();
          uint r = sampleMultinomial(p);
          effect = rule->graph().elem(-1-p.N+r);
        }
        if(verbose>0){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs[s], cout); cout <<endl; }
        bool e = applyEffectLiterals(state, effect->graph(), subs[s], &rule->graph(), changes);
        if(verbose>1){
          if(e){
            cout <<"NEW STATE = " <<state <<endl;
            if(&changes) cout <<"CHANGES = " <<changes <<endl;
          }
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
        if(verbose>1) cout <<"NO NEW STATE for this rule" <<endl;
      }
    }
    if(!newFacts) break;
  }
  if(query) cout <<"FAILED" <<endl;
  return false;
}


/// actually propositional logic:
bool forwardChaining_propositional(Graph& KB, Node* q){
  KB.checkConsistency();
  uintA count(KB.N);     count=0;
  boolA inferred(KB.N);  inferred=false;
  NodeL clauses = KB.getItems("Clause");
  NodeL agenda;
  for(Node *clause:clauses){
    count(clause->index) = clause->graph().N;
    if(!count(clause->index)){ //no preconditions -> facts -> add to 'agenda'
      agenda.append(clause->parents(0));
    }
  }
  cout <<count <<endl;

  while(agenda.N){
    Node *s = agenda.popFirst();
    if(!inferred(s->index)){
      inferred(s->index) = true;
      for(Node *child : s->parentOf){ //all objects that involve 's'
        Node *clause = child->container.isItemOfParentKvg; //check if child is a literal in a clause
        if(clause){ //yes: 's' is a literal in a clause
          CHECK(count(clause->index)>0,"");
          //          if(count(clause->index)>0){ //I think this is always true...
          count(clause->index)--;
          if(!count(clause->index)){ //are all the preconditions fulfilled?
            Node *newFact = clause->parents(0);
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
