#include "fol.h"

ItemL getLiteralsOfScope(Graph& KB){
  ItemL state;
  state.anticipateMEM(KB.N);
  for(Item *i:KB) if(i->keys.N==0 && i->parents.N>0) state.append(i);
  return state;
}

ItemL getVariablesOfScope(Graph& KB){
  ItemL vars;
  vars.anticipateMEM(KB.N);
  for(Item *i:KB) if(i->parents.N==0 && i!=KB.last()) vars.append(i);
  return vars;
}

ItemL getVariables(Item* literal){
  ItemL vars;
  for(Item *i:literal->parents)
    if(&i->container==&literal->container){
      CHECK(i->parents.N==0,"");
      vars.append(i);
    }
  return vars;
}

uint getNumOfVariables(Item* literal){
  uint v=0;
  for(Item *i:literal->parents)
    if(&i->container==&literal->container){
      CHECK(i->parents.N==0,"");
      v++;
    }
  return v;
}

Item *getFirstVariable(Item* literal){
  for(Item *i:literal->parents)
    if(&i->container==&literal->container){
      CHECK(i->parents.N==0,"");
      return i;
    }
  return NULL;
}

void removeInfeasible(ItemL& domain, Item* literal, bool checkAlsoValue){
  Graph& G=literal->container.isItemOfParentKvg->container;

  CHECK(getNumOfVariables(literal)==1," remove Infeasible works only for literals with one open variable!");
  Item *var = getFirstVariable(literal);
  Item *predicate = literal->parents(0);
  bool trueValue=true; //check if the literal is negated
  if(literal->getValueType()==typeid(bool)){
    if(*((bool*)literal->getValueDirectly()) == false) trueValue = false;
    checkAlsoValue=false;
  }

  ItemL dom;
  dom.anticipateMEM(domain.N);
  for(Item *fact:predicate->parentOf) if(&fact->container==&G){
    //-- check that all arguments are the same, except for var!
    bool match=true;
    Item *value=NULL;
    for(uint i=0;i<literal->parents.N;i++){
      Item *lit_arg = literal->parents(i);
      Item *fact_arg = fact->parents(i);
      if(lit_arg==var) value = fact_arg;
      else if(lit_arg!=fact_arg){ match=false; break; }
    }
    if(match && checkAlsoValue){
      if(fact->getValueType()!=literal->getValueType()) match=false;
      match = fact->hasEqualValue(literal);
    }
    if(match){
      CHECK(value && &value->container==&G,""); //the value should be a constant!
//      dom.ItemL::setAppendInSorted(value, ItemComp);
      dom.ItemL::append(value);
    }
  }
  //  cout <<"possible domain of " <<*var <<" BEFORE = " <<GRAPH(domain) <<endl;
//  if(trueValue) domain = setSectionSorted(domain, dom, ItemComp); // = setSection(domain, dom);
//  else setMinusSorted(domain, dom, ItemComp);
  if(trueValue) domain = setSection(domain, dom);
  else setMinus(domain, dom);
  //  cout <<"possible domain of " <<*var <<" AFTER = " <<GRAPH(domain) <<endl;
}

bool match(Item* literal0, Item* literal1){
  if(literal0->parents.N!=literal1->parents.N) return false;
  for(uint i=0;i<literal0->parents.N;i++){
    if(literal0->parents(i) != literal1->parents(i)) return false;
  }
  return true;
}

Item *getMatchInScope(Item *literal, Graph* scope){
  CHECK(&literal->container!=scope,"if the literal is in the scope, this does not make sense to ask");
  Item *predicate=literal->parents(0);
  for(Item *lit:predicate->parentOf) if(&lit->container==scope){
    if(match(literal, lit)) return lit;
  }
  return NULL;
}

bool checkAllMatchesInScope(ItemL& literals, Graph* scope){
  for(Item *lit:literals){
    if(!getMatchInScope(lit, scope)) return false;
  }
  return true;
}

bool match(Item* fact, Item* literal, const ItemL& subst, Graph* subst_scope,bool checkAlsoValue){
  if(fact->parents.N!=literal->parents.N) return false;
  for(uint i=0;i<literal->parents.N;i++){
    Item *lit_arg = literal->parents(i);
    Item *fact_arg = fact->parents(i);
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

ItemL getFactMatches(Item* literal, ItemL& literals){
  ItemL matches;
  for(Item *lit:literals) if(match(literal,lit)) matches.append(lit);
  return matches;
}


Item* createNewSubstitutedLiteral(Graph& KB, Item* literal, const ItemL& subst, Graph* subst_scope){
  Item *fact = literal->newClone(KB);
  for(uint i=0;i<fact->parents.N;i++){
    Item *arg=fact->parents(i);
    CHECK(&arg->container==subst_scope || &arg->container==&KB,"the literal argument should be a constant (KB scope) or variable (1st level local scope)");
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

bool applySubstitutedLiteral(Graph& KB, Item* literal, const ItemL& subst, Graph* subst_scope){
  bool trueValue=true; //check if the literal is negated
  if(literal->getValueType()==typeid(bool)){
    if(*((bool*)literal->getValueDirectly()) == false) trueValue = false;
  }

  bool hasEffects=false;

  //first collect tuple matches
  ItemL matches;
  for(Item *fact:literal->parents(0)->parentOf) if(&fact->container==&KB){
    if(match(fact, literal, subst, subst_scope)) matches.append(fact);
  }

  if(trueValue){
    if(!matches.N){
      createNewSubstitutedLiteral(KB, literal, subst, subst_scope);
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

bool applyEffectLiterals(Graph& KB, Item* effectliterals, const ItemL& subst, Graph* subst_scope){
  CHECK(effectliterals->getValueType()==typeid(KeyValueGraph), "");
  KeyValueGraph &effects = *effectliterals->getValue<KeyValueGraph>();
  bool hasEffects=false;
  for(Item *lit:effects){
    bool e = applySubstitutedLiteral(KB, lit, subst, subst_scope);
    hasEffects = hasEffects || e;
  }
  return hasEffects;
}

bool checkTruth(Item* literal, const ItemL& subst, Graph* subst_scope){
  Graph& KB=literal->container.isItemOfParentKvg->container;
  Item *predicate = literal->parents(0);
  bool trueValue=true; //check if the literal is negated
  bool checkAlsoValue=true;
  if(literal->getValueType()==typeid(bool)){
    if(*((bool*)literal->getValueDirectly()) == false) trueValue = false;
    checkAlsoValue=false;
  }

#if 1
  for(Item *fact:predicate->parentOf) if(&fact->container==&KB){
#else
  ItemL state = getLiteralsOfScope(KB);
  for(Item *fact:state) if(&fact->container==&KB){
#endif
    if(match(fact, literal, subst, subst_scope, checkAlsoValue)) return trueValue;
  }
  return !trueValue;
}

bool checkEquality(Item* it1, Item* it2, const ItemL& subst, Graph* subst_scope){
  CHECK(&it1->container==&it2->container,"");
  if(it1->getValueType()!=it2->getValueType()) return false;
  if(it1->parents(0)!=it2->parents(0)) return false;
  Graph& KB=it1->container.isItemOfParentKvg->container;
  Item *predicate = it1->parents(0);

  //find 1st match
  Item *m1=NULL, *m2=NULL;
  for(Item *fact:predicate->parentOf) if(&fact->container==&KB){
    if(match(fact, it1, subst, subst_scope, false)){ m1=fact; break; }
  }
  if(!m1) return false;
  //find 2nd match
  for(Item *fact:predicate->parentOf) if(&fact->container==&KB){
    if(match(fact, it2, subst, subst_scope, false)){ m2=fact; break; }
  }
  if(!m2) return false;

  if(m1==m2) return true;
  return m1->hasEqualValue(m2);
}


ItemL getRuleSubstitutions(Item *rule, ItemL& state, ItemL& constants, bool verbose){
  //-- extract precondition
  if(verbose){ cout <<"Substitutions for rule " <<*rule <<endl; }
  Graph& Rule=rule->kvg();
  ItemL precond;
  precond.anticipateMEM(Rule.N);
  for(Item *i:Rule){
    if(i->parents.N>0 && i!=Rule.last()) //literal <-> degree>0, last literal = outcome
      precond.append(i);
  }
  return getSubstitutions(precond, state, constants, verbose);
}


ItemL getSubstitutions(ItemL& literals, ItemL& state, ItemL& constants, bool verbose){
  CHECK(literals.N,"");
  Graph& scope = literals(0)->container; //this is usually a rule (scope = subKvg in which we'll use the indexing)

  Item* EQ = state(0)->container["EQ"];
  ItemL vars = getVariablesOfScope(scope);

  if(verbose){ cout <<"Substitutions for literals "; listWrite(literals, cout); cout <<" with variables "; listWrite(vars, cout); cout <<endl; }


  //-- initialize potential domains for each variable
  MT::Array<ItemL> domain(vars.N);
//  constants.sort(ItemComp);
  for(Item *v:vars) domain(v->index) = constants;

  if(verbose) cout <<"domains before 'constraint propagation':" <<endl;
  if(verbose) for(Item *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domain(var->index), cout); cout <<" }" <<endl; }

  //-- grab open variables for each literal
  uintA lit_numVars(scope.N);
  for(Item *literal:literals) lit_numVars(literal->index) = getNumOfVariables(literal);

  //-- first pick out all precondition predicates with just one open variable and reduce domains directly
  for(Item *literal:literals){
    if(lit_numVars(literal->index)==1){
      Item *var = getFirstVariable(literal);
      if(verbose) cout <<"checking literal '" <<*literal <<"'" <<flush;
      removeInfeasible(domain(var->index), literal, true);
      if(verbose){ cout <<" gives remaining domain for '" <<*var <<"' {"; listWrite(domain(var->index), cout); cout <<" }" <<endl; }
      if(domain(var->index).N==0){
        if(verbose) cout <<"NO POSSIBLE SUBSTITUTIONS" <<endl;
        return ItemL(); //early failure
      }
    }
  }

  if(verbose) cout <<"domains after 'constraint propagation':" <<endl;
  if(verbose) for(Item *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domain(var->index), cout); cout <<" }" <<endl; }

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
    for(uint i=0;i<vars.N;i++) domainN(i) = domain(i).N;  //collect dims/cardinalities of domains
    uint configurationsN = product(domainN); //number of all possible configurations
    for(uint config=0;config<configurationsN;config++){ //loop through all possible configurations
      uintA valueIndex = getIndexTuple(config, domainN);
      bool feasible=true;
      for(uint i=0;i<vars.N;i++) values(vars(i)->index) = domain(i)(valueIndex(i)); //assign the configuration
      //only allow for disjoint assignments
      for(uint i=0; i<values.N && feasible; i++) for(uint j=i+1; j<values.N && feasible; j++){
        if(values(i)==values(j)) feasible=false;
      }
      if(!feasible) continue;
      for(Item* literal:constraints){ //loop through all constraints
        if(literal->parents.N && literal->parents(0)==EQ){ //check equality of subsequent literals
          Item *it1 = literal->container(literal->index+1);
          Item *it2 = literal->container(literal->index+2);
          feasible = checkEquality(it1, it2, values, &scope);
        }else{
          feasible = checkTruth(literal, values, &scope);
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
        cout <<scope(i)->keys(0) <<" -> " <<substitutions(s,i)->keys(1) <<", ";
      }
      cout <<endl;
    }
  }
  return substitutions;
}


bool forwardChaining_FOL(KeyValueGraph& KB, Item* query, bool verbose){
  //  KB.checkConsistency();
  //  uintA count(KB.N);     count=0;
  //  boolA inferred(KB.N);  inferred=false;
  //  ItemL clauses = KB.getItems("Clause");
  //  ItemL agenda;
  //  for(Item *clause:clauses){
  //    count(clause->index) = clause->kvg().N;
  //    if(!count(clause->index)){ //no preconditions -> facts -> add to 'agenda'
  //      agenda.append(clause->parents(0));
  //    }
  //  }
  //  cout <<count <<endl;
  ItemL rules = KB.getItems("Rule");
  ItemL constants = KB.getItems("Constant");
  ItemL state = getLiteralsOfScope(KB);

  for(;;){
    KB.checkConsistency();
    bool newFacts=false;
    for(Item *rule:rules){
      if(verbose) cout <<"Testing Rule " <<*rule <<endl;
      ItemL subs = getRuleSubstitutions(rule, state, constants, verbose);
      for(uint s=0;s<subs.d0;s++){
        Item *effect = rule->kvg().last();
        if(verbose){ cout <<"*** applying" <<*effect <<" SUBS"; listWrite(subs[s], cout); cout <<endl; }
        bool e = applyEffectLiterals(KB, effect, subs[s], &rule->kvg());
        state = getLiteralsOfScope(KB);
        if(verbose){
          if(e) cout <<"NEW STATE = " <<GRAPH(state) <<endl;
          else cout <<"DID NOT CHANGE STATE" <<endl;
        }
        newFacts |= e;
        if(e){
          if(getFactMatches(query, state).N){
            cout <<"SUCCESS!" <<endl;
            return true;
          }
        }

//        Item *fact = createNewSubstitutedLiteral(KB, rule->kvg().last(), subs[s], &rule->kvg());
//        ItemL matches = getFactMatches(fact, state);
//        state = getLiteralsOfScope(KB);
//        cout <<"new fact = " <<*fact <<endl;
//        cout <<"NEW STATE = " <<GRAPH(state) <<endl;
//        if(matches.N){
//          cout <<"EXISTED -> DELETED!" <<endl;
//          delete fact;
//        }else{
//          newFacts=true;
//          if(getFactMatches(query, state).N){
//            cout <<"SUCCESS!" <<endl;
//            return true;
//          }
//        }
      }
      if(!subs.d0){
        if(verbose) cout <<"NO NEW STATE for this rule" <<endl;
      }
//      if(verbose) MT::wait();
    }
    if(!newFacts) break;
  }
  cout <<"FAILED" <<endl;
  return false;
}


bool forwardChaining_propositional(KeyValueGraph& KB, Item* q){
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
