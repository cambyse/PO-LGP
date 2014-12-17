#include "fol.h"



ItemL getLiteralsOfScope(Graph& KB){
  ItemL state;
  for(Item *i:KB) if(i->parents.N>0) state.append(i);
  return state;
}

ItemL getVariablesOfScope(Graph& KB){
  ItemL vars;
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

void removeInfeasible(ItemL& domain, Item* literal){
  Graph& G=literal->container.isItemOfParentKvg->container;

  ItemL lit_vars = getVariables(literal);
  CHECK(lit_vars.N==1," remove Infeasible works only for literals with one open variable!");
  Item *var = lit_vars(0);
  Item *predicate = literal->parents(0);
  bool trueValue=true; //check if the literal is negated
  if(literal->getValueType()==typeid(bool)){
    if(*literal->getValue<bool>() == false) trueValue = false;
  }

  ItemL dom;
  for(Item *state_literal:predicate->parentOf) if(&state_literal->container==&G){
    //-- check that all arguments are the same, except for var!
    bool match=true;
    Item *value=NULL;
    for(uint i=0;i<literal->parents.N;i++){
      Item *lit_arg = literal->parents(i);
      Item *state_arg = state_literal->parents(i);
      if(lit_arg==var) value = state_arg;
      else if(lit_arg!=state_arg){ match=false; break; }
    }
    if(match){
      CHECK(value && &value->container==&G,""); //the value should be a constant!
      dom.ItemL::append(value);
    }
  }
  //  cout <<"possible domain of " <<*var <<" BEFORE = " <<GRAPH(domain) <<endl;
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

bool match(Item* fact, Item* literal, const ItemL& subst){
  if(fact->parents.N!=literal->parents.N) return false;
  for(uint i=0;i<literal->parents.N;i++){
    Item *lit_arg = literal->parents(i);
    Item *fact_arg = fact->parents(i);
    if(&lit_arg->container==&literal->container){ //this is a variable -> check match of substitution
      if(subst(lit_arg->index)!=fact_arg) return false;
    }else if(lit_arg!=fact_arg) return false;
  }
  return true;
}


ItemL getFactMatches(Item* literal, ItemL& literals){
  ItemL matches;
  for(Item *lit:literals) if(match(literal,lit)) matches.append(lit);
  return matches;
}


Item* createNewSubstitutedLiteral(Graph& KB, Item* literal, const ItemL& subst){
  Graph& lit_scope = literal->container;
  Item *fact = literal->newClone(KB);
  for(uint i=0;i<fact->parents.N;i++){
    Item *arg=fact->parents(i);
    if(&arg->container==&lit_scope && subst(arg->index)!=NULL){ //is a variable, and subst exists
      fact->parents(i) = subst(arg->index);
      arg->parentOf.removeValue(fact);
      fact->parents(i)->parentOf.append(fact);
    }
  }
  cout <<*fact <<endl;
  return fact;
}


bool checkFeasibility(Item* literal, const ItemL& subst){
  Graph& G=literal->container.isItemOfParentKvg->container;
  Item *predicate = literal->parents(0);
  bool trueValue=true; //check if the literal is negated
  if(literal->getValueType()==typeid(bool)){
    if(*literal->getValue<bool>() == false) trueValue = false;
  }

  for(Item *fact:predicate->parentOf) if(&fact->container==&G){
    if(match(fact, literal, subst)) return trueValue;
  }
  return !trueValue;
}


ItemL getSubstitutions(Graph& rule, ItemL& state, ItemL& constants){
  //-- extract precondition
  ItemL precond;
  for(Item *i:rule){
    if(i->parents.N>0 && i!=rule.last()) //literal <-> degree>0, last literal = outcome
      precond.append(i);
  }
  return getSubstitutions(precond, state, constants);
}


ItemL getSubstitutions(ItemL& literals, ItemL& state, ItemL& constants, bool verbose){
  CHECK(literals.N,"");
  Graph& scope = literals(0)->container; //this is usually a rule (scope = subKvg in which we'll use the indexing)

  ItemL vars = getVariablesOfScope(scope);

  //-- initialize potential domains for each variable
  MT::Array<ItemL> domain(scope.N);
  for(Item *v:vars) domain(v->index) = constants;

  if(verbose) cout <<"domains before 'constraint propagation':" <<endl;
  if(verbose) for(Item *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domain(var->index), cout); cout <<" }" <<endl; }

  //-- grab open variables for each literal
  MT::Array<ItemL> lit_vars(scope.N);
  for(Item *literal:literals) lit_vars(literal->index) = getVariables(literal);

  //-- first pick out all precondition predicates with just one open variable and reduce domains directly
  for(Item *literal:literals){
    if(lit_vars(literal->index).N==1){
      Item *var = lit_vars(literal->index)(0);
      if(verbose) cout <<"checking literal '" <<*literal <<"'" <<flush;
      removeInfeasible(domain(var->index), literal);
      if(verbose){ cout <<" gives remaining domain for '" <<*var <<"' {"; listWrite(domain(var->index), cout); cout <<" }" <<endl; }
      if(domain(var->index).N==0) return ItemL(); //early failure
    }
  }

  if(verbose) cout <<"domains after 'constraint propagation':" <<endl;
  if(verbose) for(Item *var:vars){ cout <<"'" <<*var <<"' {"; listWrite(domain(var->index), cout); cout <<" }" <<endl; }

  //-- for the others, create constraints
  ItemL constraints;
  for(Item *literal:literals){
    if(lit_vars(literal->index).N>1){
      constraints.append(literal);
    }
  }

  if(verbose){ cout <<"remaining constraint literals:" <<endl; listWrite(constraints, cout); cout <<endl; }

  //-- naive CSP: loop through everything
  ItemL substitutions;
  ItemL values(scope.N); values.setZero();
  if(vars.N==1){
    for(Item* value0:domain(vars(0)->index)){
      values(vars(0)->index) = value0;
      substitutions.append(values);
    }
  }
  if(vars.N==2){
    for(Item* value0:domain(vars(0)->index)){
      values(vars(0)->index) = value0;
      for(Item* value1:domain(vars(1)->index)){
        values(vars(1)->index) = value1;
        bool feasible=true;
        for(Item* literal:constraints){
          if(!checkFeasibility(literal, values)){ feasible=false; break; }
        }
        if(feasible) substitutions.append(values);
      }
    }
  }
  if(vars.N==3){
    for(Item* value0:domain(vars(0)->index)){
      values(vars(0)->index) = value0;
      for(Item* value1:domain(vars(1)->index)){
        values(vars(1)->index) = value1;
        for(Item* value2:domain(vars(2)->index)){
          values(vars(2)->index) = value2;
          bool feasible=true;
          for(Item* literal:constraints){
            if(verbose){ cout <<"checking literal '" <<*literal <<"' with args "; listWrite(values, cout); }
            if(!checkFeasibility(literal, values)){
              feasible=false;
              if(verbose) cout <<" -- failed" <<endl;
              break;
            }else{
              if(verbose) cout <<" -- good" <<endl;
            }
          }
          if(feasible){
            if(verbose){ cout <<"adding feasible substitution "; listWrite(values, cout); }
            substitutions.append(values);
          }
        }
      }
    }
  }
  if(vars.N>3){
    NIY;
  }
  substitutions.reshape(substitutions.N/scope.N,scope.N);

  cout <<"POSSIBLE SUBSTITUTIONS:" <<endl;
  for(uint s=0;s<substitutions.d0;s++){
    for(uint i=0;i<substitutions.d1;i++) if(substitutions(s,i)){
      cout <<scope(i)->keys(0) <<" -> " <<substitutions(s,i)->keys(1) <<", ";
    }
    cout <<endl;
  }
  return substitutions;
}


bool forwardChaining_FOL(KeyValueGraph& KB, Item* query){
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
      ItemL subs = getSubstitutions(rule->kvg(), state, constants);
      for(uint s=0;s<subs.d0;s++){
        Item *fact = createNewSubstitutedLiteral(KB, rule->kvg().last(), subs[s]);
        ItemL matches = getFactMatches(fact, state);
        state = getLiteralsOfScope(KB);
        cout <<"new fact = " <<*fact <<endl;
        cout <<"NEW STATE = " <<GRAPH(state) <<endl;
        if(matches.N){
          cout <<"EXISTED -> DELETED!" <<endl;
          delete fact;
        }else{
          newFacts=true;
          if(getFactMatches(query, state).N){
            cout <<"SUCCESS!" <<endl;
            return true;
          }
        }
      }
      if(!subs.N){
        cout <<"NO NEW STATE" <<endl;
      }
//      MT::wait();
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
