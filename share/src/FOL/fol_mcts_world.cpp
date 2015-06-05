#include <Core/array-vector.h>
#include "fol_mcts_world.h"
#include "fol.h"

void FOL_World::Decision::write(ostream& os) const{
  if(waitDecision){
    os <<"WAIT" <<endl;
  }else{
    os <<"RULE '" <<rule->keys(1) <<"' SUB ";
    listWrite(substitution, os);
    os <<endl;
  }
}

FOL_World::FOL_World(const char* KB_file):KB(*new Graph(KB_file)), state(NULL), tmp(NULL), verbose(4){
  FILE("z.init") <<KB;
  KB.checkConsistency();
  start_state = &KB["START_STATE"]->graph();
  terminal = &KB["terminal"]->graph(); //TODO: replace by QUIT state predicate!
  rules = KB.getItems("Rule");
  constants = KB.getItems("Constant");
  Terminate_keyword = KB["Terminate"];

  reset_state();
}

std::pair<FOL_World::Handle, double> FOL_World::transition(const Handle& action){
  T_step++;
  if(verbose>2) cout <<"****************** FOL_World: step " <<T_step <<endl;
  if(verbose>2){ cout <<"*** pre-state = "; state->write(cout, " "); cout <<endl; }

  const Decision *d = std::dynamic_pointer_cast<const Decision>(action).get();
  if(verbose>2){ cout <<"*** decision = ";  d->write(cout); }
  if(d->waitDecision){
    //-- find minimal wait time
    double w=1e10;
    for(Node *i:*state){
      if(i->getValueType()==typeid(double)){
        double wi = *i->getValue<double>();
        if(w>wi) w=wi;
      }
    }
    if(verbose>2) cout <<"*** real time progress = " <<w <<endl;

    if(w==1e10){
//      HALT("BLA");
      if(verbose>2) cout <<"*** NOTHING TO WAIT FOR!" <<endl;
//      if(forceWait){ cout <<"*** STUCK - NO FEASIBLE SOLUTION FOUND" <<endl;  break; }
    }else{
      //-- subtract w from all times and collect all activities with minimal wait time
      T_real += w;
      NodeL terminatingActivities;
      for(Node *i:*state){
        if(i->getValueType()==typeid(double)){
          double &wi = *i->getValue<double>(); //this is a double reference!
          wi -= w;
          if(fabs(wi)<1e-10) terminatingActivities.append(i);
        }
      }

      //-- for all these activities call the terminate operator
      for(Node *act:terminatingActivities){
        Node *predicate = act->parents(0);
        Node *rule = KB.getChild(Terminate_keyword, predicate);
        if(!rule) HALT("No termination rule for '" <<*predicate <<"'");
        Node *effect = rule->graph().last();
        NodeL vars = getSymbolsOfScope(rule->graph());
        NodeL subs(vars.N); subs.setZero();
        CHECK(vars.N==act->parents.N-1,"");
        for(uint i=0;i<vars.N;i++) subs(i) = act->parents(i+1);

        if(verbose>2) cout <<"*** terminating activity '" <<*act <<"' with rule '" <<*rule <<endl;
        if(verbose>2){ cout <<"*** effect =" <<*effect <<" SUB"; listWrite(subs, cout); cout <<endl; }
        applyEffectLiterals(*state, effect->graph(), subs, &rule->graph());
      }
    }
  }else{
    Node *effect = d->rule->graph().last();
    if(verbose>2){ cout <<"*** effect =" <<*effect <<" SUB"; listWrite(d->substitution, cout); cout <<endl; }
    applyEffectLiterals(*state, effect->graph(), d->substitution, &d->rule->graph());
  }

  if(verbose>2){ cout <<"*** post-state = "; state->write(cout, " "); cout <<endl; }

  return {Handle(new Observation(0)), 0.};
}

const std::vector<FOL_World::Handle> FOL_World::get_actions(){
  if(verbose>2) cout <<"****************** FOL_World: Computing possible decisions" <<flush;
  MT::Array<Handle> decisions; //tuples of rule and substitution
  decisions.append(Handle(new Decision(true, NULL, {}, decisions.N))); //the wait decision (true as first argument, no rule, no substitution)
  for(Node* rule:rules){
    NodeL subs = getRuleSubstitutions(*state, rule, constants, (verbose>4) );
    for(uint s=0;s<subs.d0;s++){
        decisions.append(Handle(new Decision(false, rule, subs[s], decisions.N))); //a grounded rule decision (abstract rule with substution)
    }
  }
  if(verbose>2) cout <<"-- # possible decisions: " <<decisions.N <<endl;
  if(verbose>3) for(Handle& d:decisions) d.get()->write(cout);
//    cout <<"rule " <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl;

  return VECTOR(decisions);
}

const MCTS_Environment::Handle FOL_World::get_state(){
    return Handle(new State());
}

bool FOL_World::is_terminal_state() const{
  //-- test the terminal state
  if(allFactsHaveEqualsInScope(*state, *terminal)){
    if(verbose>0) cout <<"************* FOL_World: TERMINAL STATE FOUND (T_steps=" <<T_step <<", T_real="<<T_real <<") ************" <<endl;
    if(verbose>1){ cout <<"*** FINAL STATE = "; state->write(cout, " "); cout <<endl; }
    return true;
  }
  return false;
}

double FOL_World::get_terminal_reward() const {
  return -T_real;
}

void FOL_World::set_state(const Handle& state){
  NIY;
}

void FOL_World::reset_state(){
  FILE("z.before") <<KB;
  T_step=0;
  T_real=0.;
#if 1
  KB.checkConsistency();
  if(state){
//    state->clear();
    delete state->isItemOfParentKvg;
  }
  state = new Graph();
  state->operator =(*start_state);
#else
  state = start_state;
#endif
  state->isItemOfParentKvg->keys(0)="STATE";
  //  new Node_typed<Graph>(KB, {"STATE"}, {}, new Graph(start_state), true);

  if(tmp) delete tmp->isItemOfParentKvg;
  new Node_typed<Graph>(KB, {"TMP"}, {}, new Graph, true);
  tmp   = &KB["TMP"]->graph();

  KB.checkConsistency();
  FILE("z.after") <<KB;

  if(verbose>1) cout <<"****************** FOL_World: reset_state" <<endl;
  if(verbose>1){ cout <<"*** state = "; state->write(cout, " "); cout <<endl; }
}

bool FOL_World::get_info(InfoTag tag) const{
  switch(tag){
    case hasTerminal: return true;
    case isDeterministic: return true;
    case hasMaxReward: return true;
    case hasMinReward: return true;
    case isMarkov: return true;
    default: HALT("unknown tag" <<tag);
  }
}

double FOL_World::get_info_value(InfoTag tag) const{
  switch(tag){
    case getMaxReward: return 1.;
    case getMinReward: return 0.;
    default: HALT("unknown tag" <<tag);
  }
}
