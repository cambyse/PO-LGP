#include <Core/array-vector.h>
#include "fol_mcts_world.h"
#include "fol.h"

#define DEBUG(x)

void FOL_World::Decision::write(ostream& os) const{
  if(waitDecision){
    os <<"WAIT()";
  }else{
#if 0
    os <<"RULE '" <<rule->keys(1) <<"' SUB ";
    Graph &r=rule->graph();
    for(uint i=0;i<substitution.N;i++){
      os <<r.elem(i)->keys.last() <<'/' <<substitution.elem(i)->keys.last() <<' ';
    }
#else
    os <<rule->keys.last() <<"( ";
    for(uint i=0;i<substitution.N;i++){ if(i) os <<", ";  os <<substitution.elem(i)->keys.last(); }
    os <<" )";
#endif
  }
}

FOL_World::FOL_World(const char* KB_file)
  : stepCost(0.1), timeCost(1.), deadEndCost(100.),
    KB(*new Graph(KB_file)), state(NULL), tmp(NULL), verbose(0), verbFil(0) {
  FILE("z.init") <<KB;
  KB.checkConsistency();
  start_state = &KB["START_STATE"]->graph();
  rewardFct = &KB["REWARD"]->graph();
//  terminal = &KB["terminal"]->graph(); //TODO: replace by QUIT state predicate!
  decisionRules = KB.getNodes("DecisionRule");
  Terminate_keyword = KB["Terminate"];
  Quit_keyword = KB["QUIT"];
  Quit_literal = new Node_typed<bool>(KB, {}, {Quit_keyword}, new bool(true), true);


  if(verbose>1){
    cout <<"****************** FOL_World: creation info:" <<endl;
    cout <<"*** start_state=" <<*start_state <<endl;
    cout <<"*** reward fct=" <<*rewardFct <<endl;
    cout <<"*** decisionRules = "; listWrite(decisionRules, cout); cout <<endl;
  }
  MT::open(fil, "z.FOL_World");

  start_T_step=0;
  start_T_real=0.;
  reset_state();
}

FOL_World::~FOL_World(){
  delete &KB; //the reference new'ed in the constructor
}

std::pair<FOL_World::Handle, double> FOL_World::transition(const Handle& action){
  double reward=0.;
  T_step++;
  reward -= stepCost;

  if(verbose>2) cout <<"****************** FOL_World: step " <<T_step <<endl;
  if(verbose>2){ cout <<"*** pre-state = "; state->write(cout, " "); cout <<endl; }

  const Decision *d = std::dynamic_pointer_cast<const Decision>(action).get();
  if(verbose>2){ cout <<"*** decision = ";  d->write(cout); cout <<endl; }
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
      if(verbose>2) cout <<"*** NOTHING TO WAIT FOR!" <<endl;
      reward -= 10.*timeCost;
    }else{
      //-- subtract w from all times and collect all activities with minimal wait time
      T_real += w;
      reward -= w*timeCost; //cost per real time
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
        NodeL symbols;
        symbols.append(Terminate_keyword);
        symbols.append(act->parents);
        createNewFact(*state, symbols);
      }
    }
  }else{ //normal decision
    //first check if probabilistic
    Node *effect = d->rule->graph().last();
    if(effect->getValueType()==typeid(arr)){
      HALT("probs in decision rules not properly implemented (observation id is not...)");
      arr p = effect->V<arr>();
      uint r = sampleMultinomial(p);
      effect = d->rule->graph().elem(-1-p.N+r);
    }
    if(verbose>2){ cout <<"*** effect =" <<*effect <<" SUB"; listWrite(d->substitution, cout); cout <<endl; }
    applyEffectLiterals(*state, effect->graph(), d->substitution, &d->rule->graph());
  }

#if 1 //generic world transitioning
  int decisionObservation = 0;
  forwardChaining_FOL(KB, NULL, NoGraph, verbose-3, &decisionObservation);
#endif

  //-- check for QUIT
//  successEnd = allFactsHaveEqualsInScope(*state, *terminal);
  successEnd = getEqualFactInKB(*state, Quit_literal);
  deadEnd = (T_step>100);

  //-- check for rewards
  double rValue=0.;
  if(rewardFct) for(Node *rTerm:*rewardFct){
    if(rTerm->getValueType()==typeid(double)) rValue=rTerm->V<double>();
    else{
      CHECK(rTerm->getValueType()==typeid(Graph),"");
      Graph& rCase=rTerm->graph();
      CHECK(rCase.N==1 || rCase.N==2, "");
      if(rCase.N==1){
        CHECK(rCase(0)->getValueType()==typeid(Graph),"");
        if(allFactsHaveEqualsInScope(*state, rCase(0)->graph())) reward += rValue;
      }
      if(rCase.N==2){
        CHECK(rCase(0)->getValueType()==typeid(Graph),"");
        CHECK(rCase(1)->getValueType()==typeid(bool),"");
        if(rCase(1)->parents(0)==d->rule){
          if(allFactsHaveEqualsInScope(*state, rCase(0)->graph())) reward += rValue;
        }
      }
    }
  }else{
    if(successEnd) reward += 100.;
  }

  if(deadEnd) reward -= deadEndCost;

  if(verbose>2){ cout <<"*** post-state = "; state->write(cout, " "); cout <<endl; }
  if(verbFil){
      fil <<"--\n  T_step=" <<T_step;
      fil <<"\n  decision="; d->write(fil);
      fil <<"\n  T_real=" <<T_real;
      fil <<"\n  observation=" <<decisionObservation;
      fil <<"\n  reward=" <<reward;
      fil <<"\n  state="; state->write(fil," ","{}"); fil <<endl;
  }

  R_total += reward;

  return {Handle(new Observation(decisionObservation)), reward};
}

const std::vector<FOL_World::Handle> FOL_World::get_actions(){
  if(verbose>2) cout <<"****************** FOL_World: Computing possible decisions" <<flush;
  MT::Array<Handle> decisions; //tuples of rule and substitution
  decisions.append(Handle(new Decision(true, NULL, {}, decisions.N))); //the wait decision (true as first argument, no rule, no substitution)
  for(Node* rule:decisionRules){
//    NodeL subs = getRuleSubstitutions(*state, rule, constants, (verbose>4) );
    NodeL subs = getRuleSubstitutions2(*state, rule, (verbose>4) );
    for(uint s=0;s<subs.d0;s++){
        decisions.append(Handle(new Decision(false, rule, subs[s], decisions.N))); //a grounded rule decision (abstract rule with substution)
    }
  }
  if(verbose>2) cout <<"-- # possible decisions: " <<decisions.N <<endl;
  if(verbose>3) for(Handle& d:decisions){ d.get()->write(cout); cout <<endl; }
//    cout <<"rule " <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl;
//  Ndecisions=decisions.N;
  return VECTOR(decisions);
}

const MCTS_Environment::Handle FOL_World::get_state(){
    return Handle(new State());
}

bool FOL_World::is_terminal_state() const{
  if(deadEnd){
    if(verbose>0) cout <<"************* FOL_World: DEAD END STATE (T_steps=" <<T_step <<", T_real="<<T_real <<") ************" <<endl;
    if(verbose>1){ cout <<"*** FINAL STATE = "; state->write(cout, " "); cout <<endl; }
    if(verbFil) {
        (*((ofstream*)&fil)) <<"--\n  DEAD END STATE";
        (*((ofstream*)&fil)) <<"\n  total reward=" <<R_total <<endl;
    }
    return true;
  }
  //-- test the terminal state
  if(successEnd){
    if(verbose>0) cout <<"************* FOL_World: SUCCESS STATE FOUND (T_steps=" <<T_step <<", T_real="<<T_real <<") ************" <<endl;
    if(verbose>1){ cout <<"*** FINAL STATE = "; state->write(cout, " "); cout <<endl; }
    if(verbFil) {
        (*((ofstream*)&fil)) <<"--\n  SUCCESS STATE";
        (*((ofstream*)&fil)) <<"\n  total reward=" <<R_total <<endl;
    }
    return true;
  }
  return false;
}

void FOL_World::make_current_state_default() {
  start_state->copy(*state, &KB);
  start_state->isNodeOfParentGraph->keys(0)="START_STATE";
  start_T_step = T_step;
  start_T_real = T_real;
  DEBUG(KB.checkConsistency();)
  if(verbose>1) cout <<"****************** FOL_World: reassign start state" <<endl;
  if(verbose>1){ cout <<"*** start_state = "; start_state->write(cout, " "); cout <<endl; }
  if(verbFil) {
      fil <<"*** reassign start state ***" <<endl;
      fil <<"  start_state="; start_state->write(fil," ","{}"); fil <<endl;
  }
}

void FOL_World::reset_state(){
  FILE("z.before") <<KB;
  T_step=start_T_step;
  T_real=start_T_real;
  R_total=0.;
  deadEnd=false;
  successEnd=false;
  if(!state) state = new Graph();
  state->copy(*start_state, &KB);
  DEBUG(KB.checkConsistency();)
  state->isNodeOfParentGraph->keys(0)="STATE";

  if(tmp) delete tmp->isNodeOfParentGraph;
  new Node_typed<Graph>(KB, {"TMP"}, {}, new Graph, true);
  tmp   = &KB["TMP"]->graph();

  DEBUG(KB.checkConsistency();)
  FILE("z.after") <<KB;

  //-- check for terminal
//  successEnd = allFactsHaveEqualsInScope(*state, *terminal);
  successEnd = getEqualFactInKB(*state, Quit_literal);

  if(verbose>1) cout <<"****************** FOL_World: reset_state" <<endl;
  if(verbose>1){ cout <<"*** state = "; state->write(cout, " "); cout <<endl; }

  if(verbFil){
      fil <<"*** reset ***" <<endl;
      fil <<"  T_step=" <<T_step <<"\n  T_real=" <<T_real <<endl;
      fil <<"  state="; state->write(fil," ","{}"); fil <<endl;
  }
}

bool FOL_World::get_info(InfoTag tag) const{
  switch(tag){
    case hasTerminal: return true;
    case isDeterministic: return true;
    case hasMaxReward: return true;
    case hasMinReward: return true;
    case isMarkov: return true;
    case writeState:{
      cout <<"INFO: deadEnd=" <<deadEnd <<" successEnd=" <<successEnd <<" T_step=" <<T_step <<" T_real=" <<T_real <<" R_total=" <<R_total <<" state=" <<endl;
      state->write(cout," ","{}");
      return true;
    }
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

void FOL_World::write_current_state(ostream& os){
    state->write(os," ","{}");
}
