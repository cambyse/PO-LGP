#include <Core/array-vector.h>
#include "fol_mcts_world.h"
#include "fol.h"

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

FOL_World::FOL_World(const char* KB_file):KB(*new Graph(KB_file)), state(NULL), tmp(NULL), verbose(4){
  FILE("z.init") <<KB;
  KB.checkConsistency();
  start_state = &KB["START_STATE"]->graph();
  terminal = &KB["terminal"]->graph(); //TODO: replace by QUIT state predicate!
  decisionRules = KB.getNodes("DecisionRule");
  constants = KB.getNodes("Constant");
  Terminate_keyword = KB["Terminate"];

  if(verbose>1){
    cout <<"****************** FOL_World: creation info:" <<endl;
    cout <<"*** start_state=" <<*start_state <<endl;
    cout <<"*** terminal query=" <<*terminal <<endl;
    cout <<"*** constants = "; listWrite(constants, cout); cout <<endl;
    cout <<"*** decisionRules = "; listWrite(decisionRules, cout); cout <<endl;
  }
  MT::open(fil, "z.FOL_World");

  reset_state();
}

FOL_World::~FOL_World(){
  delete &KB; //the reference new'ed in the constructor
}

std::pair<FOL_World::Handle, double> FOL_World::transition(const Handle& action){
  double reward=0.;
  T_step++;
  reward -= 0.1; //cost per step

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
      if(Ndecisions==1) deadEnd=true;
    }else{
      //-- subtract w from all times and collect all activities with minimal wait time
      T_real += w;
      reward -= w; //cost per real time
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
#if 0
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
#else
        NodeL symbols;
        symbols.append(Terminate_keyword);
        symbols.append(act->parents);
        createNewFact(*state, symbols);
#endif
      }
    }
  }else{
    //first check if probabilistic
    Node *effect = d->rule->graph().last();
    if(effect->getValueType()==typeid(arr)){
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

  //-- check for terminal
  successEnd = allFactsHaveEqualsInScope(*state, *terminal);

  if(deadEnd) reward -= 100.;
  if(successEnd) reward += 100.;

  if(verbose>2){ cout <<"*** post-state = "; state->write(cout, " "); cout <<endl; }
  fil <<"--\n  T_step=" <<T_step;
  fil <<"\n  decision="; d->write(fil);
  fil <<"\n  T_real=" <<T_real <<"\n  state="; state->write(fil," ","{}");
  fil <<"\n  reward=" <<reward <<endl;

  //reward=0.;
  R_total += reward;

  return {Handle(new Observation(decisionObservation)), reward};
}

const std::vector<FOL_World::Handle> FOL_World::get_actions(){
  if(verbose>2) cout <<"****************** FOL_World: Computing possible decisions" <<flush;
  MT::Array<Handle> decisions; //tuples of rule and substitution
  decisions.append(Handle(new Decision(true, NULL, {}, decisions.N))); //the wait decision (true as first argument, no rule, no substitution)
  for(Node* rule:decisionRules){
    NodeL subs = getRuleSubstitutions(*state, rule, constants, (verbose>4) );
    for(uint s=0;s<subs.d0;s++){
        decisions.append(Handle(new Decision(false, rule, subs[s], decisions.N))); //a grounded rule decision (abstract rule with substution)
    }
  }
  if(verbose>2) cout <<"-- # possible decisions: " <<decisions.N <<endl;
  if(verbose>3) for(Handle& d:decisions){ d.get()->write(cout); cout <<endl; }
//    cout <<"rule " <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl;
  Ndecisions=decisions.N;
  return VECTOR(decisions);
}

const MCTS_Environment::Handle FOL_World::get_state(){
    return Handle(new State());
}

bool FOL_World::is_terminal_state() const{
  if(deadEnd){
    if(verbose>0) cout <<"************* FOL_World: DEAD END STATE (T_steps=" <<T_step <<", T_real="<<T_real <<") ************" <<endl;
    if(verbose>1){ cout <<"*** FINAL STATE = "; state->write(cout, " "); cout <<endl; }
    (*((ofstream*)&fil)) <<"--\n  DEAD END STATE";
    (*((ofstream*)&fil)) <<"\n  total reward=" <<R_total <<endl;
    return true;
  }
  //-- test the terminal state
  if(successEnd){
    if(verbose>0) cout <<"************* FOL_World: SUCCESS STATE FOUND (T_steps=" <<T_step <<", T_real="<<T_real <<") ************" <<endl;
    if(verbose>1){ cout <<"*** FINAL STATE = "; state->write(cout, " "); cout <<endl; }
    (*((ofstream*)&fil)) <<"--\n  SUCCESS STATE";
    (*((ofstream*)&fil)) <<"\n  total reward=" <<R_total <<endl;
    return true;
  }
  return false;
}

void FOL_World::make_current_state_default() {
#if 0
  delete start_state->isNodeOfParentGraph;
  start_state = new Graph();
#endif
  start_state->copy(*state, &KB);
  start_state->isNodeOfParentGraph->keys(0)="START_STATE";
  KB.checkConsistency();
  if(verbose>1) cout <<"****************** FOL_World: reassign start state" <<endl;
  if(verbose>1){ cout <<"*** start_state = "; start_state->write(cout, " "); cout <<endl; }
  fil <<"*** reassign start state ***" <<endl;
  fil <<"  start_state="; start_state->write(fil," ","{}"); fil <<endl;
}

void FOL_World::reset_state(){
  FILE("z.before") <<KB;
  T_step=0;
  T_real=0.;
  R_total=0.;
  deadEnd=false;
  successEnd=false;
  Ndecisions=0;
#if 0
  KB.checkConsistency();
  if(state) delete state->isNodeOfParentGraph;
  state = new Graph();
#endif
  if(!state) state = new Graph();
  state->copy(*start_state, &KB);
  KB.checkConsistency();
  state->isNodeOfParentGraph->keys(0)="STATE";
  //  new Node_typed<Graph>(KB, {"STATE"}, {}, new Graph(start_state), true);

  if(tmp) delete tmp->isNodeOfParentGraph;
  new Node_typed<Graph>(KB, {"TMP"}, {}, new Graph, true);
  tmp   = &KB["TMP"]->graph();

  KB.checkConsistency();
  FILE("z.after") <<KB;

  if(verbose>1) cout <<"****************** FOL_World: reset_state" <<endl;
  if(verbose>1){ cout <<"*** state = "; state->write(cout, " "); cout <<endl; }

  fil <<"*** reset ***" <<endl;
  fil <<"  T_step=" <<T_step <<"\n  T_real=" <<T_real <<"\n  state="; state->write(fil," ","{}"); fil <<endl;
}

// void FOL_World::write_current_state(ostream& os){
//   state->write(os," ","{}");
// }

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