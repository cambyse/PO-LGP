#include "fol_mcts_world.h"
#include "fol.h"

#define DEBUG(x) x

void FOL_World::Decision::write(ostream& os) const{
  if(waitDecision){
    os <<"(WAIT)";
  }else{
#if 0
    os <<"RULE '" <<rule->keys(1) <<"' SUB ";
    Graph &r=rule->graph();
    for(uint i=0;i<substitution.N;i++){
      os <<r.elem(i)->keys.last() <<'/' <<substitution.elem(i)->keys.last() <<' ';
    }
#else
    os <<'(' <<rule->keys.last();
    for(uint i=0;i<substitution.N;i++){ os <<' ' <<substitution.elem(i)->keys.last(); }
    os <<')' <<flush;
#endif
  }
}

FOL_World::FOL_World()
    : hasWait(true), gamma(0.9), stepCost(0.1), timeCost(1.), deadEndCost(100.),
      state(NULL), lastDecisionInState(NULL), tmp(NULL), verbose(0), verbFil(0),
      generateStateTree(false),
      lastStepDuration(0.), lastStepProbability(1.), count(0) {}

FOL_World::FOL_World(istream& is)
    : hasWait(true), gamma(0.9), stepCost(0.1), timeCost(1.), deadEndCost(100.),
      state(NULL), lastDecisionInState(NULL), tmp(NULL), verbose(0), verbFil(0),
      generateStateTree(false),
      lastStepDuration(0.), lastStepProbability(1.), count(0) {
  init(is);
}

void FOL_World::init(istream& is){
  KB.read(is);
  FILE("z.init") <<KB; //write what was read, just for inspection
  KB.checkConsistency();

  start_state = &KB.get<Graph>("START_STATE");
  rewardFct = &KB.get<Graph>("REWARD");
  worldRules = KB.getNodes("Rule");
  decisionRules = KB.getNodes("DecisionRule");
  Terminate_keyword = KB["Terminate"];  CHECK(Terminate_keyword, "You need to declare the Terminate keyword");
  Quit_keyword = KB["QUIT"];            CHECK(Quit_keyword, "You need to declare the QUIT keyword");
  Wait_keyword = KB["WAIT"];            CHECK(Wait_keyword, "You need to declare the WAIT keyword");
  Quit_literal = new Node_typed<bool>(KB, {}, {Quit_keyword}, true);

  Graph *params = KB.find<Graph>("FOL_World");
  if(params){
    hasWait = params->get<bool>("hasWait", hasWait);
    gamma = params->get<double>("gamma", gamma);
    stepCost = params->get<double>("stepCost", stepCost);
    timeCost = params->get<double>("timeCost", timeCost);
    deadEndCost = params->get<double>("deadEndCost", deadEndCost);
  }

  if(verbose>1){
    cout <<"****************** FOL_World: creation info:" <<endl;
    cout <<"*** start_state=" <<*start_state <<endl;
    cout <<"*** reward fct=" <<*rewardFct <<endl;
    cout <<"*** worldRules = "; listWrite(worldRules, cout); cout <<endl;
    cout <<"*** decisionRules = "; listWrite(decisionRules, cout); cout <<endl;
  }
  mlr::open(fil, "z.FOL_World");

  start_T_step=0;
  start_T_real=0.;
  reset_state();
}

FOL_World::~FOL_World(){
}

std::pair<FOL_World::Handle, double> FOL_World::transition(const Handle& action){
  double reward=0.;
  T_step++;
  reward -= stepCost;

  //-- store the old state; make a new state that is child of the old
  if(generateStateTree){
    Node *new_state = KB.appendSubgraph({STRING("STATE"<<count++)}, {state->isNodeOfParentGraph});
    new_state->graph().copy(*state);
    state = &new_state->graph();
    DEBUG(KB.checkConsistency());
  }

  if(verbose>2) cout <<"****************** FOL_World: step " <<T_step <<endl;
  if(verbose>2){ cout <<"*** pre-state = "; state->write(cout, " "); cout <<endl; }

  //-- get the decision
  const Decision *d = std::dynamic_pointer_cast<const Decision>(action).get();
  if(verbose>2){ cout <<"*** decision = ";  d->write(cout); cout <<endl; }

  //-- remove state annotations from state, if exists
  for(uint i=state->N;i--;){
    Node *n=state->elem(i);
    if(n->keys.N) delete n;
  }

  //-- remove the old decision-fact, if exists (Obsolete - implicit in the above)
//#if 0
//  if(lastDecisionInState) delete lastDecisionInState;
//#else
//  for(uint i=state->N;i--;){
//    Node *n=state->elem(i);
//    if(n->parents.N && n->parents.first()->keys.N && n->parents.first()->keys.first()=="DecisionRule") delete n;
//  }
//#endif

  //-- add the decision as a fact
  if(!d->waitDecision){
    NodeL decisionTuple = {d->rule};
    decisionTuple.append(d->substitution);
    lastDecisionInState = createNewFact(*state, decisionTuple);
    lastDecisionInState->keys.append("decision");
  }else{
    lastDecisionInState = createNewFact(*state, {Wait_keyword});
    lastDecisionInState->keys.append("decision");
  }

  //-- check for rewards
  if(rewardFct){
    reward += evaluateFunction(*rewardFct, *state, verbose-3);

#if 0
  double rValue=0.;
  if(rewardFct) for(Node *rTerm:*rewardFct){
    if(rTerm->isOfType<double>()) rValue=rTerm->get<double>();
    else{
      CHECK(rTerm->isGraph(),"");
      Graph& rCase=rTerm->graph();
#if 0
      if(rCase.N==1){
        CHECK(rCase(0)->isGraph(),"");
        if(allFactsHaveEqualsInScope(*state, rCase(0)->graph())) reward += rValue;
      }
      if(rCase.N>=2){
        CHECK(rCase.last(-2)->isGraph(),"");
        CHECK(rCase.last(-1)->isOfType<bool>(),"");
        if(rCase.last(-1)->parents(0)==d->rule){
          if(allFactsHaveEqualsInScope(*state, rCase(0)->graph())) reward += rValue;
        }
      }
#else
      NodeL subs = getRuleSubstitutions2(*state, rTerm, 0);
      if(rCase.last()->isOfType<double>() && rCase.last()->keys.last()=="count"){
        if(subs.d0 == rCase.last()->get<double>()) reward += rValue;
      }else{
        if(subs.d0) reward += rValue;
      }
#endif
    }
#endif
  }else{
    if(successEnd) reward += 100.;
  }

  //-- apply effects of decision
  if(d->waitDecision){
    CHECK(hasWait,"");

    //-- find minimal wait time
    double w=1e10;
    for(Node *i:*state){
      if(i->isOfType<double>()){
        double wi = i->get<double>();
        if(w>wi) w=wi;
      }
    }
    if(verbose>2) cout <<"*** real time progress = " <<w <<endl;

    if(w==1e10){
      if(verbose>2) cout <<"*** NOTHING TO WAIT FOR!" <<endl;
      reward -= 10.*timeCost;
      lastStepDuration=10.;
    }else{
      //-- subtract w from all times and collect all activities with minimal wait time
      T_real += w;
      reward -= w*timeCost; //cost per real time
      lastStepDuration=w;
      NodeL terminatingActivities;
      for(Node *i:*state){
        if(i->isOfType<double>()){
          double &wi = i->get<double>(); //this is a double reference!
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
    if(effect->isOfType<arr>()){
      HALT("probs in decision rules not properly implemented (observation id is not...)");
      arr p = effect->get<arr>();
      uint r = sampleMultinomial(p);
      effect = d->rule->graph().elem(-1-p.N+r);
    }
    if(verbose>2){ cout <<"*** effect =" <<*effect <<" SUB"; listWrite(d->substitution, cout); cout <<endl; }
    applyEffectLiterals(*state, effect->graph(), d->substitution, &d->rule->graph());
  }

  //-- generic world transitioning
  int decisionObservation = 0;
  forwardChaining_FOL(*state, worldRules, NULL, NoGraph, verbose-3, &decisionObservation);

  //-- check for QUIT
//  successEnd = allFactsHaveEqualsInScope(*state, *terminal);
  successEnd = getEqualFactInKB(*state, Quit_literal);
  deadEnd = (T_step>100);


  //-- delete decision fact again
  //if(decision) delete decision;

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
  mlr::Array<Handle> decisions; //tuples of rule and substitution
  if(hasWait){
    decisions.append(Handle(new Decision(true, NULL, {}, decisions.N))); //the wait decision (true as first argument, no rule, no substitution)
  }
  for(Node* rule:decisionRules){
//    NodeL subs = getRuleSubstitutions(*state, rule, constants, (verbose>4) );
    NodeL subs = getRuleSubstitutions2(*state, rule, verbose-3 );
    for(uint s=0;s<subs.d0;s++){
        decisions.append(Handle(new Decision(false, rule, subs[s], decisions.N))); //a grounded rule decision (abstract rule with substution)
    }
  }
  if(verbose>2) cout <<"-- # possible decisions: " <<decisions.N <<endl;
  if(verbose>3) for(Handle& d:decisions){ d.get()->write(cout); cout <<endl; }
//    cout <<"rule " <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl;
//  Ndecisions=decisions.N;
  return conv_arr2stdvec(decisions);
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
  if(!start_state) start_state = &newSubGraph(KB,{"START_STATE"},state->isNodeOfParentGraph->parents)->value;
  start_state->copy(*state);
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
  if(!state) state = &KB.appendSubgraph({"STATE"}, {})->value;
  state->copy(*start_state);
  DEBUG(KB.checkConsistency();)

  if(tmp) delete tmp->isNodeOfParentGraph;
  KB.appendSubgraph({"TMP"}, {});
  tmp   = &KB["TMP"]->graph();

  DEBUG(KB.checkConsistency();)
  FILE("z.after") <<KB;

  //-- forward chain rules
  forwardChaining_FOL(KB, KB.get<Graph>("STATE"), NULL, NoGraph, verbose-3); //, &decisionObservation);

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
    case getGamma: return gamma;
    case getMaxReward: return 100.;
    case getMinReward: return -deadEndCost;
    default: HALT("unknown tag" <<tag);
  }
}

void FOL_World::write_state(ostream& os){
  state->write(os," ","{}");
}

void FOL_World::set_state(mlr::String& s){
  state->clear();
  s >>"{";
  state->read(s);
}

Graph*FOL_World::getState(){
  return state;
}

void FOL_World::setState(Graph *s){
  state = s;
  CHECK(state->isNodeOfParentGraph && &s->isNodeOfParentGraph->container==&KB,"");
}


void FOL_World::addAgent(const char* name){
//  Node* n = new Node_typed<bool>(KB, {name}, {}, true); //already exists in kinematic part
  Node* n = KB[name];
  new Node_typed<bool>(*state, {}, {KB["agent"], n}, true);
  new Node_typed<bool>(*state, {}, {KB["free"], n}, true);
}

void FOL_World::addObject(const char* name){
//  Node* n = new Node_typed<bool>(KB, {name}, {}, true);
  Node* n = KB[name];
  new Node_typed<bool>(*state, {}, {KB["object"], n}, true);
}

void FOL_World::addFact(const StringA& symbols){
  NodeL parents;
  for(const mlr::String& s:symbols) parents.append(KB[s]);
  new Node_typed<bool>(*state, {}, parents, true);
}
