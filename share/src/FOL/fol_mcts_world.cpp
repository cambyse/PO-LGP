#include "fol_mcts_world.h"
#include "fol.h"

#define DEBUG(x) //x

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
      state(NULL), lastDecisionInState(NULL), verbose(0), verbFil(0),
      lastStepReward(0.), lastStepDuration(0.), lastStepProbability(1.), lastStepObservation(0), count(0) {}

FOL_World::FOL_World(istream& is)
    : hasWait(true), gamma(0.9), stepCost(0.1), timeCost(1.), deadEndCost(100.),
      state(NULL), lastDecisionInState(NULL), verbose(0), verbFil(0),
      lastStepReward(0.), lastStepDuration(0.), lastStepProbability(1.), lastStepObservation(0), count(0) {
  init(is);
}

void FOL_World::init(istream& is){
  KB.read(is);
  DEBUG( FILE("z.init") <<KB; ) //write what was read, just for inspection
  KB.checkConsistency();

  start_state = &KB.get<Graph>("START_STATE");
  rewardFct = &KB.get<Graph>("REWARD");
  worldRules = KB.getNodes("Rule");
  decisionRules = KB.getNodes("DecisionRule");
  Terminate_keyword = KB["Terminate"];  CHECK(Terminate_keyword, "You need to declare the Terminate keyword");
  Quit_keyword = KB["QUIT"];            CHECK(Quit_keyword, "You need to declare the QUIT keyword");
  Wait_keyword = KB["WAIT"];            //CHECK(Wait_keyword, "You need to declare the WAIT keyword");
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
    cout <<"*** decisionRules = "; listWrite(decisionRules, cout, "\n"); cout <<endl;
  }
  mlr::open(fil, "z.FOL_World");

  start_T_step=0;
  start_T_real=0.;
  reset_state();
}

FOL_World::~FOL_World(){
}

MCTS_Environment::TransitionReturn FOL_World::transition(const Handle& action){
  lastStepReward = -stepCost;
  lastStepDuration = 0.;
  lastStepProbability = 1.;
  lastStepObservation = 0;

  T_step++;

  CHECK(!hasWait || Wait_keyword,"if the FOL uses wait, the WAIT keyword needs to be declared");


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
    lastStepReward += evaluateFunction(*rewardFct, *state, verbose-3);
  }else{
    if(successEnd) lastStepReward += 100.;
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
      lastStepDuration = 10.;
    }else{
      //-- subtract w from all times and collect all activities with minimal wait time
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

      lastStepDuration = w;
    }
  }else{ //normal decision
    //first check if probabilistic
    Node *effect = d->rule->graph().last();
    if(effect->isOfType<arr>()){
      HALT("probs in decision rules not properly implemented (observation id is not...)");
      arr p = effect->get<arr>();
      uint r = sampleMultinomial(p);
      lastStepProbability = p(r);
      lastStepObservation = lastStepObservation*p.N + r; //raise previous observations to the factor p.N and add current decision
      effect = d->rule->graph().elem(-1-p.N+r);
    }else{
      lastStepProbability = 1.;
    }
    if(verbose>2){ cout <<"*** effect =" <<*effect <<" SUB"; listWrite(d->substitution, cout); cout <<endl; }
    applyEffectLiterals(*state, effect->graph(), d->substitution, &d->rule->graph());

    if(!hasWait) lastStepDuration = 1.;
  }

  T_real += lastStepDuration;
  lastStepReward -= lastStepDuration*timeCost; //cost per real time

  //-- generic world transitioning
  forwardChaining_FOL(*state, worldRules, NULL, NoGraph, verbose-3, &lastStepObservation);

  //-- check for QUIT
  successEnd = getEqualFactInKB(*state, Quit_literal);
  deadEnd = (T_step>100);

  if(deadEnd) lastStepReward -= deadEndCost;

  if(verbose>2){ cout <<"*** post-state = "; state->write(cout, " "); cout <<endl; }
  if(verbFil){
      fil <<"--\n  T_step=" <<T_step;
      fil <<"\n  decision="; d->write(fil);
      fil <<"\n  T_real=" <<T_real;
      fil <<"\n  observation=" <<lastStepObservation;
      fil <<"\n  reward=" <<lastStepReward;
      fil <<"\n  state="; state->write(fil," ","{}"); fil <<endl;
  }

  R_total += lastStepReward;

  return { Handle(new Observation(lastStepObservation)), lastStepReward, lastStepDuration };
}

const std::vector<FOL_World::Handle> FOL_World::get_actions(){
  if(verbose>2) cout <<"****************** FOL_World: Computing possible decisions" <<flush;
  mlr::Array<Handle> decisions; //tuples of rule and substitution
  if(hasWait){
    decisions.append(Handle(new Decision(true, NULL, {}, decisions.N))); //the wait decision (true as first argument, no rule, no substitution)
  }
  for(Node* rule:decisionRules){
    NodeL subs = getRuleSubstitutions2(*state, rule, verbose-3 );
    for(uint s=0;s<subs.d0;s++){
        decisions.append(Handle(new Decision(false, rule, subs[s], decisions.N))); //a grounded rule decision (abstract rule with substution)
    }
  }
  if(verbose>2) cout <<"-- # possible decisions: " <<decisions.N <<endl;
  if(verbose>3) for(Handle& d:decisions){ d.get()->write(cout); cout <<endl; }
//    cout <<"rule " <<d.first->keys(1) <<" SUB "; listWrite(d.second, cout); cout <<endl;
  return conv_arr2stdvec(decisions);
}

bool FOL_World::is_feasible_action(const MCTS_Environment::Handle& action){
  const Decision *d = std::dynamic_pointer_cast<const Decision>(action).get();
  return substitutedRulePreconditionHolds(*state, d->rule, d->substitution);
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
  if(!start_state) start_state = &KB.newSubgraph({"START_STATE"}, state->isNodeOfParentGraph->parents)->value;
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
  DEBUG( FILE("z.before") <<KB; )
  T_step=start_T_step;
  T_real=start_T_real;
  R_total=0.;
  deadEnd=false;
  successEnd=false;

#if 1
  setState(start_state);
#else
  if(!state) state = &KB.newSubgraph({"STATE"}, {start_state->isNodeOfParentGraph})->value;
  state->copy(*start_state);
  DEBUG(KB.checkConsistency();)
#endif

  DEBUG( KB.checkConsistency(); )
  DEBUG( FILE("z.after") <<KB; )

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

Graph* FOL_World::getState(){
  return state;
}

void FOL_World::setState(Graph *s, int setT_step){
  if(!state) state = &KB.newSubgraph({"STATE"}, {s->isNodeOfParentGraph})->value;
  state->copy(*s);
  {  //reqire the parent! NOT NICE!
    Node *n=state->isNodeOfParentGraph;
    n->parents.scalar()->parentOf.removeValue(n);
    n->parents.scalar() = s->isNodeOfParentGraph;
    n->parents.scalar()->parentOf.append(n);
  }
  if(setT_step>=0) T_step = setT_step;
  DEBUG(KB.checkConsistency();)
  CHECK(state->isNodeOfParentGraph && &state->isNodeOfParentGraph->container==&KB,"");
}

Graph* FOL_World::createStateCopy(){
  Graph* new_state = &KB.newSubgraph({STRING("STATE_"<<count++)}, state->isNodeOfParentGraph->parents)->value;
  new_state->copy(*state);
  return new_state;
}

void FOL_World::addAgent(const char* name){
//  Node* n = new Node_typed<bool>(KB, {name}, {}, true); //already exists in kinematic part
  Node* n = KB[name];
  new Node_typed<bool>(*start_state, {}, {KB["agent"], n}, true);
  new Node_typed<bool>(*start_state, {}, {KB["free"], n}, true);
}

void FOL_World::addObject(const char* name){
//  Node* n = new Node_typed<bool>(KB, {name}, {}, true);
  Node* n = KB[name];
  new Node_typed<bool>(*start_state, {}, {KB["object"], n}, true);
}

void FOL_World::addFact(const StringA& symbols){
  NodeL parents;
  for(const mlr::String& s:symbols) parents.append(KB[s]);
  new Node_typed<bool>(*start_state, {}, parents, true);
}
