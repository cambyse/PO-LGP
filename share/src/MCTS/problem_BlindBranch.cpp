
#include "problem_BlindBranch.h"

BlindBranch::BlindBranch(uint H):H(H){
  actions = { Handle(new Action(-1)), Handle(new Action(+1)) };
}

void BlindBranch::reset_state(){ state=0; T=0; }

std::pair<MCTS_Environment::Handle, double> BlindBranch::transition(const MCTS_Environment::Handle& action){
  state += std::dynamic_pointer_cast<const Action>(action)->d;
  T++;
  double r=0.;
  if(is_terminal_state()) r = (double)state/H;
  return {Handle(NULL), r};
}

std::pair<MCTS_Environment::Handle, double>  BlindBranch::transition_randomly(){
  if(mlr::rnd.uni()<.5) return transition(actions(0));
  return transition(actions(1));
}

const std::vector<MCTS_Environment::Handle> BlindBranch::get_actions(){
  return conv_arr2stdvec(actions);
}

const MCTS_Environment::Handle BlindBranch::get_state(){
  return MCTS_Environment::Handle(new State(state, T));
}

void BlindBranch::set_state(const MCTS_Environment::Handle& _state){
  auto s = std::dynamic_pointer_cast<const State>(_state);
  state = s->sum;
  T = s->T;
}

bool BlindBranch::is_terminal_state() const{ return T>=H; }


bool BlindBranch::get_info(InfoTag tag) const{
  switch(tag){
    case hasTerminal: return true;
    case isDeterministic: return true;
    case hasMaxReward: return true;
    case hasMinReward: return true;
    case isMarkov: return true;
    default: HALT("unknown tag" <<tag);
  }
}
double BlindBranch::get_info_value(InfoTag tag) const{
  switch(tag){
    case getMaxReward: return 1.;
    case getMinReward: return 0.;
    default: HALT("unknown tag" <<tag);
  }
}
