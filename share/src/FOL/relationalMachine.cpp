#include "relationalMachine.h"

RelationalMachine::RelationalMachine()
  : state(NULL), tmp(NULL), _log("RelationalMachine", 1, 0){
}

RelationalMachine::RelationalMachine(const char* filename)
  : state(NULL), tmp(NULL), _log("RelationalMachine", 1, 0){
  init(filename);
}

void RelationalMachine::init(const char* filename){
  mlr::FileToken fil(filename);
  if(fil.exists()){
    fil >>KB;
    KB.checkConsistency();
  }
  if(!KB["TMP"])   KB.newSubgraph({"TMP"}, {});
  if(!KB["STATE"]) KB.newSubgraph({"STATE"}, {});
  state = &KB["STATE"]->graph();
  tmp   = &KB["TMP"]->graph();
}

bool RelationalMachine::queryCondition(mlr::String query) const{
  tmp->clear();
  bool q=false;
  try{
    query >>*tmp;
    tmp->checkConsistency();
    q=allFactsHaveEqualsInKB(*state, *tmp);
  }catch(...){
    LOG(-1) <<"queryCondition "<<query <<" -- syntax error of query:" ;
    return false;
  }
  LOG(2) <<"  query=" <<*tmp <<"  outcome=" <<(q?"TRUE":"FALSE");
  return q;
}

bool RelationalMachine::applyEffect(mlr::String effect, bool fwdChain){
  tmp->clear();
  bool e=false;
  try{
    effect >>*tmp;
    tmp->checkConsistency();
    e = applyEffectLiterals(*state, *tmp, {}, NULL);
  }catch(...){
    LOG(-1) <<"applyEffect "<<effect <<" -- syntax error of query";
//    return false;
  }
  LOG(1) <<"  effects=" <<*tmp;
  LOG(2) <<"  new state=\n  " <<getState();
  if(fwdChain) fwdChainRules();
  return e;
}

bool RelationalMachine::applyEffect(Node* literal, bool fwdChain){
  bool e = applySubstitutedLiteral(*state, literal, {}, NULL);
  LOG(1) <<"  effects=" <<*literal;
  LOG(2) <<"  new state=\n  " <<getState();
  if(fwdChain) fwdChainRules();
  return e;
}

NodeL RelationalMachine::fwdChainRules(){
  tmp->clear();
  forwardChaining_FOL(KB, KB.get<Graph>("STATE"), NULL, *tmp, false);
  LOG(2) <<"  changes=" <<*tmp;
  LOG(2) <<"  new state=\n  " <<getState();
  return *tmp;
}

Node* RelationalMachine::declareNewSymbol(mlr::String symbolStr){
  Node *it = KB.readNode(symbolStr);
  return it;
}

mlr::String RelationalMachine::getKB() {
  mlr::String str;
  KB.write(str, "\n  ");
  return str;
}

mlr::String RelationalMachine::getState() const{
  mlr::String str;
  state->write(str, "\n  ");
  return str;
}

mlr::String RelationalMachine::getRules() const{
  NodeL rules = KB.getNodes("Rule");
  mlr::String str;
  listWrite(rules, str, "\n  ", "[]");
  return str;
}

StringA RelationalMachine::getSymbols() const{
  NodeL symbols = getSymbolsOfScope(KB);
  StringA strs(symbols.N);
  for(uint i=0;i<symbols.N;i++){
    strs(i) <<*symbols(i);
  }
  return strs;
}
