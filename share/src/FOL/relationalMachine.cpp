#include "relationalMachine.h"

RelationalMachine::RelationalMachine():state(NULL), tmp(NULL), _log("RelationalMachine", 1, 0){
}

RelationalMachine::RelationalMachine(const char* filename):state(NULL), tmp(NULL), _log("RelationalMachine", 1, 0){
  init(filename);
}

void RelationalMachine::init(const char* filename){
  MT::FileToken fil(filename);
  if(fil.exists()){
    fil >>KB;
    KB.checkConsistency();
  }else{
    LOG(1) <<"No '"<<filename<<"' for initialization given! This might fail!";
  }
  if(!KB["TMP"])   new Node_typed<Graph>(KB, {"TMP"}, {}, new Graph, true);
  if(!KB["STATE"]) new Node_typed<Graph>(KB, {"STATE"}, {}, new Graph(), true);
  state = &KB["STATE"]->graph();
  tmp   = &KB["TMP"]->graph();
}

bool RelationalMachine::queryCondition(MT::String query) const{
  tmp->clear();
  bool q=false;
  try{
    query >>*tmp;
    tmp->checkConsistency();
    q=allFactsHaveEqualsInScope(*state, *tmp);
  }catch(...){
    LOG(-1) <<"queryCondition "<<query <<" -- syntax error of query:" ;
    return false;
  }
  LOG(2) <<"  query=" <<*tmp <<"  outcome=" <<(q?"TRUE":"FALSE");
  return q;
}

bool RelationalMachine::applyEffect(MT::String effect, bool fwdChain){
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
  forwardChaining_FOL(KB, NULL, *tmp, false);
  LOG(2) <<"  changes=" <<*tmp;
  LOG(2) <<"  new state=\n  " <<getState();
  return *tmp;
}

Node *readNode(Graph& containingGraph, std::istream& is, bool verbose, bool parseInfo, MT::String prefixedKey=MT::String());

Node* RelationalMachine::declareNewSymbol(MT::String symbolStr){
  Node *it = readNode(KB, symbolStr, false, false);
  return it;
}

MT::String RelationalMachine::getKB() {
  MT::String str;
  KB.write(str, "\n  ");
  return str;
}

MT::String RelationalMachine::getState() const{
  MT::String str;
  state->write(str, "\n  ");
  return str;
}

MT::String RelationalMachine::getRules(){
  NodeL rules = KB.getNodes("Rule");
  MT::String str;
  listWrite(rules, str, "\n  ", "[]");
  return str;
}

StringA RelationalMachine::getSymbols(){
  NodeL symbols = getSymbolsOfScope(KB);
  StringA strs(symbols.N);
  for(uint i=0;i<symbols.N;i++){
    strs(i) <<*symbols(i);
  }
  return strs;
}
