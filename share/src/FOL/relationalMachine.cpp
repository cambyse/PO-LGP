#include "relationalMachine.h"

RelationalMachine::RelationalMachine():state(NULL), tmp(NULL), verbose(false){
}

RelationalMachine::RelationalMachine(const char* filename):state(NULL), tmp(NULL), verbose(false){
  init(filename);
}

void RelationalMachine::init(const char* filename){
  MT::FileToken fil(filename);
  if(fil.exists()){
    fil >>KB;
    KB.checkConsistency();
  }else{
    MT_MSG("No '"<<filename<<"' for initialization given! This might fail!")
  }
  new Node_typed<Graph>(KB, {"TMP"}, {}, new Graph, true);
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
    MT_MSG("queryCondition "<<query <<" -- syntax error of query:" );
    return false;
  }
  if(verbose){
    cout <<__FUNCTION__ <<":";
    cout <<"\n  query="; tmp->write(cout, " ");
    cout <<"\n  outcome=" <<(q?"TRUE":"FALSE") <<endl;
  }
  return q;
}

bool RelationalMachine::applyEffect(MT::String effect){
  tmp->clear();
  bool e=false;
  try{
    effect >>*tmp;
    tmp->checkConsistency();
    e = applyEffectLiterals(*state, *tmp, {}, NULL);
  }catch(...){
    MT_MSG("applyEffect "<<effect <<" -- syntax error of query");
//    return false;
  }
  if(verbose){
    cout <<__FUNCTION__ <<":";
    cout <<"\n  effects="; tmp->write(cout, " ");
    cout <<"\n  new state="; state->write(cout, " ");
    cout <<endl;
  }
  return e;
}

NodeL RelationalMachine::fwdChainRules(){
  tmp->clear();
  forwardChaining_FOL(KB, NULL, *tmp, false);
  if(verbose){
    cout <<__FUNCTION__ <<":";
    cout <<"\n  changes="; tmp->write(cout, " ");
    cout <<"\n  new state="; state->write(cout, " ");
    cout <<endl;
  }
  return *tmp;
}

Node *readItem(Graph& containingKvg, std::istream& is, bool verbose, bool parseInfo, MT::String prefixedKey=MT::String());

Node* RelationalMachine::declareNewSymbol(MT::String symbol){
  Node *it = readItem(KB, symbol, false, false);
  return it;
}

MT::String RelationalMachine::getKB() {
  MT::String str;
  KB.write(str, " ");
  return str;
}

MT::String RelationalMachine::getState(){
  MT::String str;
  state->write(str, " ");
  return str;
}

MT::String RelationalMachine::getRules(){
  NodeL rules = KB.getItems("Rule");
  MT::String str;
  listWrite(rules, str, "\n", "[]");
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
