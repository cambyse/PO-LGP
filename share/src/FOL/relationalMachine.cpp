#include "relationalMachine.h"

RelationalMachine::RelationalMachine(const char* filename):state(NULL), tmp(NULL), verbose(false){
  MT::FileToken fil(filename);
  if(fil.exists()){
    fil >>KB;
    KB.checkConsistency();
  }else{
    MT_MSG("No '"<<filename<<"' for initialization given! This might fail!")
  }
  new Item_typed<Graph>(KB, {"TMP"}, {}, new Graph, true);
  state = &KB["STATE"]->kvg();
  tmp   = &KB["TMP"]->kvg();
}

bool RelationalMachine::queryCondition(MT::String query){
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

ItemL RelationalMachine::fwdChainRules(){
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

MT::String RelationalMachine::getState(){
  MT::String str;
  state->write(str, " ");
  return str;
}

MT::String RelationalMachine::getRules(){
  ItemL rules = KB.getItems("Rule");
  MT::String str;
  listWrite(rules, str, "\n", "[]");
  return str;
}

StringA RelationalMachine::getSymbols(){
  ItemL symbols = getSymbolsOfScope(KB);
  StringA strs(symbols.N);
  for(uint i=0;i<symbols.N;i++){
    strs(i) <<*symbols(i);
  }
  return strs;
}
