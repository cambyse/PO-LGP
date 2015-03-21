#include "relationalMachine.h"

RelationalMachine::RelationalMachine(const char* filename):state(NULL), tmp(NULL){
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
  try{
    query >>*tmp;
    tmp->checkConsistency();
  }catch(...){
    MT_MSG("queryCondition "<<query <<" -- syntax error of query:" );
    return false;
  }

  return allFactsHaveEqualsInScope(*state, *tmp);
}

bool RelationalMachine::applyEffect(MT::String effect){
  tmp->clear();
  try{
    effect >>*tmp;
    tmp->checkConsistency();
  }catch(...){
    MT_MSG("applyEffect "<<effect <<" -- syntax error of query");
    return false;
  }

  return applyEffectLiterals(*state, *tmp, {}, NULL);
}

bool RelationalMachine::fwdChainRules(){
  return forwardChaining_FOL(KB, NULL, false);
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
