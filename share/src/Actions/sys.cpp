#include "sys.h"

void createSymbolsForShapes(RelationalMachine& RM, const mlr::KinematicWorld& world){
  LOG(1) <<"Shape Symbols:";
  for(mlr::Shape *sh:world.shapes){
    LOG(1) <<"adding symbol for Shape " <<sh->name;
    RM.declareNewSymbol(sh->name.p);
  }
}

void createSymbolsForActivities(RelationalMachine& RM, const Graph& activityRegistry){
  LOG(1) <<"Activity Symbols:";
  for(Node *n:activityRegistry){
    LOG(1) <<"adding symbol for " <<n->keys(0);
    RM.declareNewSymbol(n->keys(0).p);
  }
}
