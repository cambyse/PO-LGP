#include "taskMaps.h"

TaskMap *newTaskMap(const Graph& specs, const ors::KinematicWorld& world){
  TaskMap *map;
  MT::String type = specs.V<MT::String>("type", "pos");
  if(type=="wheels"){
    map = new TaskMap_qItself(world, "worldTranslationRotation");
  }else if(type=="collisionIneq"){
    map = new CollisionConstraint(specs.V<double>("margin", 0.1) );
  }else if(type=="qItself"){
    if(specs["ref1"]) map = new TaskMap_qItself(world, specs["ref1"]->V<MT::String>());
    else map = new TaskMap_qItself();
  }else{
    map = new DefaultTaskMap(specs, world);
  }
  return map;
}
