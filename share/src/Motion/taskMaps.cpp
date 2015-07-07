#include "taskMaps.h"

TaskMap *newTaskMap(const Graph& specs, const ors::KinematicWorld& world){
  TaskMap *map;
  MT::String type = specs.V<MT::String>("type", "pos");
  if(type=="wheels"){
    map = new TaskMap_qItself(world, "worldTranslationRotation");
  }else if(type=="collisionIneq"){
    map = new CollisionConstraint(specs.V<double>("margin", 0.1) );
  }else if(type=="proxy"){
    map = new ProxyTaskMap(allPTMT, {0u}, specs.V<double>("margin", 0.1) );
  }else if(type=="qItself"){
    if(specs["ref1"]) map = new TaskMap_qItself(world, specs["ref1"]->V<MT::String>());
    else if(specs["Hmetric"]) map = new TaskMap_qItself(specs["Hmetric"]->V<double>()*world.getHmetric());
    else map = new TaskMap_qItself();
  }else if(type=="GJK_vec"){
    map = new TaskMap_GJK(world, specs);
  }else{
    map = new DefaultTaskMap(specs, world);
  }
  return map;
}

TaskMap_GJK::TaskMap_GJK(const ors::KinematicWorld& W, const Graph& specs){
  Node *it;
  if((it=specs["ref1"])){ auto name=it->V<MT::String>(); auto *s=W.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); i=s->index; }
  if((it=specs["ref2"])){ auto name=it->V<MT::String>(); auto *s=W.getShapeByName(name); CHECK(s,"shape name '" <<name <<"' does not exist"); j=s->index; }
  if((it=specs["vec1"])) vec1 = ors::Vector(it->V<arr>());  else vec1.setZero();
  if((it=specs["vec2"])) vec2 = ors::Vector(it->V<arr>());  else vec2.setZero();
}

void TaskMap_GJK::phi(arr& y, arr& J, const ors::KinematicWorld& W, int t){
  ors::Shape *s1 = i<0?NULL: W.shapes(i);
  ors::Shape *s2 = j<0?NULL: W.shapes(j);
  CHECK(s1 && s2,"");
  ors::Vector p1, p2;
  GJK_sqrDistance(s1->mesh, s2->mesh, s1->X, s2->X, p1, p2, NoVector, NoVector, NoPointType, NoPointType);
  arr y2, J2;
  W.kinematicsPos(y,  (&J?J :NoArr), s1->body, s1->body->X.rot/(p1-s1->body->X.pos));
  W.kinematicsPos(y2, (&J?J2:NoArr), s2->body, s2->body->X.rot/(p2-s2->body->X.pos));
  y -= y2;
  y -= ARRAY(vec1);
  if(&J) J -= J2;

//  ors::Proxy *p = new ors::Proxy();
//  p->a=i; p->b=j; p->posA=p1; p->posB=p2; p->colorCode=1;
//  ((ors::KinematicWorld*)&W)->proxies.append(p);
}
