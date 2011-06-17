#include "build_mesh_process.h"

Build_mesh_process::Build_mesh_process():Process("build mesh"){
  obj=NULL;
};

void
Build_mesh_process::open(){}

void
Build_mesh_process::step(){
  GraspObject *old;
  GraspObject *copy;

   if (!obj || !obj->o) return;

  /* TODO: To lock or not to lock, that's the question
   */ 

   MT_MSG("Building mesh...");
   obj->writeAccess(this); 
   obj->o->buildMesh();
   obj->deAccess(this);
   

  /*  TODO: Might be more appropriate to not lock. Instead work on a copy of
   *  GraspObject and just set mesh then.:
   */
  
  /* //copy
   * obj->readAccess(this);
   * old = obj->o;
   * copy = (GraspObject*)malloc(sizeof(*obj->o));
   * memcpy(copy, obj->o, sizeof(*obj->o)); // FIX: we need deep copy and this does not copy the X.p aray
   * obj->deAccess(this);
   * 
   * // work on copy 
   * copy->buildMesh();
   * 
   * // set mesh back; if in the meantime changed -- skip 
   * obj->writeAccess(this);
   * if(obj->o == old)  obj->o->m = copy->m;
   * obj->deAccess(this);
   * 
   * free(copy);
   */

  MT::wait(.01);
}

void Build_mesh_process::close(){}

