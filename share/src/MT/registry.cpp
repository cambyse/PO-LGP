#include "registry.h"

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

struct TypeRegistrationSpace{
  MapGraph registry;
  TypeRegistrationL types;
  RWLock lock;
  TypeRegistrationSpace(){ types.memMove=true; }
} *global_TypeRegistrationSpace=NULL;

TypeRegistrationSpace& typeRegistrationSpace(){
  static bool currentlyCreating=false;
  if(currentlyCreating) return *((TypeRegistrationSpace*) NULL);
  if(!global_TypeRegistrationSpace) {
    static Mutex m;
    m.lock();
    if(!global_TypeRegistrationSpace) {
      currentlyCreating=true;
      global_TypeRegistrationSpace = new TypeRegistrationSpace();
      currentlyCreating=false;
    }
    m.unlock();
  }
  return *global_TypeRegistrationSpace;
}

MapGraph& registry(){
  return typeRegistrationSpace().registry;
}

TypeRegistrationL& typeRegistry(){
  return typeRegistrationSpace().types;
}

void reg_report(){
  cout <<"\n +++ TYPES +++" <<endl;
  for_list_(TypeRegistration, it, typeRegistry()) cout <<*it;
}

TypeRegistration* reg_find(const char* type){
  for_list_rev_(TypeRegistration, vi, typeRegistry()){
    if(!strcmp(vi->typeinfo().name(),type)) return vi;
    for(uint j=0;j<vi->keys.N;j++) if(vi->keys(j)==type) return vi;
  }
  return NULL;
}
