#include "registry.h"

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

struct TypeRegistrationSpace{
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

TypeRegistrationL& typeRegistrations(){
  return typeRegistrationSpace().types;
}

void registerType(TypeRegistration* v){
  typeRegistrations().append(v);
}

void reg_report(){
  cout <<"\n +++ TYPES +++" <<endl;
  for_list_(TypeRegistration, it, typeRegistrations()) cout <<*it;
}

TypeRegistration* reg_find(const char* type){
  for_list_rev_(TypeRegistration, vi, typeRegistrations()){
    if(!strcmp(vi->typeinfo().name(),type)) return vi;
    for(uint j=0;j<vi->keys.N;j++) if(vi->keys(j)==type) return vi;
  }
  return NULL;
}
