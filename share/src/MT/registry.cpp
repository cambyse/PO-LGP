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
  uint i;
  TypeRegistration *vi;
  for_list_rev(i,vi,typeRegistrations()){
    cout
      <<"Type ";
    if(vi->key) cout <<" key=" <<vi->key;
    cout <<" username=" <<vi->userType
        <<" sysname=" <<vi->sysType <<endl;
  }
}

TypeRegistration* reg_find(const char* type){
  uint i;
  TypeRegistration *vi;
  for_list_rev(i,vi,typeRegistrations()){
    if(!strcmp(vi->key,type) || !strcmp(vi->sysType,type) || !strcmp(vi->userType,type)) return vi;
  }
  return NULL;
}
