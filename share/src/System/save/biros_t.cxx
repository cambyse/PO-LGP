/**
 * @file
 * @ingroup group_biros
 */
/**
 * @addtogroup group_biros
 * @{
 */
#include "biros.h"

template<class T> T* Biros::getVariable(const char* name, Module *p, bool required) {
  writeAccess(p);
  Variable *raw = listFindByName(variables, name); // NULL if not found
  T *v = dynamic_cast<T*>(raw); // NULL if cast fails because of RTTI
  deAccess(p);
  if (!v && raw) { HALT(name << " which is asked for by " << (p?p->name:STRING("NULL")) << " is of wrong type."); }
  else if (!raw) {
    if(required) { HALT("can't find required biros variable '" <<name <<"' -- Process '" <<(p?p->name:STRING("NULL")) <<"' will not work"); }
    else MT_MSG("can't find biros variable '" <<name <<"' -- Process '" <<(p?p->name:STRING("NULL")) <<"' will not connect");
  }
  //else { MT_MSG("Autoconnect Process '" << (p?p->name:STRING("NULL")) <<"' with biros variable '" << name << "'.");}
  return v;
}

template<class T> T* Biros::getOrCreateVariable(const char* name, Module *p){
  T *v = getVariable<T>(name, p, false);
  if(!v) v = new T(name);
  return v;
}

template<class T> void Biros::getVariable(T*& v, const char* name, Module *p, bool required){
  v = getVariable<T>(name, p, required);
}

template<class T>  T* Biros::getProcess(const char* name, Module *caller, bool required) {
  writeAccess(caller);
  Process *raw = listFindByName(processes, name); // NULL if not found
  T *p = dynamic_cast<T*>(raw); // NULL if cast fails because of RTTI
  deAccess(caller);
  if (!p && raw) { HALT(name << " which is asked for by " << (caller?caller->name:STRING("NULL")) << " is of wrong type."); }
  else if (!raw) {
    if(required) { HALT("can't find required biros process'" <<name <<"' -- Caller '" <<(caller?caller->name:STRING("NULL")) <<"' will not work"); }
    else MT_MSG("can't find biros process '" <<name <<"' -- Caller '" <<(p?p->name:STRING("NULL")) <<"' will not connect");
  }
  return p;
}

template<class T> T Biros::getParameter(const char *name, const T &_default, Module *p) {
  Parameter_typed<T> *par;
  writeAccess(p);
  par = (Parameter_typed<T>*)listFindByName(parameters, name);
  deAccess(p);
  if (!par) par = new Parameter_typed<T>(name, _default);
  if (!par->dependers.contains(p)) par->dependers.append(p);
  return par->value;
}

template<class T> T Biros::getParameter(const char *name, Module *p) {
  return getParameter<T>(name, *((T*)NULL), p);
}

template<class T> void Biros::setParameter(const char *name, T value) {
  Process *p = getProcessFromPID();
  Parameter_typed<T> *par;
  writeAccess(p->module);
  par = (Parameter_typed<T>*)listFindByName(parameters, name);
  deAccess(p->module);
  if (!par) MT_MSG("WARNING: cannot find " <<name
                   <<" in parameters, nothing is changed.");
  par->value = value;
}
/** @} */
