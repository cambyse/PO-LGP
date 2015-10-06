/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a COPYING file of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>
    -----------------------------------------------------------------  */


/** The purpose of the 'Module' package is to provide a most minimal
    interface for method-programmers to DEFINE Modules. This interface
    is independent from the specific engine or middleware that is
    eventually used to run these modules and organize their
    communication. Here only the declaration of modules is
    standardized. This is also the reason why this file resides in
    'Core' instead of 'System' -- package that want to provide Modules
    only need to load 'Core' -- not any specific lib for running
    modules. In fact, libs providing Modules should NOT link to
    'System'. The design aim is to be as minimalistic as possible. */

#ifndef Core_module_h
#define Core_module_h

#include <Core/array.h>
#include <Core/registry.h>
#include <Core/thread.h>

struct Access;
struct Module;
typedef mlr::Array<Access*> AccessL;
typedef mlr::Array<Module*> ModuleL;
extern Singleton<ConditionVariable> shutdown;

//===========================================================================

Node *getModuleNode(Module*);
Node *getVariable(const char* name);
void openModules();
void stepModules();
void closeModules();
void threadOpenModules(bool waitForOpened);
void threadCloseModules();
void threadCancelModules();
void modulesReportCycleTimes();

//===========================================================================
//
/** This is the core abstraction for users to code modules: derive
    from this class (and perhaps REGISTER_MODULE(...)) to enable the
    engine to instantiate your module and run/schedule it. The
    accesses store all accesses of this module; the engine can
    automatically analyze them and instantiate respective variables if
    necessary */

struct Module : Thread{

  /** DON'T open drivers/devices/files or so here in the constructor,
      but in open(). Sometimes a module might be created only to see
      which accesses it needs. The default constructure should really
      do nothing */
  Module(const char* name=NULL, double beatIntervalSec=-1.):Thread(name, beatIntervalSec){
    new Node_typed<Module>(registry(), {"Module", name}, {}, this, false);
  }
  virtual ~Module(){}

  /** The most important method of all of this: step does the actual
      computation of the module. Modules should be state less. Access
      the variables by calling the x.get(), x.set() or
      x.[read|write|de]Access(), where ACCESS(TYPE, x) was
      declared. */
  virtual void step(){ HALT("you should not run a virtual module"); }

  /** use this to open drivers/devices/files and initialize
      parameters; this is called within the thread */
  virtual void open(){}

  /** use this to close drivers/devices/files; this is called within
      the thread */
  virtual void close(){}
};


//===========================================================================
//
/** When defining a module you need to declare which variables the
    module needs access to (for reading or writing). This is done by
    declaring members as 'Access_typed<TYPE> name;' instead of 'TYPE
    name;'. The macro ACCESS(TYPE, name); makes this simpler. Access
    is the base class of Access_typed */

struct Access{
  mlr::String name; ///< name; by default the access' name; redefine to a variable's name to autoconnect
  Type *type;      ///< type; must be the same as the variable's type
  Module *module;  ///< which module is this a member of
  RevisionedAccessGatedClass *var;   ///< which variable does it access
  Access(const char* _name, Type *_type, Module *_module, RevisionedAccessGatedClass *_var):name(_name), type(_type), module(_module), var(_var){}
  virtual ~Access(){}
  int readAccess(){  CHECK(var,"This Access has not been associated to any Variable"); return var->readAccess((Thread*)module); }
  int writeAccess(){ CHECK(var,"This Access has not been associated to any Variable"); return var->writeAccess((Thread*)module); }
  int deAccess(){    CHECK(var,"This Access has not been associated to any Variable"); return var->deAccess((Thread*)module); }
  int waitForNextRevision(){    CHECK(var,"This Access has not been associated to any Variable"); return var->waitForNextRevision(); }
  int waitForRevisionGreaterThan(int rev){    CHECK(var,"This Access has not been associated to any Variable"); return var->waitForRevisionGreaterThan(rev); }
//  double& tstamp(){ CHECK(var,""); return var->data_time; } ///< reference to the data's time. Variable should be locked while accessing this.
  double& dataTime(){ CHECK(var,""); return var->data_time; } ///< reference to the data's time. Variable should be locked while accessing this.
};


/** See Access. This provides read/write 'tokens' (generated by get()
    and set()) which allow convenient and typed read/write access to
    the variable's content */
template<class T>
struct Access_typed:Access{
  Variable<T> *v;

//  Access_typed(const Access_typed<T>& acc) = delete;

  Access_typed(Module* _module, const Access_typed<T>& acc, bool moduleListens=false)
    : Access(acc.name, new Type_typed<T, void>(), _module, NULL), v(NULL){
    Node *vnode = registry().getNode("Variable", name);
    v = acc.v;
    var = acc.var;
    CHECK(vnode && &vnode->V<Variable<T> >()==v,"something's wrong")
    if(module){
      Node *m = getModuleNode(module);
      new Node_typed<Access_typed<T> >(registry(), {"Access", name}, {m,vnode}, this, false);
      if(moduleListens) module->listenTo(*var);
    }else{
      new Node_typed<Access_typed<T> >(registry(), {"Access", name}, {vnode}, this, false);
    }
  }

  Access_typed(Module* _module, const char* name, bool moduleListens=false)
    : Access(name, new Type_typed<T, void>(), _module, NULL), v(NULL){
    Node *vnode = registry().getNode("Variable", name);
    if(!vnode){
      v = new Variable<T>(name);
      vnode = new Node_typed<Variable<T> >(registry(), {"Variable", name}, {}, v, true);
    }else{
      v = &vnode->V<Variable<T> >();
    }
    var=(RevisionedAccessGatedClass*)v;
    if(module){
      Node *m = getModuleNode(module);
      new Node_typed<Access_typed<T> >(registry(), {"Access", name}, {m,vnode}, this, false);
      if(moduleListens) module->listenTo(*var);
    }else{
      new Node_typed<Access_typed<T> >(registry(), {"Access", name}, {vnode}, this, false);
    }
  }

  ~Access_typed(){ delete type; }
  T& operator()(){ CHECK(v && var,"This Access has not been associated to any Variable"); CHECK(v->rwlock.isLocked(),"");  return v->data; }
  T* operator->(){ CHECK(v && var,"This Access has not been associated to any Variable"); CHECK(v->rwlock.isLocked(),"");  return &(v->data); }
  typename Variable<T>::ReadToken get(){ CHECK(v && var,"");  return v->get((Thread*)module); } ///< read access to the variable's data
  typename Variable<T>::WriteToken set(){ CHECK(v && var,"");  return v->set((Thread*)module); } ///< write access to the variable's data
  typename Variable<T>::WriteToken set(const double& dataTime){ CHECK(v && var,"");  return v->set(dataTime, (Thread*)module); } ///< write access to the variable's data
};

//===========================================================================
//
// dealing with the globally registrated (in moduleSystem) modules
//




//===========================================================================
//
/** Instead of declaring 'Access_typed<TYPE> name;' as a module
    member, use the macro ACCESS(TYPE, name). This is almost
    equivalent, but automatically assigns the name. */

#if 1

#define ACCESS(type, name)\
struct __##name##__Access:Access_typed<type>{ \
  __##name##__Access():Access_typed<type>(NULL, #name){} \
} name;

#else

#define ACCESS(type, name) Access_typed<type> name = Access_typed<type>(this, #name);


#endif

#define ACCESSnew(type, name) Access_typed<type> name = Access_typed<type>(this, #name);
#define ACCESSlisten(type, name) Access_typed<type> name = Access_typed<type>(this, #name, true);
#define ACCESSname(type, name) Access_typed<type> name = Access_typed<type>(NULL, #name);

//===========================================================================
//
/** Use this in the cpp-file to register the module. Once it is
    registered it appears in the global registry, no matter if you
    #included its definition. This allows anyone (the engine) to
    instantiate the module just by referring to its string name. */

#define REGISTER_MODULE(name) \
  RUN_ON_INIT_BEGIN(name) \
  new Node_typed<Type>(registry(), {mlr::String("Decl_Module"), mlr::String(#name)}, NodeL(), new Type_typed<name, void>(NULL,NULL), true); \
  RUN_ON_INIT_END(name)



//===========================================================================
//
/** Macros for a most standard declaration of a module */

#define BEGIN_MODULE(name) \
  struct name : Module { \
    struct s##name *s; \
    name(): Module(#name), s(NULL) {} \
    virtual void open(); \
    virtual void step(); \
    virtual void close();

#define END_MODULE() };

#define FIELD(type, name) type name;

//===========================================================================
//
// dummy pipe operators
//

inline void operator>>(istream&, Module&){ NIY }
inline void operator<<(ostream& os, const Module& m){ os <<"Module '" <<m.name <<'\''; }

inline void operator>>(istream&, Access&){ NIY }
inline void operator<<(ostream& os, const Access& a){ os <<"Access '" <<a.name <<"' from '" <<(a.module?a.module->name:mlr::String("NIL")) <<"' to '" << (a.var ? a.var->name : String("??")) <<'\''; }

#endif
