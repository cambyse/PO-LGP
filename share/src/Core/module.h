/*  ------------------------------------------------------------------
    Copyright 2016 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or (at
    your option) any later version. This program is distributed without
    any warranty. See the GNU General Public License for more details.
    You should have received a COPYING file of the full GNU General Public
    License along with this program. If not, see
    <http://www.gnu.org/licenses/>
    --------------------------------------------------------------  */


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
typedef Thread Module;
//struct Module;
typedef mlr::Array<Access*> AccessL;
typedef mlr::Array<Module*> ModuleL;
extern Singleton<ConditionVariable> moduleShutdown;

//===========================================================================

Node *getVariable(const char* name);
template <class T> T* getVariable(const char* name){  return dynamic_cast<T*>(registry().get<RevisionedAccessGatedClass*>({"Variable",name}));  }
template <class T> T* getThread(const char* name){  return dynamic_cast<T*>(registry().get<Thread*>({"Thread",name}));  }
RevisionedAccessGatedClassL getVariables();
void openModules();
void stepModules();
void closeModules();
void threadOpenModules(bool waitForOpened, bool setSignalHandler=true);
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

//struct Module : Thread{
//  Module(const char* name=NULL, double beatIntervalSec=-1.):Thread(name, beatIntervalSec){
////    registry().newNode<Module*>({"Module", name}, {}, this);
//  }
//  virtual ~Module(){}
//  virtual void step(){ HALT("you should not run a virtual module"); }
//  virtual void open(){}
//  virtual void close(){}
//};

inline bool operator==(const Module&,const Module&){ return false; }

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
  Module *thread;  ///< which module is this a member of
  RevisionedAccessGatedClass *var;   ///< which variable does it access
  struct Node* registryNode;
  Access(const char* _name, Type *_type, Module *_thread, RevisionedAccessGatedClass *_var):name(_name), type(_type), thread(_thread), var(_var){}
  virtual ~Access(){}
  bool hasNewRevision(){ CHECK(var,"This Access has not been associated to any Variable"); return var->hasNewRevision(); }
  int readAccess(){  CHECK(var,"This Access has not been associated to any Variable"); return var->readAccess((Thread*)thread); }
  int writeAccess(){ CHECK(var,"This Access has not been associated to any Variable"); return var->writeAccess((Thread*)thread); }
  int deAccess(){    CHECK(var,"This Access has not been associated to any Variable"); return var->deAccess((Thread*)thread); }
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

  /// A "copy" of acc: An access to the same variable as acc refers to, but now for '_module'
  Access_typed(Module* _thread, const Access_typed<T>& acc, bool moduleListens=false)
    : Access(acc.name, new Type_typed<T, void>(), _thread, NULL), v(NULL){
    v = acc.v;
    var = acc.var;
    if(thread){
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {thread->registryNode, v->registryNode}, this);
      if(moduleListens) thread->listenTo(*var);
    }else{
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {v->registryNode}, this);
    }
  }

  /// searches for globally registrated variable 'name', checks type equivalence, and becomes an access for '_module'
  Access_typed(Module* _thread, const char* name, bool moduleListens=false)
    : Access(name, new Type_typed<T, void>(), _thread, NULL), v(NULL){
    RevisionedAccessGatedClass** _var = registry().find<RevisionedAccessGatedClass*>({"Variable", name});
    if(!_var){
      v = new Variable<T>(name);
      var = dynamic_cast<RevisionedAccessGatedClass*>(v);
    }else{
      var = *_var;
      v = dynamic_cast<Variable<T>*>(var);
      CHECK(v,"something is wrong");
    }
    if(thread){
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {thread->registryNode, v->registryNode}, this);
      if(moduleListens) thread->listenTo(*var);
    }else{
      registryNode = registry().newNode<Access_typed<T>* >({"Access", name}, {v->registryNode}, this);
    }
  }

  ~Access_typed(){ delete type;  delete registryNode; }
  T& operator()(){ CHECK(v && var,"This Access has not been associated to any Variable"); CHECK(v->rwlock.isLocked(),"");  return v->data; }
  T* operator->(){ CHECK(v && var,"This Access has not been associated to any Variable"); CHECK(v->rwlock.isLocked(),"");  return &(v->data); }
  typename Variable<T>::ReadToken get(){ CHECK(v && var,"");  return v->get((Thread*)thread); } ///< read access to the variable's data
  typename Variable<T>::WriteToken set(){ CHECK(v && var,"");  return v->set((Thread*)thread); } ///< write access to the variable's data
  typename Variable<T>::WriteToken set(const double& dataTime){ CHECK(v && var,"");  return v->set(dataTime, (Thread*)thread); } ///< write access to the variable's data
};

inline bool operator==(const Access&,const Access&){ return false; }


//===========================================================================
//
/** Instead of declaring 'Access_typed<TYPE> name;' as a module
    member, use the macro ACCESS(TYPE, name). This is almost
    equivalent, but automatically assigns the name. */

#if 1

#define ACCESSold(type, name)\
struct __##name##__Access:Access_typed<type>{ \
  __##name##__Access():Access_typed<type>(NULL, #name){} \
} name;

#else

#define ACCESS(type, name) Access_typed<type> name = Access_typed<type>(this, #name);


#endif

#define ACCESS(type, name) Access_typed<type> name = Access_typed<type>(this, #name);
#define ACCESSlisten(type, name) Access_typed<type> name = Access_typed<type>(this, #name, true);
#define ACCESSname(type, name) Access_typed<type> name = Access_typed<type>(NULL, #name);

//===========================================================================
//
/** Use this in the cpp-file to register the module. Once it is
    registered it appears in the global registry, no matter if you
    #included its definition. This allows anyone (the engine) to
    instantiate the module just by referring to its string name. */

#define REGISTER_MODULE(name) \
  RUN_ON_INIT_BEGIN(Decl_Module##_##name) \
  registry().newNode<std::shared_ptr<Type> >({mlr::String("Decl_Module"), mlr::String(#name)}, NodeL(), std::make_shared<Type_typed<name, void> >()); \
  RUN_ON_INIT_END(Decl_Module##_##name)



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
inline void operator<<(ostream& os, const Access& a){ os <<"Access '" <<a.name <<"' from '" <<(a.thread?a.thread->name:mlr::String("NIL")) <<"' to '" << (a.var ? a.var->name : String("??")) <<'\''; }


//===========================================================================
//
// generic recorder module
//

template <class T>
struct Recorder : Module{
  Access_typed<T> access;
  T buffer;
  ofstream fil;

  Recorder(const char* var_name):Module(STRING("Recorder_"<<var_name)), access(this, var_name, true){}

  void open(){
    mlr::open(fil, STRING("z." <<access.name <<'.' <<mlr::getNowString() <<".dat"));
  }
  void step(){
    uint rev = access.readAccess();
    buffer = access();
    double time = access.var->revisionTime();
    access.deAccess();
    mlr::String tag;
    tag.resize(30, false);
    sprintf(tag.p, "%6i %13.6f", rev, time);
    fil <<tag <<' ' <<buffer <<endl;
  }
  void close(){
    fil.close();
  }
};


//===========================================================================
//
// file replayer
//

template<class T>
struct FileReplay : Module{
  Access_typed<T> access;
  T x;
  FileReplay(const char* file_name, const char* var_name, double beatIntervalSec)
    : Module(STRING("FileReplay_"<<var_name), beatIntervalSec),
      access(this, var_name, false) {
    x <<FILE(file_name);
  }
  void open(){}
  void step(){ access.set() = x; }
  void close(){}
};

#endif
