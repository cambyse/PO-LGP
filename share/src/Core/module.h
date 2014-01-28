/*  ---------------------------------------------------------------------
    Copyright 2013 Marc Toussaint
    email: mtoussai@cs.tu-berlin.de
    
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

struct Access;
struct Module;
typedef MT::Array<Access*> AccessL;

//===========================================================================
//
/** VariableAccess is an abstraction of how a module may access
    variables. Its implementation is engine dependent. Currently a
    shared memory (mutexed and revisioned) is used. ROS topics could
    equally be used. */

struct VariableAccess{
  MT::String name;  ///< Variable name
  Type *type;       ///< Variable type
  void *data;       ///< pointer to data struct; Access_typed knows how to cast it
  timespec tstamp;  ///< timestamp of the data's origin
  VariableAccess(const char* _name):name(_name), type(NULL), data(NULL){}
  virtual int writeAccess(Module*) = 0; ///< tell the engine that a module accesses -> mutex or publish
  virtual int readAccess(Module*) = 0;  ///< tell the engine that a module accesses
  virtual int deAccess(Module*) = 0;    ///< tell the engine that the module de-accesses
  virtual double revisionTime() = 0;
  virtual int revisionNumber() = 0;
  virtual int waitForNextWriteAccess() = 0;
  virtual int waitForRevisionGreaterThan(int rev) = 0; //returns the revision
};


extern Module *currentlyCreating;

//===========================================================================
//
/** This is the core abstraction for users to code modules: derive
    from this class (and perhaps REGISTER_MODULE(...)) to enable the
    engine to instantiate your module and run/schedule it. The
    accesses stores all accesses of this module; the engine can
    automatically analyze them and instantiate respective variables if
    necessary */

struct Module{
  MT::String name;
  AccessL accesses;
  struct ModuleThread *thread;

  /** DON'T open drivers/devices/files or so here in the constructor,
      but in open(). Sometimes a module might be created only to see
      which accesses it needs. The default constructure should really
      do nothing */
  Module(const char* _name=NULL):name(_name), thread(NULL){ currentlyCreating=this; }
  virtual ~Module(){};

  /** The most important method of all of this: step does the actual
      computation of the module. Modules should be state less. Access
      the variables by calling the x.get(), x.set() or
      x.[read|write|de]Access(), where ACCESS(TYPE, x) was
      declared. */
  virtual void step() = 0;

  /** use this to open drivers/devices/files and initialize
      parameters; this is called within the thread */
  virtual void open(){};

  /** use this to close drivers/devices/files; this is called within
      the thread */
  virtual void close(){}; 
  virtual bool test(){ return true; } ///< define a unit test
};


//===========================================================================
//
/** When defining a module you need to declare which variables the
    module needs access to (for reading or writing). This is done by
    declaring members as 'Access_typed<TYPE> name;' instead of 'TYPE
    name;'. The macro ACCESS(TYPE, name); makes this simpler. Access
    is the base class of Access_typed */

struct Access{
  MT::String name; ///< name; by default the access' name; redefine to a variable's name to autoconnect
  Type *type;      ///< type; must be the same as the variable's type
  Module *module;  ///< which module is this a member of
  VariableAccess *var; ///< which variable does it access
  Access(const char* _name):name(_name), type(NULL), module(NULL), var(NULL){}
  int readAccess(){  CHECK(var,""); return var->readAccess(module); }
  int writeAccess(){ CHECK(var,""); return var->writeAccess(module); }
  int deAccess(){    CHECK(var,""); return var->deAccess(module); }
};


/** See Access. This provides read/write 'tokens' (generated by get()
    and set()) which allow convenient and typed read/write access to
    the variable's content */
template<class T>
struct Access_typed:Access{
  struct ReadToken{
    Access_typed<T> *a;
    ReadToken(Access_typed<T> *_a):a(_a){ a->readAccess(); }
    ~ReadToken(){ a->deAccess(); }
    operator const T&(){ return (*a)(); }
    const T& operator()(){ return (*a)(); }
  };
  struct WriteToken{
    Access_typed<T> *a;
    WriteToken(Access_typed<T> *_a):a(_a){ a->writeAccess(); }
    ~WriteToken(){ a->deAccess(); }
    WriteToken& operator=(const T& x){ (*a)() = x; return *this; }
    T& operator()(){ return (*a)(); }
  };

  Access_typed(const char* name, Module *m=NULL, VariableAccess *d=NULL):Access(name){ type=new Type_typed<T, void>();  module=currentlyCreating; var=d; if(module) module->accesses.append(this); }
  T& operator()(){ CHECK(var && var->data,""); return *((T*)var->data); }
  ReadToken get(){ return ReadToken(this); } ///< read access to the variable's data
  WriteToken set(){ return WriteToken(this); } ///< write access to the variable's data
  timespec& tstamp(){ CHECK(var,""); return var->tstamp; } ///< reference to variable's timestamp. Make sure to have locked the variable first.
};


//===========================================================================
//
/** Instead of declaring 'Access_typed<TYPE> name;' as a module
    member, use the macro ACCESS(TYPE, name). This is almost
    equivalent, but automatically assigns the name. */

#define ACCESS(type, name)\
struct __##name##__Access:Access_typed<type>{ \
  __##name##__Access():Access_typed<type>(#name){} \
} name; \


//===========================================================================
//
/** Use this in the cpp-file to register the module. Once it is
    registered it appears in the global registry, no matter if you
    #included its definition. This allows anyone (the engine) to
    instantiate the module just by referring to its string name. */

#define REGISTER_MODULE(name) \
  Item_typed<Type> name##_ModuleRegistryItem(ARRAY<MT::String>(MT::String("Decl_Module"), MT::String(#name)), ItemL(), new Type_typed<name, void>(NULL,NULL), &registry());


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
inline void operator<<(ostream& os, const Access& a){ os <<"Access '" <<a.name <<"' from '" <<a.module->name <<"' to '" << (a.var ? a.var->name : String("??")) <<'\''; }

#endif
