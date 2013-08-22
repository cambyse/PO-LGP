#ifndef Core_module_h
#define Core_module_h

#include <Core/array.h>
#include <Core/registry.h>

struct Access;
struct Module;
typedef MT::Array<Access*> AccessL;

//===========================================================================
//
// an abstraction that needs to be realized by an engine
//

struct DataAccess{
  MT::String name;            ///< Variable name
  Type *type;
  void *data;
  DataAccess(const char* _name):name(_name), type(NULL), data(NULL){}
  virtual int writeAccess(Module*) = 0;
  virtual int readAccess(Module*) = 0;
  virtual int deAccess(Module*) = 0;
};


//===========================================================================
//
// Module base class
//

extern Module *currentlyCreating;

struct Module{
  MT::String name;
  AccessL accesses;
  void *thread;
  Module(const char* _name=NULL):name(_name), thread(NULL){ currentlyCreating=this; }
  virtual ~Module(){};
  virtual void step() = 0;
  virtual void open(){};
  virtual void close(){};
  virtual bool test(){ return true; }
};


//===========================================================================
//
// modules communicate to variables via an Access
//

struct Access{
  MT::String name;
  Type *type;
  Module *module;
  DataAccess *var;
  Access(const char* _name):name(_name), type(NULL), module(NULL), var(NULL){}
};


template<class T>
struct Access_typed:Access{
  struct ReadToken{
    Access_typed<T> *a;
    ReadToken(Access_typed<T> *_a):a(_a){ a->readAccess(); }
    ~ReadToken(){ a->deAccess(); }
    const T& operator()(){ return (*a)(); }
  };
  struct WriteToken{
    Access_typed<T> *a;
    WriteToken(Access_typed<T> *_a):a(_a){ a->writeAccess(); }
    ~WriteToken(){ a->deAccess(); }
    T& operator()(){ return (*a)(); }
  };

  Access_typed(const char* name, Module *m=NULL, DataAccess *d=NULL):Access(name){ type=new Type_typed<T, void>();  module=currentlyCreating; var=d; currentlyCreating->accesses.append(this); }
  void readAccess(){CHECK(var,""); var->readAccess(module); }
  void writeAccess(){ CHECK(var,""); var->writeAccess(module); }
  void deAccess(){ CHECK(var,""); var->deAccess(module); }
  T& operator()(){ CHECK(var && var->data,""); return *((T*)var->data); }
  const T& get(){ return ReadToken(this)(); }
  T& set(){ return WriteToken(this)(); }
};


//===========================================================================
//
// macro to declare an Access
// (a bit of magic for automatically setting the access name...

#define ACCESS(type, name)\
struct __##name##__Access:Access_typed<type>{ \
  __##name##__Access():Access_typed<type>(#name){} \
} name; \


//===========================================================================
//
// macro to register a Module
//

#define REGISTER_MODULE(name) \
  Item_typed<Type> name##_ModuleRegistryItem(ARRAY<MT::String>(MT::String("Decl_Module"), MT::String(#name)), ItemL(), new Type_typed<name, void>(NULL,NULL), &registry());


//===========================================================================
//
// macro for default modules
//

#define BEGIN_MODULE1(name) \
struct name : Module { \
  struct s##name *s; \
  name(); \
  ~name(); \
  virtual void open(); \
  virtual void step(); \
  virtual void close();

#define BEGIN_MODULE(name) \
  struct name : Module { \
    name(): Module(#name) {} \
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
inline void operator<<(ostream&, const Module&){ NIY }

inline void operator>>(istream&, Access&){ NIY }
inline void operator<<(ostream& os, const Access& a){ os <<"Access '" <<a.name <<"' from '" <<a.module->name <<"' to '" <<(a.var?a.var->name:"??") <<'\''; }

#endif
