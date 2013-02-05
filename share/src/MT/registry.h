#ifndef MT_registry_h
#define MT_registry_h

#include "util.h"
#include "array.h"
#include "mapGraph.h"


//===========================================================================
//
// global registry of anything
//

MapGraph& registry();

#define REGISTER_ITEM(Type, key, value) \
  Item_typed<Type > key##_RegistryEntry(ARRAY<MT::String>(MT::String(#key)), ItemL(), value, &registry());

#define REGISTER_ITEM2(Type, key1, key2, value) \
  Item_typed<Type > key1##_##key2##_RegistryEntry(ARRAY<MT::String>(MT::String(#key1),MT::String(#key2)), ItemL(), value, &registry());


//===========================================================================
//
// a generic singleton registry (list) of things (usually Base_typed deriving from a common Base)
//

template<class T>
struct Singleton{
  struct SingletonFields{ //class cannot have own members: everything in the singleton which is created on first demand
    T obj;
    RWLock lock;
  };

  static SingletonFields *singleton;

  SingletonFields& getSingleton() const{
    static bool currentlyCreating=false;
    if(currentlyCreating) return *((SingletonFields*) NULL);
    if(!singleton) {
      static Mutex m;
      m.lock();
      if(!singleton) {
        currentlyCreating=true;
        singleton = new SingletonFields();
        currentlyCreating=false;
      }
      m.unlock();
    }
    return *singleton;
  }

  T& obj(){ return getSingleton().obj; }
};


//===========================================================================
//
// define a type registry
//

struct TypeRegistration:TypeBase{
  StringA keys;
  MT::Array<TypeRegistration*> parents;
  virtual const std::type_info& typeinfo() const = 0;
  virtual struct Item* read(istream&) const = 0;
  virtual void* newInstance() const { NIY }
  void write(std::ostream& os) const{
    os <<"Type '" <<keys <<"' [" <<typeinfo().name() <<"] ";
    if(parents.N){
      cout <<"parents=[";
      for_list_(TypeRegistration, p, parents) cout <<' ' <<p->typeinfo().name();
      cout <<" ]";
    }
  }
};
stdOutPipe(TypeRegistration);

typedef MT::Array<TypeRegistration*> TypeRegistrationL;

// user interface

//-- query existing types
TypeRegistration* reg_findType(const char* type);
template <class T> TypeRegistrationL reg_findDerived();
inline Item* readTypeIntoItem(const char* type, std::istream& is){
  TypeRegistration *it = reg_findType(type);
  if(it) return it->read(is);
  return NULL;
}

//-- use these macros to register types in cpp files

#define KO ,
#define REGISTER_TYPE(Type) \
  REGISTER_ITEM2(TypeRegistration_typed<Type KO void>, type, Type, TypeRegistration_typed<Type KO void>(#Type,NULL,NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Type, NULL)

#define REGISTER_TYPE_Key(Key, Type) \
  REGISTER_ITEM2(TypeRegistration_typed<Type KO void>, type, Key, TypeRegistration_typed<Type KO void>(#Type,NULL,NULL,NULL));
//  REGISTER_ITEM2(Type*, type, Key, NULL);

#define REGISTER_TYPE_DERIVED(Type, Base) \
  REGISTER_ITEM2(TypeRegistration_typed<Type KO Base>, type, Type, TypeRegistration_typed<Type KO Base>(#Type,NULL,#Base,NULL));

#include "registry_t.cxx"


//===========================================================================
//
// registrator -- helper to call registration code for classes & members
//


// stuff that needs to be included by the header to enable macros and templates

#endif
