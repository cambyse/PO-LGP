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
// entry of the type registry
//

struct TypeRegistration;
typedef MT::Array<TypeRegistration*> TypeRegistrationL;
TypeRegistrationL& typeRegistry();

struct TypeRegistration{
  StringA keys;
  TypeRegistrationL parents;
  virtual const std::type_info& typeinfo() const = 0;
  virtual struct Item* read(istream&) const = 0;
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


//===========================================================================
//
// user interface
//

//-- query existing types
void reg_report();
TypeRegistration* reg_find(const char* type);
template <class T> TypeRegistrationL reg_findDerived();
inline Item* readTypeIntoItem(const char* type, std::istream& is){
  TypeRegistration *t = reg_find(type);
  if(t) return t->read(is);
  return NULL;
}

//-- use these macros to register types in cpp files

#define REGISTER_TYPE_Key(Key, Type)\
  TypeRegistration_typed<Type,void> Key##_registrationDummy(#Type, #Key, NULL, &typeRegistry());

#define REGISTER_TYPE(Type)\
  TypeRegistration_typed<Type,void> Type##_registrationDummy(#Type, NULL, NULL, &typeRegistry());

#define REGISTER_DERIVED_TYPE(Type, Base)\
  TypeRegistration_typed<Type,Base> Type##_registrationDummy(#Type, NULL, #Base, &typeRegistry());

#define KO ,
#define REGISTER_TYPE_(Type) \
  REGISTER_ITEM2(TypeRegistration_typed<Type KO void>, type, Type, TypeRegistration_typed<Type KO void>(#Type,NULL,NULL,NULL));

#define REGISTER_TYPE_DERIVED(Type, Base) \
  REGISTER_ITEM2(TypeRegistration_typed<Type KO Base>, type, Type, TypeRegistration_typed<Type KO Base>(#Type,NULL,#Base,NULL));



// stuff that needs to be included by the header to enable macros and templates
#include "registry_t.cxx"

#endif
