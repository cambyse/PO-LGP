#ifndef MT_typeRegistry_h
#define MT_typeRegistry_h

#include <MT/util.h>
#include <MT/array.h>


//===========================================================================
//
// entry of the type registry
//

struct TypeRegistration;
typedef MT::Array<TypeRegistration*> TypeRegistrationL;

struct TypeRegistration{
  const char *key, *userType, *sysType;
  TypeRegistrationL parents;
  virtual const std::type_info& typeinfo() const = 0;
  virtual const std::type_info& baseinfo() const = 0;
  void write(ostream& os) const{ cout <<userType <<endl; }
  virtual struct Item* read(istream&) const = 0;
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
  TypeRegistration_typed<Type,void> Key##_registrationDummy(#Type,#Key,NULL);

#define REGISTER_TYPE(Type)\
  TypeRegistration_typed<Type,void> Type##_registrationDummy(#Type,NULL,NULL);

#define REGISTER_DERIVED_TYPE(Type,Base)\
  TypeRegistration_typed<Type,Base> Type##_registrationDummy(#Type,NULL,#Base);



// stuff that needs to be included by the header to enable macros and templates
#include "registry_t.cxx"

#endif
