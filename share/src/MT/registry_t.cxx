#include "mapGraph.h"

//===========================================================================
//
// stuff that should be hidden away - not for the user
//

void registerType(TypeRegistration*);
TypeRegistrationL& typeRegistrations();

template<class Type, class Base>
struct TypeRegistration_typed:TypeRegistration{
  TypeRegistration_typed(const char *_userType, const char *_key, const char *userBase ){
    key = _key;
    userType= _userType;
    sysType = typeid(Type).name();
    if(userBase){
      TypeRegistration* t = reg_find(userBase);
      if(t) parents.append(t);
    }
    registerType(this);
  }
  virtual const std::type_info& typeinfo() const { return typeid(Type); }
  virtual const std::type_info& baseinfo() const { return typeid(Base); }
  virtual Item* read(istream& is) const{ Type x; is >>x; return new Item_typed<Type>(x); }
};

template <class T>
TypeRegistrationL reg_findDerived(){
  TypeRegistrationL results;
  TypeRegistration *t;
  uint i;
  for_list(i, t, typeRegistrations()){
    if(t->baseinfo()==typeid(T)){ results.append(t); }
  }
  return results;
}
