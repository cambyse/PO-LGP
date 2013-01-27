#include "mapGraph.h"

//===========================================================================
//
// stuff that should be hidden away - not for the user
//

template<class Type, class Base>
struct TypeRegistration_typed:TypeRegistration{
  TypeRegistration_typed(const char *_userType, const char *_key, const char *userBase, TypeRegistrationL *container ){
    keys.append(MT::String(_userType));
    if(_key) keys.append(MT::String(_key));
    if(userBase){
      TypeRegistration* t = reg_find(userBase);
      if(t) parents.append(t);
    }
    if(container){
      container->append(this);
    }
  }
  virtual const std::type_info& typeinfo() const { return typeid(Type); }
  virtual Item* read(istream& is) const{ Type x; is >>x; return new Item_typed<Type>(x); }
};

template <class T>
TypeRegistrationL reg_findDerived(){
  TypeRegistrationL results;
  TypeRegistration *t, *p;
  uint i,j;
  for_list(i, t, typeRegistry()){
    for_list(j, p, t->parents){
      if(p->typeinfo()==typeid(T)){ results.append(t); }
    }
  }
  return results;
}
