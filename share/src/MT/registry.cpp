#include "registry.h"

//===========================================================================
//
// global singleton TypeRegistrationSpace
//

TypeRegistration* reg_findType(const char* type){
  return NULL;
  ItemL items = registry().getItems(type);
  for_list_(Item, it, items){
    if(it->is_derived_from_TypeBase()){ //check if it is a type registration
      TypeRegistration *t= dynamic_cast<TypeRegistration*>(&(((Item_typed<TypeBase>*)it)->value));
      if(t) return t;
    }
  }
  return NULL;
}

Singleton<MapGraph> single_registry;

MapGraph& registry(){
  return single_registry.obj();
}
