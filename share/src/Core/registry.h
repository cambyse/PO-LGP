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


/// @file
/// @ingroup group_Core
/// @addtogroup group_Core
/// @{

#ifndef MLR_registry_h
#define MLR_registry_h

#include "array.h"
#include "graph.h"

//===========================================================================
//
// global registry of anything using a singleton Graph
//

extern Singleton<Graph> registry;
void initRegistry();


//===========================================================================
//
// to register a type (instead of general thing/item), use this:
//

template<class T, class Base>
struct Type_typed_readable:Type_typed<T,Base> {
  virtual Node* readIntoNewNode(Graph& container, std::istream& is) const { Node_typed<T> *n = container.newNode<T>(T()); is >>n->value; return n; }
};

typedef mlr::Array<std::shared_ptr<Type> > TypeInfoL;


//===========================================================================
//
// retrieving types
//

//-- query existing types
inline Node *reg_findType(const char* key) {
  NodeL types = registry().getNodesOfType<std::shared_ptr<Type> >();
  for(Node *ti: types) {
    if(mlr::String(ti->get<std::shared_ptr<Type> >()->typeId().name())==key) return ti;
    if(ti->matches(key)) return ti;
  }
  return NULL;
}

template<class T>
Node *reg_findType() {
  NodeL types = registry().getNodesOfType<std::shared_ptr<Type> >();
  for(Node *ti: types) {
    if(ti->get<std::shared_ptr<Type> >()->typeId()==typeid(T)) return ti;
  }
  return NULL;
}


//===========================================================================
//
// read a value from a stream by looking up available registered types
//

inline Node* readTypeIntoNode(Graph& container, const char* key, std::istream& is) {
  Node *ti = reg_findType(key);
  if(ti) return ti->get<std::shared_ptr<Type> >()->readIntoNewNode(container, is);
  return NULL;
}


//===========================================================================
//
// typed version
//



//===========================================================================
//
// macros for declaring types (in *.cpp files)
//

#define KO ,

#define REGISTER_TYPE(T) \
  RUN_ON_INIT_BEGIN(Decl_Type##_##T) \
  registry().newNode<std::shared_ptr<Type> >({mlr::String("Decl_Type"), mlr::String(#T)}, NodeL(), std::make_shared<Type_typed_readable<T KO void> >()); \
  RUN_ON_INIT_END(Decl_Type##_##T)

#define REGISTER_TYPE_Key(Key, T) \
  RUN_ON_INIT_BEGIN(Decl_Type##_##Key) \
  registry().newNode<std::shared_ptr<Type> >({mlr::String("Decl_Type"), mlr::String(#Key)}, NodeL(), std::make_shared<Type_typed_readable<T KO void> >()); \
  RUN_ON_INIT_END(Decl_Type##_##Key)

#define REGISTER_TYPE_DERIVED(T, Base) \
  RUN_ON_INIT_BEGIN(Decl_Type##_##T) \
  registry().newNode<std::shared_ptr<Type> >({mlr::String("Decl_Type"), mlr::String(#T)}, NodeL(), std::make_shared<Type_typed_readable<T KO Base> >()); \
  RUN_ON_INIT_END(Decl_Type##_##T)


#endif

/// @} //end group
