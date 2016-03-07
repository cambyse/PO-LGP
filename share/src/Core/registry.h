/*  ---------------------------------------------------------------------
    Copyright 2014 Marc Toussaint
    email: marc.toussaint@informatik.uni-stuttgart.de
    
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
void initRegistry(int argc, char *argv[]);


//===========================================================================
//
// to register a type (instead of general thing/item), use this:
//

struct Type {
  virtual ~Type(){}
  virtual const std::type_info& typeId() const {NIY}
  virtual struct Node* readIntoNewNode(Graph& container, istream&) const {NIY}
  virtual void* newInstance() const {NIY}
//  virtual Type* clone() const {NIY}
  void write(std::ostream& os) const {  os <<"Type '" <<typeId().name() <<"' ";  }
  void read(std::istream& is) const {NIY}
};
stdPipes(Type)

template<class T, class Base>
struct Type_typed : Type {
  virtual const std::type_info& typeId() const { return typeid(T); }
  virtual void* newInstance() const { return new T(); }
//  virtual Type* clone() const { Type *t = new Type_typed<T, void>(); t->parents=parents; return t; }
};

template<class T, class Base>
struct Type_typed_readable:Type_typed<T,Base> {
  virtual Node* readIntoNewNode(Graph& container, istream& is) const { Node_typed<T> *n = new Node_typed<T>(container, T()); is >>n->value; return n; }
//  virtual Type* clone() const { Type *t = new Type_typed_readable<T, void>(); t->parents=Type::parents; return t; }
};

inline bool operator!=(Type& t1, Type& t2){ return t1.typeId() != t2.typeId(); }
inline bool operator==(Type& t1, Type& t2){ return t1.typeId() == t2.typeId(); }

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
  new Node_typed<std::shared_ptr<Type> >(registry(), {mlr::String("Decl_Type"), mlr::String(#T)}, NodeL(), std::make_shared<Type_typed_readable<T KO void> >()); \
  RUN_ON_INIT_END(Decl_Type##_##T)

#define REGISTER_TYPE_Key(Key, T) \
  RUN_ON_INIT_BEGIN(Decl_Type##_##Key) \
  new Node_typed<std::shared_ptr<Type> >(registry(), {mlr::String("Decl_Type"), mlr::String(#Key)}, NodeL(), std::make_shared<Type_typed_readable<T KO void> >()); \
  RUN_ON_INIT_END(Decl_Type##_##Key)

/*

#define REGISTER_TYPE_DERIVED(T, Base) \
  RUN_ON_INIT_BEGIN(Decl_Type##_##T) \
  new Node_typed<Type*>(registry(), {mlr::String("Decl_Type"), mlr::String(#T)}, NodeL(), new Type_typed_readable<T KO Base>(#Base,NULL)); \
  RUN_ON_INIT_END(Decl_Type##_##T)

*/

#endif

/// @} //end group
