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

#include "graph.h"

//===========================================================================
//
//  typed Node
//

template<class T>
struct Node_typed : Node {
  T value;

  Node_typed():value(NULL) { HALT("shouldn't be called, right? You always want to append to a container"); }

  /// directly store pointer to value
  Node_typed(Graph& container, const T& value)
    : Node(typeid(T), container), value(value) {
    if(isGraph()) graph().isNodeOfParentGraph = this; //this is the only place where isNodeOfParentGraph is set
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  /// directly store pointer to value
  Node_typed(Graph& container, const StringA& keys, const NodeL& parents, const T& value)
    : Node(typeid(T), container, keys, parents), value(value) {
    if(isGraph()) graph().isNodeOfParentGraph = this; //this is the only place where isNodeOfParentGraph is set
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_new(this);
  }

  virtual ~Node_typed(){
    if(&container && container.callbacks.N) for(GraphEditCallback *cb:container.callbacks) cb->cb_delete(this);
  }

  virtual void *getValueDirectly() const {
    return (void*)&value;
  }

  virtual void copyValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
    if(typeid(T)==typeid(Graph*)){
      graph().copy(it->graph());
    }else{
      value = itt->value;
    }
  }

  virtual bool hasEqualValue(Node *it) {
    Node_typed<T> *itt = dynamic_cast<Node_typed<T>*>(it);
    CHECK(itt,"can't assign to wrong type");
#ifdef MLR_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdynamic-class-memaccess"
#endif
    if(typeid(T)==typeid(Graph*)){
      Graph *g1=it->V<Graph*>();
      Graph *g2=V<Graph*>();
      return *g1==*g2;
    }
    return itt->value == value;
#ifdef MLR_CLANG
#  pragma clang diagnostic pop
#endif
  }

  virtual void writeValue(std::ostream &os) const {
    if(typeid(T)==typeid(NodeL)) listWrite(*getValue<NodeL>(), os, " ");
    else os <<value;
  }
  
  virtual const std::type_info& getValueType() const {
    return typeid(T);
  }
  
  virtual Node* newClone(Graph& container) const {
    if(getValueType()==typeid(Graph*)){
      Graph& g = newSubGraph(container, keys, parents)->value;
      g.copy(*V<Graph*>());
      return g.isNodeOfParentGraph;
    }
    return new Node_typed<T>(container, keys, parents, value);
  }
};

template<class T> T* Node::getValue() {
  Node_typed<T>* typed = dynamic_cast<Node_typed<T>*>(this);
  if(!typed) return NULL;
  return &typed->value;
}

template<class T> const T* Node::getValue() const {
  const Node_typed<T>* typed = dynamic_cast<const Node_typed<T>*>(this);
  if(!typed) return NULL;
  return &typed->value;
}

template<class T> Nod::Nod(const char* key, const T& x){
  n = new Node_typed<T>(G, x);
  n->keys.append(STRING(key));
}

template<class T> Nod::Nod(const char* key, const StringA& parents, const T& x)
  : parents(parents){
  n = new Node_typed<T>(G, x);
  n->keys.append(STRING(key));
}

template<class T> Node* Graph::getNodeOfType(const char *key) const {
  NodeL nodes = getNodes(key);
  for(Node* n:nodes) if(n->getValueType()==typeid(T)) return n;
  return NULL;
}

template<class T> T& Graph::get(const char *key) const {
  Node *n = getNode(key);
  if(!n) HALT("node '"<< key<< "' does not exist (to retrieve type '"<<typeid(T).name() <<"')");
  T* val=n->getValue<T>();
  if(!val) HALT("node " <<*n <<" does not have type '"<<typeid(T).name() <<"'");
  return *val;
}

template<class T> T& Graph::get(const StringA& keys) const {
  Node *n = getNode(keys);
  if(!n) HALT("node '"<< keys<< "' does not exist (to retrieve type '"<<typeid(T).name() <<"')");
  T* val=n->getValue<T>();
  if(!val) HALT("node " <<*n <<" does not have type '"<<typeid(T).name() <<"'");
  return *val;
}

template<class T> const T& Graph::get(const char *key, const T& defaultValue) const{
  Node *n = getNode(key);
  if(!n) return defaultValue;
  T* val=n->getValue<T>();
  if(!val) return defaultValue;
  return *val;
}

template<class T> mlr::Array<T*> Graph::getValuesOfType(const char* key) {
  mlr::Array<T*> ret;
  for(Node *n: (*this)) if(n->getValueType()==typeid(T)) {
    if(!key) ret.append(n->getValue<T>());
    else for(uint i=0; i<n->keys.N; i++) if(n->keys(i)==key) {
      ret.append(n->getValue<T>());
      break;
    }
  }
  return ret;
}

template<class T> Node *Graph::append(const StringA& keys, const NodeL& parents, const T& x){
  return new Node_typed<T>(*this, keys, parents, x);
}



