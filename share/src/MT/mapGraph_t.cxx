//===========================================================================
//
//  typed Item
//

template<class T>
struct Item_typed:Item {
  T value;

  Item_typed(const T& _value):value(_value){}

  Item_typed(const StringA& _keys, const ItemL& _parents, const T& _value, MapGraph *container=NULL):value(_value){
    keys=_keys;
    parents=_parents;
    if(container) container->append(this);
  }

  virtual void writeValue(std::ostream &os) const {
    if(typeid(T)==typeid(ItemL)) listWrite(*(ItemL*)(&value), os, " ");
    else os <<value;
  }

  virtual const std::type_info& valueType() const {
    return typeid(T);
  }

  virtual Item* newClone() const { return new Item_typed<T>(keys, parents, value); }
};

template<class T> T& Item::value(){
  Item_typed<T>* typed = dynamic_cast<Item_typed<T>*>(this);
  if(!typed){
    MT_MSG("can't cast type '" <<valueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning NULL");
    return *((T*)NULL);
  }
  return typed->value;
}

template<class T> const T& Item::value() const{
  const Item_typed<T>* typed = dynamic_cast<const Item_typed<T>*>(this);
  if(!typed){
    MT_MSG("can't cast type '" <<valueType().name() <<"' to type '" <<typeid(T).name() <<"' -- returning reference-to-NULL");
    return *((T*)NULL);
  }
  return typed->value;
}


template<class T> T* MapGraph::get(const char *key){
  Item *it = getItem(key);
  if(!it) return NULL;
  return &it->value<T>();
}

template<class T> Item *MapGraph::append(const StringA& keys, const ItemL& parents, const T& x){
  return append(new Item_typed<T>(keys, parents, x, NULL));
}
