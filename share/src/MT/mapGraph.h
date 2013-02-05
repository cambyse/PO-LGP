#ifndef MT_mapGraph_h
#define MT_mapGraph_h

#include <MT/array.h>
#include <MT/util.h>

struct Item;
struct MapGraph;
typedef MT::Array<Item*> ItemL;
typedef MT::Array<MT::String> StringA;
struct TypeBase{ virtual ~TypeBase(){}; }; //if types derive from TypeBase, more tricks are possible

struct Item {
  StringA keys;
  ItemL parents;
  ItemL parentOf;
  uint index;
  virtual ~Item() {};
  template<class T> T& value();    //access the value
  template<class T> const T& value() const;
  void write(std::ostream &os) const;
  virtual void writeValue(std::ostream &os) const = 0;
  virtual const std::type_info& valueType() const = 0;
  virtual bool is_derived_from_TypeBase() const = 0;
  virtual Item *newClone() const { NIY }
  virtual void *newInstance() const { NIY }
};
stdOutPipe(Item);


struct MapGraph:ItemL{
  struct sMapGraph *s;

  MapGraph();
  ~MapGraph();

  MapGraph& operator=(const MapGraph&);

  Item* getItem(const char *key);
  Item* getItem(const char *key1, const char *key2);
  ItemL getItems(const char*);
  Item* operator[](const char *key){ return getItem(key); }

  template<class T> Item* getTypedItem(const char*);

  template<class T> MT::Array<T*> getDerivedItems();

  template<class T> T* get(const char *key);
  template<class T> bool get(T& x, const char *key){ T* y=get<T>(key); if(y){ x=*y; return true; } return false; }

  Item *append(Item* it){ ItemL::append(it); return it; }
  template<class T> Item *append(const StringA& keys, const ItemL& parents, const T& x);
  template<class T> Item *append(const StringA& keys, const T& x){ return append(keys, ItemL(), x); }
  template<class T> Item *append(const char *key,T& x); //{ append(STRINGS(key), ItemL(), x); }


  Item *add(const uintA& tuple);
  ItemL& getParents(uint i);

  void sortByDotOrder();

  void read(std::istream& is);
  void write(std::ostream& os=std::cout, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(std::ostream& os=std::cout);
};
stdPipes(MapGraph);

#include "mapGraph_t.cxx"

#endif
