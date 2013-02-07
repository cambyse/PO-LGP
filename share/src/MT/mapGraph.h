#ifndef MT_mapGraph_h
#define MT_mapGraph_h

#include <MT/array.h>
#include <MT/util.h>

struct Item;
struct MapGraph;
typedef MT::Array<Item*> ItemL;
typedef MT::Array<MT::String> StringA;
struct TypeBase{ virtual ~TypeBase(){}; }; //if types derive from TypeBase, more tricks are possible
inline std::istream& operator>>(std::istream&, TypeBase&){ NIY; }
inline std::ostream& operator<<(std::ostream&, const TypeBase&){ NIY; }

struct Item {
  StringA keys;
  ItemL parents;
  ItemL parentOf;
  uint index;
  virtual ~Item() {};
  template<class T> T *value();    //access the value
  template<class T> const T *value() const;
  void write(std::ostream &os) const;
  virtual void writeValue(std::ostream &os) const {NIY}
  virtual const std::type_info& valueType() const {NIY}
  virtual bool is_derived_from_TypeBase() const {NIY}
  virtual Item *newClone() const {NIY}
};
stdOutPipe(Item);


struct MapGraph:ItemL{
  struct sMapGraph *s;

  MapGraph();
  ~MapGraph();

  MapGraph& operator=(const MapGraph&);

  //-- get items
  Item* getItem(const char *key);
  Item* getItem(const char *key1, const char *key2);
  Item* operator[](const char *key){ return getItem(key); }

  //-- get values directly
  template<class T> T* getValue(const char *key);
  template<class T> bool getValue(T& x, const char *key){ T* y=getValue<T>(key); if(y){ x=*y; return true; } return false; }

  //-- get lists of items
  MapGraph getItems(const char*);
  template<class T> MapGraph getTypedItems(const char*);

  //-- get lists of values
  template<class T> MT::Array<T*> getDerivedValues();

  Item *append(Item* it){ ItemL::append(it); return it; }
  template<class T> Item *append(const StringA& keys, const ItemL& parents, T *x);
  template<class T> Item *append(const StringA& keys, T *x){ return append(keys, ItemL(), x); }
  template<class T> Item *append(const char *key, T *x){ return append(ARRAY<MT::String>(MT::String(key)), ItemL(), x); }

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
