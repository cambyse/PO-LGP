#ifndef MT_mapGraph_h
#define MT_mapGraph_h

#include <MT/array.h>
#include <MT/util.h>

struct Item;
struct MapGraph;
typedef MT::Array<Item*> ItemL;
typedef MT::Array<MT::String> StringA;

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
  virtual Item *newClone() const = 0;
};
stdOutPipe(Item);


struct MapGraph:ItemL{
  struct sMapGraph *s;

  MapGraph();
  ~MapGraph();

  MapGraph& operator=(const MapGraph&);

  Item* getItem(const char*);
  Item& operator[](const char *key){ return *getItem(key); }
  ItemL getItems(const char*);
  template<class T> T* get(const char *key);
  template<class T> bool get(T& x, const char *key){ T* y=get<T>(key); if(y){ x=*y; return true; } return false; }

  template<class T> void append(const char *key,T& x);
  void append(Item* it){ ItemL::append(it); }

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
