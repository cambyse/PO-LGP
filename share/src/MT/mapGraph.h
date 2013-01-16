#ifndef MT_mapGraph_h
#define MT_mapGraph_h

#include <MT/array.h>
#include <MT/util.h>

struct Item;
struct MapGraph;
typedef MT::Array<Item*> ItemL;

struct Item {
  StringL keys;
  ItemL parents;
  ItemL parentOf;
  virtual ~Item() {};
  template<class T> T& value();    //access the value
  template<class T> const T& value() const;
  void write(std::ostream &os) const;
  virtual void writeValue(std::ostream &os) const = 0;
  virtual const std::type_info& valueType() const = 0;
};
stdOutPipe(Item);


struct MapGraph:ItemL{
  struct sMapGraph *s;

  MapGraph();
  ~MapGraph();

  Item* item(const char*);
  Item& operator[](const char *key){ return *item(key); }
  template<class T> T& value(const char*);

  template<class T> void append(const char *key,T& value);

  inline void sortByDotOrder(ItemL& items);

  void read(std::istream& is);
  void write(std::ostream& os, const char *ELEMSEP="\n", const char *delim=NULL) const;
  void writeDot(ItemL& items);
};
stdPipes(MapGraph);

#include "mapGraph_t.cxx"

#endif
