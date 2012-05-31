#include <MT/hypergraph.h>

struct GraphvizGtk{
  struct sGraphvizGtk *s;
  
  GraphvizGtk(ElementL& G, const char* title="MT::GraphvizGtk", void *container=NULL);
  ~GraphvizGtk();
  
  void update();
  void watch();
};
