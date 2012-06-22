#include <MT/hypergraph.h>

struct GraphView{
  struct sGraphView *s;
  
  GraphView(ElementL& G, const char* title="MT::GraphvizGtk", void *container=NULL);
  ~GraphView();
  
  void update();
  void watch();
};
