#include <Core/keyValueGraph.h>
#include <Gui/graphview.h>

void testDot(const char *filename=NULL){
  KeyValueGraph G;
  MT::load(G, filename?filename:"coffee_shop.fg");
  G.sortByDotOrder();
  G.writeDot();
  GraphView gv(G);
  gv.watch();
}

int main(int argc, char** argv){
  testDot(argc<2?NULL:argv[1]);

  return 0;
}
