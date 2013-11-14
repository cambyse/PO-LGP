#include <Core/keyValueGraph.h>
#include <Gui/graphview.h>

void TEST(Dot){
  const char* filename = MT::argc<2?NULL:MT::argv[1];
  KeyValueGraph G;
  MT::load(G, filename?filename:"coffee_shop.fg");
  G.sortByDotOrder();
  G.writeDot();
  GraphView gv(G);
  gv.watch();
}

int MAIN(int argc, char** argv){
  MT::initCmdLine(argc, argv);
  testDot();

  return 0;
}
