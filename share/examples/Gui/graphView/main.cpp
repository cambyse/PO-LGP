#include <Core/keyValueGraph.h>
#include <Gui/graphview.h>

void TEST(Dot){
  const char* filename = (MT::argc>=2 && MT::argv[1][0]!='-')?MT::argv[1]:NULL;
  KeyValueGraph G;
  G <<FILE(filename?filename:"coffee_shop.fg");
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
