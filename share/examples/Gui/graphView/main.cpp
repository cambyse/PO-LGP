#include <Core/keyValueGraph.h>
#include <Gui/graphview.h>

void TEST(Dot){
  const char* filename = (MT::argc>=2 && MT::argv[1][0]!='-')?MT::argv[1]:NULL;
  if(!filename) filename="coffee_shop.fg";
  KeyValueGraph G;
  G <<FILE(filename);
  cout <<"read KVG:\n----------\n" <<G <<"\n----------\n" <<endl;
  G.sortByDotOrder();
  GraphView gv(G);
  gv.writeFile("z.dot");
  gv.watch();
}

int MAIN(int argc, char** argv){
  MT::initCmdLine(argc, argv);

  testDot();

  return 0;
}
