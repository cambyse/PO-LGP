#include <Core/graph.h>
#include <Gui/graphview.h>

void TEST(Dot){
  const char* filename = (mlr::argc>=2 && mlr::argv[1][0]!='-')?mlr::argv[1]:NULL;
  if(!filename) filename="coffee_shop.fg";
  Graph G;
  G <<FILE(filename);
  cout <<"read KVG:\n----------\n" <<G <<"\n----------\n" <<endl;
  G.sortByDotOrder();
  GraphView gv(G);
  gv.writeFile("z.dot");
  gv.watch();
}

int MAIN(int argc, char** argv){
  mlr::initCmdLine(argc, argv);

  testDot();

  return 0;
}
