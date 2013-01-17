#include <MT/mapGraph.h>
#include <MT/ors.h>
#include <MT/registry.h>

REGISTER_TYPE_Key(T, ors::Transformation)

void testRead(const char *filename="../rules/coffee_shop.fg"){
  MapGraph G;

  cout <<"reading graph..." <<flush;
  MT::load(G, filename);
  cout <<"done" <<endl;
  //sortByDotOrder(H);
  //cout <<H <<endl;
  //writeDot(H);

  //GraphView gv(H);
  //MT::wait(2.);
  //gv.watch();
  cout <<G <<endl;
  cout <<G["edge"] <<endl;
  cout <<G["edge"].value<MapGraph>() <<endl;
  cout <<*G.get<MapGraph>("edge")->get<arr>("v") <<endl;
  //cout <<I;
}

int main(int argc, char** argv){
  reg_report();

  testRead(argc<2?"test.g":argv[1]);
  return 0;
}
