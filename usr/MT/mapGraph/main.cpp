#include <MT/mapGraph.h>
#include <MT/ors.h>
#include <MT/typeRegistry.h>

REGISTER_TYPE_Key(T, ors::Transformation)

void testRead(const char *filename="../rules/coffee_shop.fg"){
  MapGraph G;

  cout <<"reading graph..." <<endl;
  MT::load(G, filename);
  //sortByDotOrder(H);
  //cout <<H <<endl;
  //writeDot(H);

  //GraphView gv(H);
  //MT::wait(2.);
  //gv.watch();
  cout <<G <<endl;
  cout <<G["edge"] <<endl;
  cout <<G["edge"].value<MapGraph>() <<endl;
  cout <<G.value<MapGraph>("edge").value<arr>("v") <<endl;
  //cout <<I;
}

int main(int argc, char** argv){
  reportRegisteredTypes();
  testRead("test.g");
  return 0;
}
