#include <MT/keyValueGraph.h>
#include <MT/graphview.h>
#include <MT/ors.h>
#include <MT/registry.h>

REGISTER_TYPE_Key(T, ors::Transformation)

void testRead(const char *filename){
  KeyValueGraph G;

  cout <<"reading graph..." <<flush;
  MT::load(G, filename);
  cout <<"done" <<endl;

  GraphView gv(G);
  gv.watch();

  cout <<G <<endl;
  cout <<"access to individual items:" <<endl;
  cout <<*G["g"] <<endl;
  cout <<*G["g"]->value<KeyValueGraph>() <<endl;
  cout <<*G.getValue<KeyValueGraph>("g")->getValue<MT::String>("y") <<endl;
}

void testDot(const char *filename="coffee_shop.fg"){
  KeyValueGraph G;

  cout <<"reading graph..." <<endl;
  MT::load(G,filename);
  cout <<G <<endl;
  G.sortByDotOrder();
  G.writeDot();

  GraphView gv(G);
  gv.watch();
}


int main(int argc, char** argv){
  cout <<registry() <<endl;

  //testRead(argc<2?"test.kvg":argv[1]);

  testDot();

  return 0;
}
