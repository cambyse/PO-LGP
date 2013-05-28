#include <Core/keyValueGraph.h>
//#include <MT/ors.h>
#include <Core/registry.h>
#include <Gui/graphview.h>

//REGISTER_TYPE_Key(T, ors::Transformation)

void testRead(const char *filename=NULL){
  KeyValueGraph G;

  cout <<"\n** reading graph..." <<flush;
  MT::load(G, filename?filename:"example.kvg");
  cout <<"done" <<endl;
  cout <<G <<endl;

  if(filename) return; //below only for "example.kvg"
  cout <<"\n** access to individual items:" <<endl;
  cout <<*G["k"] <<endl;
  cout <<*G["k"]->value<KeyValueGraph>() <<endl;
  cout <<*G.getValue<KeyValueGraph>("k")->getValue<MT::String>("z") <<endl;
}

void testDot(const char *filename=NULL){
  KeyValueGraph G;
  MT::load(G, filename?filename:"coffee_shop.fg");
  G.sortByDotOrder();
  G.writeDot();
  GraphView gv(G);
  gv.watch();
}

int main(int argc, char** argv){
  cout <<"GLOBAL LATENT REGISTRY:\n" <<registry() <<endl;

  testRead(argc<2?NULL:argv[1]);

  testDot(argc<2?NULL:argv[1]);

  return 0;
}
