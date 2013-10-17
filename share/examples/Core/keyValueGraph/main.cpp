#include <Core/keyValueGraph.h>
#include <Core/registry.h>

const char *filename=NULL;

void TEST(Read){
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

void TEST(Dot){
  KeyValueGraph G;
  MT::load(G, filename?filename:"coffee_shop.fg");
  G.sortByDotOrder();
  G.writeDot();
}

int MAIN(int argc, char** argv){
  cout <<"GLOBAL LATENT REGISTRY:\n" <<registry() <<endl;

  if(argc>=2) filename=argv[1];

  testRead();
  testDot();

  return 0;
}
