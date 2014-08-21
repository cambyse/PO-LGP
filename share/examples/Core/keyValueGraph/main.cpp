#include <Core/keyValueGraph.h>
#include <Core/registry.h>

const char *filename=NULL;

//===========================================================================

void TEST(Read){
  KeyValueGraph G;

  cout <<"\n** reading graph..." <<flush;
  G <<FILE(filename?filename:"example.kvg");
  cout <<"\ndone" <<endl;
  cout <<"read kvg=\n--------------------\n" <<G <<"\n--------------------" <<endl;

//  Item *m = G.getItem("modify");
//  G.merge(m);
//  cout <<"'k modify' merged with 'k':" <<*G["k"] <<endl;

  if(filename) return; //below only for "example.kvg"
  cout <<"\n** access to individual items:" <<endl;
  cout <<*G["k"] <<endl;
  cout <<*G["k"]->getValue<KeyValueGraph>() <<endl;
  cout <<*G["val"]->getValue<double>() <<endl;
  cout <<*G.getValue<KeyValueGraph>("k")->getValue<MT::String>("z") <<endl;
}

//===========================================================================

void TEST(Dot){
  KeyValueGraph G;
  G <<FILE(filename?filename:"coffee_shop.fg");
  G.sortByDotOrder();
  G.writeDot();
}

//===========================================================================

struct Something{
  Something(double y=0.){ x=y; }
  double x;
};
void operator<<(ostream& os, Something& s){ os <<s.x; }
//the following 2 lines are optional: they enable naming the type and typed reading from file
void operator>>(istream& is, Something& s){ is >>s.x; }
REGISTER_TYPE(Something)

void TEST(Manual){
  KeyValueGraph G;
  G.append(STRINGS("hallo"), ItemL(), new Something(3));
  cout <<G <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){
  testManual();
  return 0;

  cout <<"GLOBAL LATENT REGISTRY:\n" <<registry() <<endl;

  if(argc>=2) filename=argv[1];

  testRead();
  testDot();

  return 0;
}
