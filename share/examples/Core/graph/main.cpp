#include <Core/graph.h>
#include <Core/registry.h>

//const char *filename="/home/mtoussai/git/3rdHand/documents/USTT/14-meeting3TUD/box.kvg";
const char *filename=NULL;

//===========================================================================

void TEST(Read){
  Graph G;

  G.checkConsistency();
  cout <<"\n** reading graph..." <<flush;
  G <<FILE(filename?filename:"example.kvg");
  G.checkConsistency();
  cout <<"\ndone" <<endl;
  cout <<"read kvg=\n--------------------\n" <<G <<"\n--------------------" <<endl;

//  Item *m = G.getItem("modify");
//  G.merge(m);
//  cout <<"'k modify' merged with 'k':" <<*G["k"] <<endl;

  G.checkConsistency();
  if(filename) return; //below only for "example.kvg"
  cout <<"\n** access to individual items:" <<endl;
  cout <<*G["k"] <<endl;
  cout <<G["k"]->kvg() <<endl;
  cout <<G["val"]->V<double>() <<endl;
  cout <<G["k"]->kvg()["z"]->V<MT::String>() <<endl;
  cout <<"DONE" <<endl;
}

//===========================================================================

void TEST(Init){
  Graph G = {"x", "b", {"a", 3.}, {"b", {"x"}, 5.}, {"c", MT::String("BLA")} };
  cout <<G <<endl;
  G.checkConsistency();
}

//===========================================================================

void TEST(Dot){
  Graph G;
  G <<FILE(filename?filename:"coffee_shop.fg");
  G.checkConsistency();
//  G.sortByDotOrder();
//  G.checkConsistency();
  G.writeDot(FILE("z.dot").getOs());
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
  Graph G;
  G.append({"hallo"}, {}, new Something(3), true);
  cout <<G <<endl;
}

//===========================================================================

int MAIN(int argc, char** argv){

//  cout <<"GLOBAL LATENT REGISTRY:\n" <<registry() <<endl;

  if(argc>=2) filename=argv[1];

//  testRead();
  testInit();
//  testDot();

//  if(!filename) testManual();

  return 0;
}
