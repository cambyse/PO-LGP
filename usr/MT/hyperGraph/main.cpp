#include <MT/hypergraph.h>
#include <MT/graphview.h>

void testRead(const char *filename="../rules/coffee_shop.fg"){
  HyperGraph H;

  cout <<"reading graph..." <<endl;
  MT::load(H,filename);
  //cout <<H <<endl;
  sortByDotOrder(H);
  cout <<H <<endl;
  writeDot(H);

  GraphView gv(H);
  //MT::wait(2.);
  gv.watch();
}

void testBasic(){
  HyperGraph G;
  G.add(TUP());
  G.write(cout);
  cout <<"--------------------------" <<endl;
  G.add(TUP());
  G.write(cout);
  cout <<"--------------------------" <<endl;
  G.add(TUP(1,1));
  G.write(cout);
  cout <<"--------------------------" <<endl;
  G.add(TUP(1,1));
  G.write(cout);
  cout <<"--------------------------" <<endl;
  G.add(TUP(0,1));
  G.write(cout);
  cout <<"--------------------------" <<endl;

  listWrite(G.getOutEdges(0),cout);
  cout <<"--------------------------" <<endl;
  listWrite(G.getOutEdges(1),cout);
  cout <<"--------------------------" <<endl;
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  
  int mode = MT::getParameter<int>("mode",1);
  switch(mode){
    case 0:  testBasic();  break;
    case 1:  if(argn>1) testRead(argv[1]); else testRead();  break;
  }
  
  return 0;
}
