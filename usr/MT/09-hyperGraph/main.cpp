#include <MT/hypergraph.h>

void testRead(const char *filename="graph"){
  HyperGraph H;

  cout <<"reading graph..." <<endl;
  MT::load(H,filename);
  cout <<H <<endl;
  H.sortByDotOrder();
  cout <<H <<endl;
  writeDot(H.T);
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

void testDot(const char *filename="graph"){
}

int main(int argn,char** argv){
  MT::initCmdLine(argn,argv);
  
  int mode = MT::getParameter<int>("mode",1);
  switch(mode){
    case 0:  testBasic();  break;
    case 1:  if(argn>1) testRead(argv[1]); else testRead();  break;
    case 2:  if(argn>1) testDot(argv[1]); else testDot();  break;
  }
  
  return 0;
}
