#include <MT/keyValueGraph.h>
#include <MT/graphview.h>

void testRead(const char *filename="../rules/coffee_shop.fg"){
  KeyValueGraph H;

  cout <<"reading graph..." <<endl;
  MT::load(H,filename);
  //cout <<H <<endl;
  H.sortByDotOrder();
  cout <<H <<endl;
  H.writeDot();

  GraphView gv(H);
  //MT::wait(2.);
  gv.watch();
}

void testBasic(){
  KeyValueGraph G;
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

  listWrite(G.getParents(0),cout);
  cout <<"--------------------------" <<endl;
  listWrite(G.getParents(1),cout);
  cout <<"--------------------------" <<endl;
}

int main(int argc,char** argv){
  MT::initCmdLine(argc,argv);
  
  int mode = MT::getParameter<int>("mode",1);
  switch(mode){
    case 0:  testBasic();  break;
    case 1:  if(argc>1) testRead(argv[1]); else testRead();  break;
  }
  
  return 0;
}
