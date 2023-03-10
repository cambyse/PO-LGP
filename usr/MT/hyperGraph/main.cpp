#include <Core/graph.h>
#include <Gui/graphview.h>

void testRead(const char *filename="../rules/coffee_shop.fg"){
  Graph H;

  cout <<"reading graph..." <<endl;
  H <<FILE(filename);
  //cout <<H <<endl;
  H.sortByDotOrder();
  cout <<H <<endl;
  H.writeDot();

  GraphView gv(H);
  //mlr::wait(2.);
  gv.watch();
}

void TEST(Basic){
  Graph G;
  G.append(TUP());
  cout <<G <<"\n--------------------------" <<endl;
  G.append(TUP());
  cout <<G <<"\n--------------------------" <<endl;
  G.append(TUP(1,1));
  cout <<G <<"\n--------------------------" <<endl;
  G.append(TUP(1,1));
  cout <<G <<"\n--------------------------" <<endl;
  G.append(TUP(0,1));
  cout <<G <<"\n--------------------------" <<endl;

  listWrite(G(0)->parentOf,cout);
  cout <<"\n--------------------------" <<endl;
  listWrite(G(1)->parentOf,cout);
  cout <<"\n--------------------------" <<endl;
}

int main(int argc,char** argv){
  mlr::initCmdLine(argc,argv);
  
  int mode = mlr::getParameter<int>("mode",1);
  switch(mode){
    case 0:  testBasic();  break;
    case 1:  if(argc>1) testRead(argv[1]); else testRead();  break;
  }
  
  return 0;
}
