#define MT_IMPLEMENT_TEMPLATES

#include <MT/util.h>
#include <MT/graphview.h>

void test(){
  HyperGraph G;
  cout <<"reading graph..." <<endl;
  MT::load(G,"graph");
  writeDot(G);
  
  GraphView gv(G);
  gv.watch();
}

int main(int argc, char **argv){
  test();

  return 0;
}

