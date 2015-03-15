#define MT_IMPLEMENT_TEMPLATES

#include <MT/util.h>
#include <MT/graphview.h>

void TEST(){
  HyperGraph G;
  cout <<"reading graph..." <<endl;
  G <<FILE("graph");
  writeDot(G);
  
  GraphView gv(G);
  gv.watch();
}

int main(int argc, char **argv){
  test();

  return 0;
}

