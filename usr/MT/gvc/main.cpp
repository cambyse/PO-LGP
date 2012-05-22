#define MT_IMPLEMENT_TEMPLATES
#include <MT/util.h>
#include <gtk/gtk.h>
#undef MIN
#undef MAX

#include "graphvizGtk.h"

void test(){
  HyperGraph G;
  cout <<"reading graph..." <<endl;
  MT::load(G,"graph");
  writeDot(G);
  
  GraphvizGtk gv(G);
  gv.watch();
  
}

int main(int argc, char **argv){
  gtk_init (&argc, &argv);
  
  test();

  return 0;
}

