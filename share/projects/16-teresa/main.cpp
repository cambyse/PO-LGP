#include <FOL/fol.h>

//===========================================================================

void testLoadData(const char* filename="data"){
  Graph G(filename);
  G.checkConsistency();

  cout <<"loaded:" <<G <<endl;
}

//===========================================================================


int main(int argn, char** argv){
  if(argn>1)
    testLoadData(argv[1]);
  else
    testLoadData();
  cout <<"BYE BYE" <<endl;

  return 0;
}
