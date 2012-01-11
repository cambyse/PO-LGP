#include <MT/ors_sceneGui.h>

int main(int argn,char **argv){

  const char *file="situation.ors";
  if(argn<2){
    cout <<"opening standard file `" <<file <<"'" <<endl;
  }else file=argv[1];
  
  ors::Graph ors;
  ors.init(file);
  
  OrsSceneGui gui(ors);
  gui.edit();
  
  return 0;
}
