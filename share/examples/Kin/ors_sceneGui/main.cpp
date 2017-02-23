#include <Kin/kin_sceneGui.h>

int main(int argc,char **argv){

  const char *file="situation.ors";
  if(argc<2){
    cout <<"opening standard file `" <<file <<"'" <<endl;
  }else file=argv[1];
  
  mlr::KinematicWorld ors;
  ors.init(file);
  
  OrsSceneGui gui(ors);
  gui.edit();
  
  return 0;
}
