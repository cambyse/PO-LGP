#include <Core/util.h>

int main( int argc, char **argv ) {

  Inotify I;
  while(true){
    //I.waitReport();
    I.waitForModification("main.cpp");
    cout <<"here" <<endl;
  }

  return 0;
}
