#include <signal.h>
#include <Gui/opengl.h>
#include <Hardware/schunk/schunk.h>

int main( int argc, char **argv ){
  signal(SIGINT,schunkEmergencyShutdown);

  SchunkSkin skin;
  skin.open();
  skin.report();

  uint M=6,X=6,Y=14;
  uint16A sense(M,Y,X);
  sense.setZero();

  OpenGL gl;
  byteA img;

  arr y;
  for(uint i=0;i<1000;i++){
    skin.getImage(img);
    gl.watchImage(img,false,5);
    
    skin.getIntegrals(y);
    cout <<"image max= " <<(int)img.max() <<" integrals = " <<y <<endl;
  }

  skin.close();
}
