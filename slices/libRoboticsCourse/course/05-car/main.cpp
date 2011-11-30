#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/opengl.h>


int main(int argc,char **argv){
  CarSimulator S;

  arr u(2),Y,Yparticle;
  
  S.gl->watch();
  for(uint t=0;t<1000;t++){
    u = ARR(.1, .2); //control signal
    S.step(u);
    S.meassureCurrentLandmarks(Y);
    S.getTrueLandmarksInState(Yparticle, S.x, S.y, S.theta); //that's cheating since we use the internal information of true state
    cout <<u <<Y <<Yparticle <<endl;
  }
  
  return 0;
}
