#include <stdlib.h>
#include <MT/roboticsCourse.h>
#include <MT/opengl.h>


int main(int argc,char **argv){
  CarSimulator S;
  arr u(2),y_meassured;
  uint N=S.landmarks.d0; //number of landmarks;

  //you have access to:
  //S.observationNoise (use when evaluating a particle likelihood)
  //S.dynamicsNoise (use when propagating a particle)
  
  S.gl->watch();
  for(uint t=0;t<1000;t++){
    u = ARR(.1, .2); //control signal
    S.step(u);
    S.getRealNoisyObservation(y_meassured);

    //get the linear observation model
    arr C,c;
    //NOTE: Access to S.x|y|theta is cheating!! In the solution
    //you need to plug in the current mean estimate of the car state!
    S.getLinearObservationModelGivenState(C, c, S.x, S.y, S.theta);
    cout <<C <<endl <<c <<endl;


    //sanity check of the linear observation model -- don't use this code in the solution!
    arr y_mean;
    arr landmarks = S.landmarks;
    landmarks.reshape(2*N);
    S.getMeanObservationGivenState(y_mean, S.x, S.y, S.theta);
    cout <<"linear model error = " <<maxDiff(y_mean, C*landmarks + c) <<endl; //this should be zero!!

    cout <<u <<endl <<y_meassured <<endl;
  }

  return 0;
}
