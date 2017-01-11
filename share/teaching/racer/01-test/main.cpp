#include <stdlib.h>
#include <Kin/roboticsCourse.h>
#include <Gui/opengl.h>
#include <Algo/kalman.h>
#include <Motion/motion.h>

#include <Hardware/racer/racer.h>

void TestDynamics(){
  Racer R;
  for (uint t=0; t<4000; t++){
    R.stepDynamics(0.0);
    R.gl().text.clear() <<t <<" ; " <<R.q(0) << " ; " <<R.q(1);
    R.gl().update();
    R.getEnergy();
  }
}

void CheckGradients(){
  Racer R;
  arr x(2,2);
  for (uint t=0; t<10; t++){
    rndUniform(x, -.1, .1);
    R.u = 0.; //rnd.uni(-.1,.1);
    cout <<"dyn: q=" <<R.q; checkJacobian(R.dynamicsFct(), x, 1e-4);
    cout <<"obs: q=" <<R.q; checkJacobian(R.observationFct(), x, 1e-4);
  }
}

void TestSimpleControl(){ //uses the true STATE(!) for control, not the noise state estimate
  Racer R;
  R.q(1)=.5;
  R.noise_dynamics = 0;//.1;

  for (uint t=0; t<10000; t++){
    //-- reference (0 or 1)
    double x_ref=(t/200)&1;

    //-- control
    arr k = ARR(3.1623,   8.6762,   2.6463,   2.0126);
    arr x = cat(R.q, R.q_dot);
    x(0) -= x_ref;
    double u = scalarProduct(k, x);

    //-- dynamics
    R.stepDynamics(u);
    R.gl().update();
  }
}

void testBenchmark(){
  arr k = ARR(0, 3.1623,   8.6762,   2.6463,   2.0126);

  RacerBalancingBenchmark.display=true;
  RacerBalancingBenchmark.fs(NoArr, NoArr, k);
  RacerBalancingBenchmark.display=false;
}


int main(int argc,char **argv){
//  TestDynamics();
  CheckGradients();
//  TestSimpleControl();
  testBenchmark();

  return 0;
}
