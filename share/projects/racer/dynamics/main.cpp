#include <Ors/ors.h>
#include <Gui/opengl.h>

int main(int argc, char **argv){

  OpenGL gl;
  ors::KinematicWorld G;
  init(G, gl, "racer.ors");

  uint n=G.joints.N;
  double tau=.001;
  arr x(n), xdot(n), xddot(n), u(n), M, Minv, F;
  x.setZero(); x(3) += .1;
  xdot.setZero();
  for(uint t=0;t<1000;t++){
    u.setZero();
    G.clearForces();
    G.gravityToForces();
    G.dynamics(xddot, xdot, u);
    
    //if(s->dynamicNoise) rndGauss(s->qddot, s->dynamicNoise, true);

    //Euler integration (Runge-Kutte4 would be much more precise...)
    x    += tau * xdot;
    xdot += tau * xddot;

    G.setJointState(x, xdot);
    G.calcBodyFramesFromJoints();
    gl.update();
  }

  return 0;
}
