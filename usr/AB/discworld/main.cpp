#include <Ors/ors_physx.h>
#include <Gui/opengl.h>
#include <Gui/plot.h>

#include "discworld.h"

void setup_T(DiscWorld &dw) {
  dw.clear();
  dw.setTMode(300);
  dw.addBody(10);
  dw.play();
}

void setup_G(DiscWorld &dw) {
  dw.clear();
  dw.setGMode();
  dw.addGoal(4);
  dw.addBody(10);
  dw.play();
}

void setup_collision(DiscWorld &dw) {
  dw.clear();
  dw.setGMode();
  dw.addGoal(0, 0);
  dw.addGoal(-4, 0);
  dw.addBody(-4, 0);
  dw.addBody(-2, 0);
  dw.addBody(2, 0);
  dw.play();
}

int main(int argc, char** argv) {
  MT::rnd.clockSeed();
  //MT::rnd.seed(0);

  DiscWorld dw;
  dw.setSpeed(.03);
  dw.setLWin(5);

  setup_T(dw);
  dw.replay();
  
  setup_G(dw);
  dw.replay(); // why does 1 have 3 peaks?!

  setup_collision(dw);
  dw.replay();
}

