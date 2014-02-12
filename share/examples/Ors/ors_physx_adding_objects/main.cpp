/* This demonstates how to dynamically add objects to ors and physx.
 * It's important to call the PhysX.syncWithOrs() function once the ors graph
 * was modified.
 */

#include <Ors/ors_physx.h>
#include <Gui/opengl.h>

void addRandomObject(ors::KinematicWorld &ors) {
  ors::Body *b = new ors::Body(ors);
  b->X.setRandom();
  b->X.pos.z += 1.;

  ors::Shape *s = new ors::Shape(ors, *b);
  s->type = ors::sphereST;
  s->size[0] = .1; s->size[1] = .1; s->size[2] = .1; s->size[3] = .1;
}


void TEST(OrsPhysx) {
  ors::KinematicWorld graph;
  addRandomObject(graph);

  OpenGL glMy;
  OpenGL glPh("PhysX");
  PhysXInterface physx;
  bindOrsToOpenGL(graph, glMy);
  bindOrsToPhysX(graph, glPh, physx);

  for (uint t = 0; t < 500; t++) {

    // add objects periodically
    if (t % 30 == 0) {
      addRandomObject(graph);
      physx.syncWithOrs();
      cout << "adding object: " << endl;
    }

    physx.step();
    glPh.update();
    glMy.update();
  }
}

int MAIN(int argc, char **argv) {
  testOrsPhysx();

  return 0;
}
