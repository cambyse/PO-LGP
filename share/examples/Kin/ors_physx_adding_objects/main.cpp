/* This demonstates how to dynamically add objects to ors and physx.
 * It's important to call the PhysX.syncWithOrs() function once the ors graph
 * was modified.
 */

#include <Kin/kin_physx.h>
#include <Gui/opengl.h>

void addRandomObject(mlr::KinematicWorld &ors) {
  mlr::Body *b = new mlr::Body(ors);
  b->X.setRandom();
  b->X.pos.z += 1.;

  mlr::Shape *s = new mlr::Shape(ors, *b);
  s->type = mlr::ST_sphere;
  s->size[0] = .1; s->size[1] = .1; s->size[2] = .1; s->size[3] = .1;
}


void TEST(OrsPhysx) {
  mlr::KinematicWorld graph;
  addRandomObject(graph);

  OpenGL glPh("PhysX");
  bindOrsToPhysX(graph, glPh, graph.physx());

  for (uint t = 0; t < 500; t++) {

    // add objects periodically
    if (t % 30 == 0) {
      addRandomObject(graph);
      graph.physx().pullFromPhysx();
      cout << "adding object: " << endl;
    }

    graph.physx().step();
    glPh.update();
    graph.gl().update();
  }
}

int MAIN(int argc, char **argv) {
  testOrsPhysx();

  return 0;
}
