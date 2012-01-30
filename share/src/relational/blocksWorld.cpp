#include "blocksWorld.h"

#include <MT/ors.h>

#include <cstdlib>

namespace relational {
void generateOrsBlocksSample(ors::Graph& ors, const uint numOfBlocks) {
  MT::Array<arr> pos;
  generateBlocksSample(pos, numOfBlocks);
  for (uint i = 0; i < numOfBlocks; ++i) {
    ors::Body* body = new ors::Body;
    createCylinder(*body, pos(i), ARR(1., 0., 0.)); 
    body->name = "cyl" + i;
    ors.bodies.append(body);
  }

}

void generateBlocksSample(MT::Array<arr>& sample, const uint numOfBlocks) {
  sample.clear();
  for (uint i = 0; i < numOfBlocks; ++i) {
    arr center3d = ARR(0., -.8) + randn(2,1) * 0.3;
    center3d.append(0.74);
    center3d.resize(3);

    sample.append(center3d);

    int t = rand() % 100;
    int tower = 0;
    while (t < 50 && i < numOfBlocks-1) {
      i++;
      tower++;
      center3d = center3d + randn(3,1) * 0.02;
      center3d(2) = 0.74 + tower * 0.108;
      t = rand() % 100;

      sample.append(center3d);
    }
  }
  sample.reshape(1,numOfBlocks);
}

void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color) {
  ors::Transformation t;
  t.pos = pos;
  ors::Shape* s = new ors::Shape();
  s->size[0] = 0.1;
  s->size[1] = 0.1;
  s->size[2] = 0.108;
  s->size[3] = 0.0375;

  s->type = ors::cylinderST;
  for (uint i = 0; i < 3; ++i) s->color[i] = color(i);
  s->body = &cyl;
  
  cyl.shapes.append(s);
  cyl.X = t; 
}
}
