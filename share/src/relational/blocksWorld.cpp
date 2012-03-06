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

void generateOrsFromSample(ors::Graph& ors, const MT::Array<arr>& sample) {
  for (int i = ors.bodies.N - 1; i >= 0; --i) {
    if (ors.bodies(i)->name.p[0] == 'c' &&
        ors.bodies(i)->name.p[1] == 'y' &&
        ors.bodies(i)->name.p[2] == 'l') {
      ors.bodies.remove(i);  
    }
  }
  for (uint i = 0; i < sample.N; i+=2) {
    ors::Body* body = new ors::Body;
    createCylinder(*body, sample(0,i), ARR(1., 0., 0.), ARR(0.1, 0.1, sample(0,i+1)(0), 0.0375)); 
    body->name = "cyl"; 
    ors.bodies.append(body);
  }
}

void generateBlocksSample(MT::Array<arr>& sample, const uint numOfBlocks) {
  sample.clear();
  for (uint i = 0; i < numOfBlocks; ++i) {
    arr center3d = ARR(0., -.8) + randn(2,1) * 0.3;

    int t = rand() % 100;
    double blocksize = 0.1 + (rand() % 100) / 500.;
    double towersize = 0.69 + blocksize;
    center3d.append(0.69 + 0.5*blocksize);
    center3d.resize(3);

    sample.append(center3d);
    sample.append(ARR(blocksize));
    while (t < 50 && i < numOfBlocks-1) {
      i++;
      center3d = center3d + randn(3,1) * 0.02;
      double blocksize = 0.1 + (rand() % 100) / 500.;
      center3d(2) = 0.5*blocksize + towersize;
      towersize += blocksize;

      sample.append(center3d);
      sample.append(ARR(blocksize));

      t = rand() % 100;
    }
  }
	sample.reshape(1,2*numOfBlocks);
  //sample.reshape(1,numOfBlocks);
}

void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color) {
  arr size = ARR(0.1, 0.1, 0.108, 0.0375);
  createCylinder(cyl, pos, color, size);
}

void createCylinder(ors::Body& cyl, const ors::Vector& pos, const arr& color, const arr& size) {
  ors::Transformation t;
  t.pos = pos;
  ors::Shape* s = new ors::Shape();
  for (uint i = 0; i < 4; ++i) { s->size[i] = size(i);}
  s->type = ors::cylinderST;
  for (uint i = 0; i < 3; ++i) s->color[i] = color(i);
  s->body = &cyl;
  
  cyl.shapes.append(s);
  cyl.X = t; 
}

}
