#include "blocksWorld.h"
#include <cstdlib>

void generateBlocksSample(MT::Array<arr>& sample, const uint numOfBlocks) {
  sample.clear();
  for (uint i = 0; i < numOfBlocks; ++i) {
    arr center3d = ARR(0., -.8) + randn(2,1) * 0.7;
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
}

