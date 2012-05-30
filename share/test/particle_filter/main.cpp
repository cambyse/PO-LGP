#include <JK/particle_filter/particle_filter.h>
#include <iostream>
#include <MT/array.h>
#include <MT/util.h>
#include <MT/gauss.h>

arr pos;
arr dir;
arr z;

void measure(arr& r) {
  pos += dir;
  if(pos(0,0) > 1 || pos(0,0) < 0) dir(0,0) *= -1;
  if(pos(0,1) > 1 || pos(0,1) < 0) dir(0,1) *= -1;
  r = pos + 0.1*randn(2,1);
}

double lin_weight(const arr &particle) {
  Gaussian g;
  g.c = z;
  g.C = eye(2);
  return g.evaluate(particle);
}
int main(int argc, char **argv) {
  pos = ARR(0.5, 0.4);
  dir = ARR(.0004, .0005);
  z = dir;
  pos.reshape(1,2);
  dir.reshape(1,2);
  ParticleFilter f(1000, 2);
  f.weight = &lin_weight;
  gnuplot("set xrange [0:1.5]; set yrange [0:1.5]");
  for(int i=0; i<10000; ++i) {
    measure(z);
    f.step();
    std::cout << "step" << std::endl;
    MT::IOraw=true;
    MT::save(f.particles, "z.pltX");
    MT::save(pos, "pos");
    MT::save(z, "z");
    gnuplot("plot 'z.pltX' us 1:2 with points, 'pos' us 2:3 with points, 'z' us 2:3 with points");
    MT::IOraw=false;
    MT::wait(0.01);
  }
}
