#include <JK/particle_filter/particle_filter.h>
#include <JK/utils/util.h>
#include <iostream>
#include <MT/array.h>
#include <MT/util.h>
#include <MT/gauss.h>

arr pos;
arr dir;
arr z1;
arr z2;

void measure(arr& r) {
  pos += dir;
  if(pos(0,0) > 1 || pos(0,0) < 0) dir(0,0) *= -1;
  if(pos(0,1) > 1 || pos(0,1) < 0) dir(0,1) *= -1;
  r = pos + 0.00001*randn(2,1);
  z2 = z1;
  z1 = r;
}

double lin_weight(const arr &particle, const arr &measurement) {
  Gaussian g;
  g.c = measurement;
  g.C = 0.00001*eye(2);
  return g.evaluate(particle);
}

void gaussian_control(arr &after, const arr &before) {
  JK_DEBUG(z2 - z1);
  after = before + (dir)+0.005*randn(before.d0,1); 
}

int main(int argc, char **argv) {
  z1 = z2 = ARR(0., 0.);
  pos = ARR(0.5, 0.4);
  dir = ARR(.004, .005);
  pos.reshape(1,2);
  dir.reshape(1,2);
  ParticleFilter f(1000, 2);
  f.weight = &lin_weight;
  f.control = &gaussian_control;
  gnuplot("set xrange [0:1.5]; set yrange [0:1.5]");
  for(int i=0; i<10000; ++i) {
    arr z;
    measure(z);
    f.add_measurement(z);
    f.step();
    arr mean = sum(f.particles, 0)*(1./f.particles.d0);
    MT::IOraw=true;
    MT::save(f.particles, "z.pltX");
    MT::save(mean, "sum");
    MT::save(pos, "pos");
    MT::save(z, "z");
    //gnuplot("plot 'pos' us 2:3 with points, 'z' us 2:3 with points, 'sum' us 2:3 with points");
    gnuplot("plot 'z.pltX' us 1:2 with points lc rgb '#AAAAAA', 'pos' us 2:3 with points, 'z' us 2:3 with points, 'sum' us 2:3 with points");
    MT::IOraw=false;
    MT::wait(0.01);
  }
}
