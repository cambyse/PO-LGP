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
  dir += 0.002*randn(2,1);
  pos += dir;
  if(pos(0,0) > 1 || pos(0,0) < 0) dir(0,0) *= -1;
  if(pos(0,1) > 1 || pos(0,1) < 0) dir(0,1) *= -1;
  r = pos + 0.001*randn(2,1);
  z2 = z1;
  z1 = r;
}

Gaussian g;
double lin_weight(const arr &particle, const arr &measurement) {
  return g.evaluate(particle);
}

void gaussian_control(arr &after, const arr &before) {
  after = before + (dir)+0.009*randn(before.d0,1); 
}

int main(int argc, char **argv) {
  g.C = 0.001*eye(2);
  z1 = z2 = ARR(0., 0.);
  pos = ARR(0.5, 0.4);
  dir = ARR(.004, .005);
  pos.reshape(1,2);
  dir.reshape(1,2);

  Measurement m("M");
  Particles p("P");

  ParticleFilter f;
  f.measurement = &m;
  f.particles = &p;
  arr mean = ARR(0.5, 0.4);
  mean.reshape(1,2);
  f.init(mean, 1000);
  f.add_measurement(z1);

  f.weight = &lin_weight;
  f.control = &gaussian_control;

  gnuplot("set xrange [0:1.5]; set yrange [0:1.5]");
  for(int i=0; i<1000; ++i) {
    arr z;
    measure(z);
    g.c = z;
    f.add_measurement(z);
    f.threadStep();
    MT::wait(0.1);
    p.readAccess(NULL);
    arr mean = sum(p.particles, 0)*(1./p.particles.d0);
    MT::IOraw=true;
    MT::save(p.particles, "z.pltX");
    p.deAccess(NULL);
    MT::save(mean, "sum");
    MT::save(pos, "pos");
    MT::save(z, "z");
    //gnuplot("plot 'pos' us 2:3 with points, 'z' us 2:3 with points, 'sum' us 2:3 with points");
    gnuplot("plot 'z.pltX' us 1:2 with points lc rgb '#AAAAAA', 'pos' us 2:3 with points, 'z' us 2:3 with points, 'sum' us 2:3 with points");
    MT::IOraw=false;
    MT::wait(0.01);
  }
  f.threadClose();
}
