#include "particle_filter.h"
#include <cstdlib>

class sParticleFilter {
  public:
    ParticleFilter *p;
    sParticleFilter(ParticleFilter *p) : p(p) {};
    arr measurement;

    double cur_sum, cur_particle_sum, step;
    int cur_particle;

    void reset_drawing() {
      cur_particle = 0;
      double sum = 0;
      for(int i=0; i<p->particles.d0; ++i) {
         sum += p->weight(p->particles[i], measurement);
      }
      step = sum / (p->particles.d0 + 1);
      cur_particle_sum = p->weight(p->particles[0], measurement);
      cur_sum = step / 2.;
    }

    arr draw() {
      cur_sum += step;
      if (cur_sum > cur_particle_sum) {
        cur_particle++;
        cur_particle_sum += p->weight(p->particles[cur_particle], measurement);
      }
      return p->particles[cur_particle];
    }
};

ParticleFilter::ParticleFilter(int num_of_particles, int dim) : s(new sParticleFilter(this)) {
  for (int i=0; i< num_of_particles; ++i) {
    particles.append(rand(dim,1));
  }
  particles.reshape(num_of_particles, dim);
}

void ParticleFilter::step() {
  for (int i=0; i<particles.d0; ++i) {
    arr x;
    control(x, particles[i]);
    particles[i] = x;
  }
  s->reset_drawing();
  arr next;
  for (int i=0; i<particles.d0; ++i) {
    next.append(s->draw());
  }
  next.reshape(particles.d0, particles.d1);
  copy(particles, next);
}

void ParticleFilter::add_measurement(const arr &measurement) {
  copy(s->measurement, measurement);
}
ParticleFilter::~ParticleFilter() {
  delete s;  
}
