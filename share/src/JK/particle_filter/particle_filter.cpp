#include "particle_filter.h"
#include <cstdlib>

class sParticleFilter {
  public:
    ParticleFilter *p;
    sParticleFilter(ParticleFilter *p) : p(p) {};
    double cur_sum;
    double cur_particle_sum;
    int cur_particle;
    double step;
    void reset_drawing() {
      cur_particle = 0;
      double sum = 0;
      for(int i=0; i<p->particles.d0; ++i) {
         sum += p->weight(p->particles[i]);
      }
      step = sum / (p->particles.d0 + 1);
      cur_particle_sum = p->weight(p->particles[0]);
      cur_sum = step / 2.;
    }
    arr draw() {
      cur_sum += step;
      if (cur_sum > cur_particle_sum) {
        cur_particle++;
        cur_particle_sum += p->weight(p->particles[cur_particle]);
      }
      return p->particles[cur_particle];
    }
};

ParticleFilter::ParticleFilter(int num_of_particles, int dim) : s(new sParticleFilter(this)), num_of_particles(num_of_particles), dim(dim) {
  for (int i=0; i< num_of_particles; ++i) {
    particles.append(rand(dim,1));
  }
  particles.reshape(num_of_particles, dim);
}

void ParticleFilter::step() {
  arr next;
  s->reset_drawing();
  for (int i=0; i<num_of_particles; ++i) {
    next.append(s->draw() + 0.005*randn(dim,1));
  }
  next.reshape(num_of_particles, dim);
  copy(particles, next);
}
ParticleFilter::~ParticleFilter() {
  delete s;  
}
