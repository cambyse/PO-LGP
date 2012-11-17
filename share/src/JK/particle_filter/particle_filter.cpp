#include "particle_filter.h"
#include <cstdlib>

class sParticleFilter {
  public:
    ParticleFilter *p;
    sParticleFilter(ParticleFilter *p) : p(p) {};

    double cur_sum, cur_particle_sum, step;
    int cur_particle;

    void reset_drawing(const arr &particles, const arr& measurement) {
      cur_particle = 0;
      double sum = 0;
      for(uint i=0; i<particles.d0; ++i) {
         sum += p->weight(particles[i], measurement);
      }
      step = sum / (particles.d0 + 1);
      cur_particle_sum = p->weight(particles[0], measurement);
      cur_sum = step / 2.;
    }

    arr draw(const arr &particles, const arr& measurement) {
      cur_sum += step;
      if (cur_sum > cur_particle_sum) {
        cur_particle++;
        cur_particle_sum += p->weight(particles[cur_particle], measurement);
      }
      return particles[cur_particle];
    }
};

ParticleFilter::ParticleFilter() : Process("ParticleFilter"), s(new sParticleFilter(this)) {
}

void ParticleFilter::init(const arr &mean, int num_of_particles) {
  arr next;
  for (int i=0; i<num_of_particles; ++i) {
    next.append(mean+0.001*randn(1,mean.d1));  
  }
  next.reshape(num_of_particles, mean.d1);
  particles->writeAccess(this);
  copy(particles->particles, next);
  particles->deAccess(this);
}

void ParticleFilter::open() {}
void ParticleFilter::close() {}
void ParticleFilter::step() {
  //get current measurement
  arr measurement_;
  measurement->writeAccess(this);
  //if(measurement->measurements.isEmpty() {
    //measurement->deAccess(this);
    //return;
  //}
  copy(measurement_, measurement->measurement());
  measurement->deAccess(this);

  // get copy of particles
  arr particles_;
  particles->readAccess(this);
  copy(particles_, particles->particles);
  particles->deAccess(this);

  // compute particle filter
  for (uint i=0; i<particles_.d0; ++i) {
    arr x;
    control(x, particles_[i]);
    particles_[i] = x;
  }
  s->reset_drawing(particles_, measurement_);
  arr next;
  for (uint i=0; i<particles_.d0; ++i) {
    next.append(s->draw(particles_, measurement_));
  }
  next.reshape(particles_.d0, particles_.d1);

  // write particles back to Variable
  particles->writeAccess(this); 
  copy(particles->particles, next);
  particles->deAccess(this);
}

void ParticleFilter::add_measurement(const arr &measurement) {
  this->measurement->writeAccess(this);
  copy(this->measurement->measurement, measurement);
  this->measurement->deAccess(this);
}
ParticleFilter::~ParticleFilter() {
  delete s;  
}
