#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_
#include <MT/array.h>
#include <biros/biros.h>

class Particles : public Variable {
  public:
    Particles(const char *name) : Variable(name) {reg_particles();}
    FIELD(arr, particles);  
}

class Measurement : public Variable {
  public:
    Measurement(const char *name) : Variable(name) {reg_measurement();}
    FIELD(arr, measurement);  
}

class ParticleFilter :public Process {
  public:
    class sParticleFilter *s;
    
    double (*weight)(const arr &particle, const arr &measurement);
    void (*control)(arr &after, const arr &before);
    
    Particles particles;
    Measurement measurement;

    ParticleFilter(int num_of_particles, int dim);
    ~ParticleFilter();
    void step();
    void add_measurement(const arr &measurement);
};
#endif
