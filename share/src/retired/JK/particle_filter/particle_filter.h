#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_
#include <Core/array.h>
#include <Core/array_t.h>
#include <System/biros.h>

class Particles : public Variable {
  public:
    Particles(const char *name) : Variable(name) {reg_particles();}
    FIELD(arr, particles);  
};

class Measurement : public Variable {
  public:
    Measurement(const char *name) : Variable(name) {reg_measurement();}
    FIELD(arr, measurement);  
};

class ParticleFilter :public Process {
  public:
    class sParticleFilter *s;
    
    double (*weight)(const arr &particle, const arr &measurement);
    void (*control)(arr &after, const arr &before);
    
    Particles *particles;
    Measurement *measurement;

    ParticleFilter();
    ~ParticleFilter();
    void init(const arr &mean, int num_of_particles);
    void open();
    void step();
    void close();
    void add_measurement(const arr &measurement);
};
#endif
