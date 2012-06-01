#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_
#include <MT/array.h>

class ParticleFilter {
  public:
    class sParticleFilter *s;
    
    double (*weight)(const arr &particle, const arr &measurement);
    void (*control)(arr &after, const arr &before);
    
    arr particles;

    ParticleFilter(int num_of_particles, int dim);
    ~ParticleFilter();
    void init(const arr &mean);
    void step();
    void add_measurement(const arr &measurement);
};
#endif
