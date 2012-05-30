#include <MT/array.h>

class ParticleFilter {
  public:
    class sParticleFilter *s;
    arr particles;
    double (*weight)(const arr& particle);
    int num_of_particles; 
    int dim;

    ParticleFilter(int num_of_particles, int dim);
    ~ParticleFilter();
    void step();
};
