#include "al_random.h"    
#include <JK/utils/sampler.h>
class sRandomAL {
  public:
    int classes;
    Sampler<MT::Array<arr> >* sampler;

};
void RandomAL::setTrainingsData(const MT::Array<arr>& data, const intA& classes) {
  s->classes = classes.max();
}
void RandomAL::addData(const MT::Array<arr>& data, const int class_) {
  if (class_ > s->classes) s->classes = class_;
}
int RandomAL::nextSample(MT::Array<arr>& sample) const {
  s->sampler->sample(sample);  
  return 1;
}
int RandomAL::classify(const MT::Array<arr>& data, const int set) const {
  return rand() % s->classes;
}

RandomAL::RandomAL(Sampler<MT::Array<arr> >* sampler) :
  s(new sRandomAL) {
    s->sampler = sampler;
    s->classes = 0;
  }
