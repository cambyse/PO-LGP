#ifndef _SAMPLER_H_
#define _SAMPLER_H_

#include <MT/array.h>
#include <relational/blocksWorld.h>

template<class S> class Sampler {
  public:
    virtual void sample(S& sample) = 0;
};

class BlocksWorldSampler : public Sampler<MT::Array<arr> > {
  public: 
    virtual void sample(MT::Array<arr>& sample) { relational::generateBlocksSample(sample, 2); }   
};

#endif //_SAMPLER_H_
