#ifndef _FEATURE_GENERATOR_H_
#define _FEATURE_GENERATOR_H_

#include <MT/array.h>

template<class Sample>
class FeatureGenerator {
  public:
    virtual void makeFeatures(Sample& features, const Sample& data) = 0;
};
class DistanceFeatureGenerator : public FeatureGenerator<arr> {
  public:
    virtual void makeFeatures(arr& features, const arr& data);
};
class CubicFeatureGenerator : public FeatureGenerator<arr> {
  public:
    virtual void makeFeatures(arr& features, const arr& data);
};

#endif

