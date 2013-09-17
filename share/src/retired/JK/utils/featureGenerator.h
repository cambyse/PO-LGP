#ifndef _FEATURE_GENERATOR_H_
#define _FEATURE_GENERATOR_H_

#include <Core/array.h>

template<class Sample>
class FeatureGenerator {
  public:
    virtual void makeFeatures(Sample& features, const Sample& data) = 0;
    virtual void getSize(uintA& size) = 0;
};
class DistanceFeatureGenerator : public FeatureGenerator<arr> {
  public:
    virtual void makeFeatures(arr& features, const arr& data);
    virtual void getSize(uintA& size) { size = TUP(1,4); }
};
class TrayFeatureGenerator : public FeatureGenerator<arr> {
  public:
    virtual void makeFeatures(arr& features, const arr& data);
    virtual void getSize(uintA& size) { size = TUP(1,4); }
};
class CubicFeatureGenerator : public FeatureGenerator<arr> {
  public:
    virtual void makeFeatures(arr& features, const arr& data);
    virtual void getSize(uintA& size) { size = TUP(1,4); }
};
class SimpleFeatureGenerator : public FeatureGenerator<arr> {
  public:
    virtual void makeFeatures(arr& features, const arr& data);
    virtual void getSize(uintA& size) { size = TUP(1,2); }
};
class UprightFeatureGenerator : public FeatureGenerator<arr> {
  public:
    virtual void makeFeatures(arr& features, const arr& data);
    virtual void getSize(uintA& size) { size = TUP(1,1); }
};

#endif

