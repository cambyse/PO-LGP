#include "naiveBayesClassificator.h"
#include "blocksWorld.h"

#include <MT/array.h>
#include <MT/array_t.cpp>
#include <MT/gauss.h>
#include <JK/util.h>

#include <inttypes.h>
#include <cmath>



class sNaiveBayesClassificator {
public:
  MT::Array<arr> features; 
  intA classes;

  uint8_t numOfClasses;

  MT::Array<arr> means;
  MT::Array<arr> variances;

  double getPrior(const int feature, const arr& value, const int givenClass);
  double getClassProbability(const int _class);

  void computeMeansAndVariances(const int givenClass, const int feature);

  void getProbabilities(arr& probabilities, const MT::Array<arr>& features, int set = 0);

  void rejectionSampling(MT::Array<arr>& sample, double& p, const int class1, const int class2);

  void findBestStartPoint(arr& startPoint, const int feature, const int class1, const int class2, const arr& pos1, const arr& pos2, double eps) const;
  void nextSample(arr& nextSample, double& p, const int class1, const int class2, const int feature) const;
};

NaiveBayesClassificator::NaiveBayesClassificator() {
  s = new sNaiveBayesClassificator;  
}

NaiveBayesClassificator::~NaiveBayesClassificator() {
  delete s;  
}

int NaiveBayesClassificator::classify(const MT::Array<arr>& features, int set) const {
  JK_DEBUG(features.d0);
  JK_DEBUG(features);
  JK_DEBUG(s->features.d1);
  CHECK(features.d0 == s->features.d1, "Feature vector is of different size than trainings data.");
  arr probabilities;
  s->getProbabilities(probabilities, features, set);
  return probabilities.maxIndex();
}

void sNaiveBayesClassificator::getProbabilities(arr& probabilities, const MT::Array<arr>& feature, int set) {
  probabilities.resize(numOfClasses);
  for (uint8_t i = 0; i < numOfClasses; ++i) {
    probabilities(i) = 1; // normally this should be the class probability, but 
                          // since it would be divided later, we can leave it out
    for (uint8_t j = 0; j < feature.d1; ++j) {
      // this is actually a probability density value, not a probability. The
      // probability of a single point is 0. However it is a good estimation.
      // But note that it can be bigger than 1.
      probabilities(i) *= getPrior(j, feature(j), i); 
    }
  }
  // this holds if we assume that every data point must be in exactly one class
  probabilities = probabilities * (1./(sum(probabilities)));
}

void NaiveBayesClassificator::setTrainingsData(const MT::Array<arr >& features, const intA& classes) {
  s->features = features;
  s->classes = classes;

  s->numOfClasses = 1;
  for (uint8_t i = 0; i < s->classes.N; ++i) {
    if (classes(i) + 1 > s->numOfClasses) s->numOfClasses = s->classes(i) + 1;
  }

  s->means.resize(s->features.d1, s->numOfClasses);
  s->variances.resize(s->features.d1, s->numOfClasses);

  for (uint8_t f = 0; f < s->features.d1; ++f) {
    for (uint8_t c = 0; c < s->numOfClasses; ++c) {
      s->computeMeansAndVariances(c, f);
    }
  }
}

void NaiveBayesClassificator::addData(const MT::Array<arr>& data, const int class_) {
  JK_DEBUG(data);
  CHECK(data.d0 == s->features.d1, "The new feature vector does not match the number of features for data in this classificator");
  int d1 = s->features.d1;
  s->features.append(data);
  s->features.reshape(s->features.N/d1, d1);
  s->classes.append(class_);

  for (uint8_t f = 0; f < s->features.d1; ++f) {
    for (uint8_t c = 0; c < s->numOfClasses; ++c) {
      s->computeMeansAndVariances(c, f);
    }
  }

}

double sNaiveBayesClassificator::getPrior(const int feature, const arr& value, const int givenClass) {
  Gaussian g;
  g.setC(means(feature, givenClass), variances(feature, givenClass));
  //std::cout << "feature: " << feature << std::endl << "given class: " << givenClass << std::endl;
  //std::cout << "value: " << value << std::endl;

  //std::cout << g.evaluate(value) << std::endl << variances(feature, givenClass) << std::endl << "----" << std::endl;
  return g.evaluate(value);
}

void sNaiveBayesClassificator::computeMeansAndVariances(const int givenClass, const int feature) {
  MT::Array<arr> featuresInClass;
  for (uint8_t d = 0; d < features.d0; ++d) {
    if (classes(d) == givenClass) {
      featuresInClass.append(features(d, feature));
    }
  }

  MT::Array<double> mean;
  mean.resizeAs(featuresInClass(0)); 
  for (uint i=0; i<featuresInClass.d0; ++i) mean += featuresInClass(i); 
  mean = mean * (1./featuresInClass.N);
  means(feature, givenClass) = mean; 

  // This is kind of a hack. Theoretically the covariance can be zero, but 
  // that gives problems, because the covariance-matrix won't be regular.
  // Thus we would have to made a special case for that. I made my life easy 
  // by adding some small constant to the covariance diagonal. Practically this 
  // doesn't make any difference and is somehow true, since we assume that 
  // there will be some variance in every direction...
  variances(feature, givenClass) = 0.01 * eye(mean.d0, mean.d0);
  
  for (uint8_t f0 = 0; f0 < mean.d0; ++f0) {
    for (uint8_t f1 = 0; f1 < mean.d0; ++f1) {
      double cov = 0;
      for (uint16_t d = 0; d < featuresInClass.d0; ++d) {
        cov += (featuresInClass(d)(f0) - means(feature, givenClass)(f0)) *
               (featuresInClass(d)(f1) - means(feature, givenClass)(f1)); 
      }
      cov /= (featuresInClass.d0 - 1);
      variances(feature, givenClass)(f0, f1) += cov;
    }
  }
}

template<class T> T limit(T min, T max, T value) {
  if (min > value) return min;
  if (max < value) return max;
  return value;
}

void sNaiveBayesClassificator::findBestStartPoint(arr& startPoint, const int feature, const int class1, const int class2, const arr& pos1, const arr& pos2, double eps = 0.001) const {
  arr centerdir = pos2 - pos1;
  if (startPoint != pos1) startPoint = pos1;

  Gaussian g1, g2;
  g1.setC(means(feature, class1), variances(feature, class1));
  g2.setC(means(feature, class2), variances(feature, class2));

  double olddir = 0;
  double stepSize = 0.5;
  while(fabs(g1.evaluate(startPoint) - g2.evaluate(startPoint)) > eps) {
    // RProp 
    double dir = limit(-1., 1., (g1.evaluate(startPoint) - g2.evaluate(startPoint)));
    if (dir * olddir < 0) stepSize *= 0.5;
    else if (dir * olddir > 0.7) stepSize *= 1.5;
    olddir = dir;

    startPoint = startPoint + dir * stepSize * centerdir;
  }
}
 
void sNaiveBayesClassificator::rejectionSampling(MT::Array<arr>& nextSample, double& p, const int class1, const int class2) {

  MT::Array<arr> testSample;
  double eps = 0.01;
  double maxdens = 0;

  srand ( time(NULL) % 5 * time(NULL)  );
  MT::rnd.clockSeed();
  
  for (uint i = 0; i < 1000; ++i) {
    generateBlocksSample(testSample, 2);

    arr probabilities;
    getProbabilities(probabilities, testSample);

    double diff = fabs(probabilities(class1) - probabilities(class2));
    double dens1 = 1;
    double dens2 = 1;
    for (uint8_t j = 0; j < features.d1; ++j) {
      dens1 *= getPrior(j, testSample(j), class1); 
      dens2 *= getPrior(j, testSample(j), class2);
    }
    double dens = (dens1 + dens2)/2;

    if (diff < eps && dens > maxdens) {
      nextSample = testSample;
      maxdens = dens;
    }
  } 
  p = maxdens;
  JK_DEBUG(nextSample << p);  
}

void sNaiveBayesClassificator::nextSample(arr& nextSample, double& p, const int class1, const int class2, const int feature) const {
  arr oldn;
  double oldp = -2;
  
  Gaussian g1, g2;
  g1.setC(means(feature, class1), variances(feature, class1));
  g2.setC(means(feature, class2), variances(feature, class2));

  findBestStartPoint(nextSample, feature, class1, class2, means(feature,class1), means(feature,class2)); 
  
  p = g1.evaluate(nextSample);

  double stepSize = 0.2;

  while (p > oldp) {
    // cache old values 
    oldn = nextSample;
    oldp = p;

    arr grad1, grad2;
    g1.gradient(grad1, nextSample);
    grad1.append(-1);
    g2.gradient(grad2, nextSample);
    grad2.append(-1);

    // we set all x_i, i>1 to 0 and solve Ax=b 
    arr b = zeros(grad1.d0, 1);
    b.reshape(b.N);
    b(0) = g1.evaluate(nextSample);
    for (uint i = 0; i < nextSample.d0; ++i) b(0) -= grad1(i)*nextSample(i);
    b(1) = g2.evaluate(nextSample);
    for (uint i = 0; i < nextSample.d0; ++i) b(1) -= grad2(i)*nextSample(i);
    b = -b;
  
    arr G = eye(grad1.d0);
    G[0]() = grad1;
    G[1]() = grad2;
    G = ~G; // LAPACK fills matrices the other way round
    
    arr l ;
    mldivide(l, G, b);
    

    // test whether we go in positive or negative direction for gradient descent 
    arr dir = (l.sub(0, l.N-2) - nextSample);
    arr n1 = nextSample - 0.2 * dir;
    arr n2 = nextSample + 0.2 * dir;
    
    if (g1.evaluate(n1) > g1.evaluate(n2))  { nextSample = n1; }
    else { nextSample = n2; }

    // find point where N(mu, SIG)(x) = N(mu2, SIG2)(x)
    findBestStartPoint(nextSample, feature, class1, class2, nextSample, means(feature,class2)); 

    p = g1.evaluate(nextSample);

    
  }
  nextSample = oldn;
  p = oldp;
}

void NaiveBayesClassificator::nextSample(MT::Array<arr> &sample) const {
  //for (uint f = 0; f < s->features.d1; ++f) {
    double pmax = 0;
    MT::Array<arr> max;

    for (uint c1 = 0; c1 < s->numOfClasses; ++c1) {
      for (uint c2 = 0; c2 < s->numOfClasses; ++c2) {
        if (c1 == c2) continue;
        MT::Array<arr> next;
        double p;
        //s->nextSample(next, p, c1, c2, f);
        s->rejectionSampling(next, p, c1, c2);
        if(p > pmax) { pmax = p; max = next; }
      }
    }
    sample = max;
    //sample.append(max);
  //}
}
