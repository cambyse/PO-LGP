#include "../naiveBayesClassificator.h"
#include "../DataReader.h"

#include <JK/util.h>

#include <MT/array_t.cpp>

void testNaiveBayesClassificator() {
  MT::Array<arr> traindata;
  traindata.append(ARR(0.61, 0.));
  traindata.append(ARR(0.44, 0.));
  traindata.append(ARR(0., 0.67));
  traindata.append(ARR(0., 0.76));

  MT::Array<int> classes;
  classes.resize(4);
  classes(0) = 0;
  classes(1) = 0;
  classes(2) = 1;
  classes(3) = 1;
  traindata.resize(4,1);

  NaiveBayesClassificator c;
  c.setTrainingsData(traindata, classes);

  MT::Array<arr> testdata;
  testdata.append(ARR(0., 0.7));
  testdata.reshape(1,1);
  int cl = c.classify(testdata);
  std::cout << "class of (0, 0.7): " << cl << std::endl;

  testdata(0,0) = ARR(0.5, 0.);
  cl = c.classify(testdata);
  std::cout << "class of (0.5, 0): " << cl << std::endl;
  
}

void testDataReader() {
  DataReader d; 
	d.readDataFile("s111230-1356-0.dat", "s111230-1356-0.dat.rel");
  d.readDataFile("s111230-1356-1.dat", "s111230-1356-1.dat.rel");
	d.readDataFile("s111230-1436-1.dat", "s111230-1436-1.dat.rel");
	d.readDataFile("s111230-1439-0.dat", "s111230-1439-0.dat.rel");
	d.readDataFile("s111230-1439-1.dat", "s111230-1439-1.dat.rel");
  d.readDataFile("s111230-1442-0.dat", "s111230-1442-0.dat.rel");
  d.readDataFile("s111230-1442-1.dat", "s111230-1442-1.dat.rel");
  d.readDataFile("s111230-1442-2.dat", "s111230-1442-2.dat.rel");

  std::cout << d.getData() << std::endl;
  std::cout << d.getClasses() << std::endl;
}

void testBayesOnRealData() {
    
  DataReader d;

	d.readDataFile("s111230-1356-0.dat", "s111230-1356-0.dat.rel");
  d.readDataFile("s111230-1356-1.dat", "s111230-1356-1.dat.rel");
	d.readDataFile("s111230-1436-1.dat", "s111230-1436-1.dat.rel");
	d.readDataFile("s111230-1439-0.dat", "s111230-1439-0.dat.rel");
	d.readDataFile("s111230-1439-1.dat", "s111230-1439-1.dat.rel");
  d.readDataFile("s111230-1442-0.dat", "s111230-1442-0.dat.rel");

  NaiveBayesClassificator c;
  c.setTrainingsData(d.getData(), d.getClasses());

  DataReader testdata;
  testdata.readDataFile("s111230-1442-1.dat", "s111230-1442-1.dat.rel");

  std::cout << "Classified " << testdata.getData() << " as " << c.classify(testdata.getData()) << " (Should be " << testdata.getClasses() << ")" << std::endl;

  DataReader testdata2;
  testdata2.readDataFile("s111230-1442-2.dat", "s111230-1442-2.dat.rel");

  std::cout << "Classified " << testdata2.getData() << " as " << c.classify(testdata2.getData()) << " (Should be " << testdata2.getClasses() << ")" << std::endl;
}

void testSampling() {
  MT::Array<arr> traindata;
  traindata.append(ARR(0.61, 0.));
  traindata.append(ARR(0.44, 0.));
  traindata.append(ARR(0., 0.67));
  traindata.append(ARR(0., 0.76));

  MT::Array<int> classes;
  classes.resize(4);
  classes(0) = 0;
  classes(1) = 0;
  classes(2) = 1;
  classes(3) = 1;
  traindata.resize(4,1);

  NaiveBayesClassificator c;
  c.setTrainingsData(traindata, classes);

  MT::Array<arr> test;
  c.nextSample(test);
  std::cout << test << std::endl;

  test.clear();

  DataReader d;

	d.readDataFile("s111230-1356-0.dat", "s111230-1356-0.dat.rel");
  d.readDataFile("s111230-1356-1.dat", "s111230-1356-1.dat.rel");
	d.readDataFile("s111230-1436-1.dat", "s111230-1436-1.dat.rel");
	d.readDataFile("s111230-1439-0.dat", "s111230-1439-0.dat.rel");
	d.readDataFile("s111230-1439-1.dat", "s111230-1439-1.dat.rel");
  d.readDataFile("s111230-1442-0.dat", "s111230-1442-0.dat.rel");
  d.readDataFile("s111230-1442-1.dat", "s111230-1442-1.dat.rel");
  d.readDataFile("s111230-1442-2.dat", "s111230-1442-2.dat.rel");

  NaiveBayesClassificator cl;
  cl.setTrainingsData(d.getData(), d.getClasses());

  cl.nextSample(test);
  std::cout << "test: " << test << std::endl;

}

int main(int argc, char** argv) {
  testDataReader();
  testNaiveBayesClassificator();
  testBayesOnRealData();
  testSampling();
}
