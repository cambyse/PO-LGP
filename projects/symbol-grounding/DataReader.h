#ifndef DATAREADER_H_
#define DATAREADER_H_

#include <MT/array.h>
#include <string>

class sDataReader;

class DataReader {
private:
  sDataReader* s;

public:
  DataReader();
  int readDataFile(const std::string& dataFile, const std::string& relationalFile);
  
  MT::Array<MT::Array<double> >& getData() const;
  MT::Array<int>& getClasses() const;
};

#endif
