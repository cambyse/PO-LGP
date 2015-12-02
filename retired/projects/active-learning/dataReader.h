#ifndef DATAREADER_H_
#define DATAREADER_H_

#include <Core/array.h>
#include <string>

class sDataReader;

class DataReader {
private:
  sDataReader* s;

public:
  DataReader();
  int readDataFile(const std::string& dataFile, const std::string& relationalFile);
  
  mlr::Array<mlr::Array<double> >& getData() const;
  mlr::Array<int>& getClasses() const;
  int getClass(const std::string predicate) const;
};

#endif
