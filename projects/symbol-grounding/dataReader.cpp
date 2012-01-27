#include "dataReader.h"

#include <MT/array_t.cpp>

#include <string>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <stdlib.h>

class sDataReader {
public:
  MT::Array<int> classes;
  MT::Array<MT::Array<double> > data;

  std::map<int, MT::Array<double> > objectData;
  std::map<std::string, int> classMapping;

};

int DataReader::getClass(const std::string predicate) const {
  return s->classMapping[predicate];  
}

void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}

int colorToInt(const std::string& color) {
  if (color == "red") {
    return 0;
  }
  else if (color == "orange") {
    return 1;  
  }
  else if (color == "yellow") {
    return 2;  
  }
  else if (color == "green") {
    return 3;  
  }
  else if (color == "blue") {
    return 4;  
  }
  return 5;
}

DataReader::DataReader() {
  s = new sDataReader;  
}

int DataReader::readDataFile(const std::string& dataFile, const std::string& relationalFile) {
  // read data 
  
  std::ifstream dataIn(dataFile.c_str());
  std::string item;
  std::vector<std::string> lines;
  while(std::getline(dataIn, item)) {
    lines.push_back(item);
  }
  for (uint8_t i = 0; i < lines.size(); ++i) {
    std::vector<std::string> elems;
    split(lines[i], ',', elems);
    s->objectData[atoi(elems[0].c_str())] = ARR(
        (double) colorToInt(elems[1]), 
        atof(elems[2].c_str()), 
        atof(elems[3].c_str()), 
        atof(elems[4].c_str()), 
        atof(elems[5].c_str()));
  }
  
  // read relational data
  std::ifstream relDataIn(relationalFile.c_str());
  lines.clear();
  while(std::getline(relDataIn, item)) {
    lines.push_back(item);
  }
  for (uint8_t i = 0; i < lines.size(); ++i) {

    std::vector<std::string> elems;
    split(lines[i], ' ', elems);

    if (s->classMapping.find(elems[1]) == s->classMapping.end()) {
      s->classMapping[elems[1]] = s->classMapping.size()-1;  
    }

    s->classes.append(s->classMapping[elems[1]]);

    int fst = atoi(elems[0].c_str());
    int snd = atoi(elems[2].c_str());

    // sort data
    //s->data.append(ARR(s->objectData[fst](0)));
    //s->data.append(ARR(s->objectData[fst](1)));
    s->data.append(ARR(s->objectData[fst](2), s->objectData[fst](3), s->objectData[fst](4)));

    //s->data.append(ARR(s->objectData[snd](0)));
    //s->data.append(ARR(s->objectData[snd](1)));
    s->data.append(ARR(s->objectData[snd](2), s->objectData[snd](3), s->objectData[snd](4)));
  }
  s->data.resize(s->data.N/2, 2);
  return 0;
}

MT::Array<int>& DataReader::getClasses() const {
  return s->classes; 
}

MT::Array<MT::Array<double> >& DataReader::getData() const {
  return s->data;  
}
