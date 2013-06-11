#include <Core/array.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>


int main(int argc, char** argv) {
  time_t _time = time(NULL); 
  struct tm* gm = localtime(&_time);
  char date[15];
  strftime(date, 15, "s%y%m%d-%H%M-", gm);
  std::string sessionName = date;

  for (uint i = 0; i < 10; ++i) {

    std::stringstream filename;
    filename << sessionName << i << ".dat";
    std::ofstream rawfile(filename.str().c_str());
   
    arr center3d = ARR(0., -.8) + randn(2,1) * 0.3;
    center3d.append(0.74);
    center3d.resize(3);

    std::string rel;

    rawfile << "0" << "," 
      << "green," 
      << 1 << ","
      << center3d(0) << "," 
      << center3d(1) << ","
      << center3d(2) << std::endl;

    int t = rand() % 100;
    // is on
    if (t < 50) {
       rel = "0 on 1";
       center3d = center3d + randn(3,1) * 0.02;
       center3d(2) = 0.848;
    }
    else {
       rel = "0 noton 1";
       center3d = ARR(0., -.8) + randn(2,1) * 0.3;
       center3d.resize(3);
       center3d(2) = 0.74;
    }

    rawfile << "1" << "," 
      << "green," 
      << 1 << ","
      << center3d(0) << "," 
      << center3d(1) << ","
      << center3d(2) << std::endl;
    
    rawfile.close();

    filename << ".rel";
    std::ofstream relfile(filename.str().c_str());

    relfile << rel << std::endl;

    relfile.close();
  }

}


