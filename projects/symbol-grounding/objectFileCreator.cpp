#include "ObjectFileCreator.h"
/*
#include <MT/robot_variables.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <time.h>
class sObjectFileCreator {
public:
	std::string sessionName;
	int dataNumber;
};

ObjectFileCreator::ObjectFileCreator() : Process("Object File Creator") {
  
}

void ObjectFileCreator::open() {
  s = new sObjectFileCreator();

  s->dataNumber = 0;
  time_t _time = time(NULL); 
  struct tm* gm = localtime(&_time);
  char date[15];
  strftime(date, 15, "s%y%m%d-%H%M-", gm);
  s->sessionName = date;
  
  std::cout << "To save a situation pres s<CR>." << std::endl; 
}

void ObjectFileCreator::step() {
  char c;
  std::cin >> c;
  if (c == 's') {
    if (input) {
      for (unsigned int i = 0; i < input->objects.N; ++i) {
        if (input->objects(i).found) {
          std::cout << "Object " << i << ": " 
           << "Color: " << input->objects(i).hsvObjectColorString <<  " " 
           << "Type: "<< input->objects(i).shapeType << " "
           << "Position: " << input->objects(i).center3d(0) << "/" 
           << input->objects(i).center3d(1) << "/"
           << input->objects(i).center3d(2) << std::endl;
        }
      }
    
      while (c != 'n' && c != 'y') {
        std::cout << "Is this correct? [y/n]" << std::endl;
        std::cin >> c;
      }

      if (c == 'n') return;
       
      std::cout << "saving data" << std::endl;
      std::stringstream filename;
      filename << s->sessionName << s->dataNumber << ".dat";
      ofstream rawfile(filename.str().c_str());
      
      for (unsigned int i = 0; i < input->objects.N; ++i) {
        if (input->objects(i).found) {
          rawfile << i << "," 
            << input->objects(i).hsvObjectColorString <<  "," 
            << input->objects(i).shapeType << ","
            << input->objects(i).center3d(0) << "," 
            << input->objects(i).center3d(1) << ","
            << input->objects(i).center3d(2) << std::endl;
        }
      }
      
      rawfile.close();
      
      std::cout << "Edit the corresponding relational data file in an editor." << std::endl;
      char tmp;
      std::cin >> tmp;
      std::stringstream editor;

      char* syseditor = getenv("EDITOR");
      if (syseditor == NULL) {
        std::cout << "WARNING. No systenwide editor defined. Fallback on vim." <<
          "You can quit with :q. Set $EDITOR to get the editor you are used to.";
        syseditor = "vim";
      }

      editor << syseditor << " "<< filename.str() << ".rel"; 
      system( editor.str().c_str());
      s->dataNumber++;
      }
    
  }

}

void ObjectFileCreator::close() {
  delete s;
}
*/
