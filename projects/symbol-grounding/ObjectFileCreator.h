#ifndef OBJECTFILECREATOR_H_
#define OBJECTFILECREATOR_H_
#include <MT/process.h>

class sObjectFileCreator;
class PerceptionOutput;

class ObjectFileCreator : public Process {
  private:
    sObjectFileCreator* s;
  public:
    ObjectFileCreator();

	void open();
	void step();
	void close();

	PerceptionOutput* input;
};


#endif
