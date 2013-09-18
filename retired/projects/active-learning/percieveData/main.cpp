#include <vision/visionModule.h>
#include <MT/robotVariables.h>

#include "../ObjectFileCreator.h"

int main(int argc, char** argv) {
  VisionModule vision;
  ObjectFileCreator fileCreator;
  PerceptionOutput percout;

	vision.output = &percout;
  fileCreator.input = &percout;

	vision.open();
  fileCreator.threadOpen();
  fileCreator.threadLoop();
}
