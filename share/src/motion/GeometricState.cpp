#include "motion.h"

GeometricState::GeometricState():Variable("GeometricState"){
  MT::String ors_file = MT::getParameter<MT::String>("ors_file");
  ors.init(ors_file);
}
  