#include "motion.h"

GeometricState::GeometricState():Variable("GeometricState") {
  MT::String ors_file = birosInfo.getParameter<MT::String>("ors_file", NULL);
  ors.init(ors_file);
}
