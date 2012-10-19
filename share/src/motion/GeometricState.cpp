#include "motion.h"

GeometricState::GeometricState():Variable("GeometricState") {
  reg_ors();
  MT::String ors_file = birosInfo().getParameter<MT::String>("ors_file", NULL);
  ors.init(ors_file);
}
