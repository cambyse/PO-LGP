#include "motion.h"

GeometricState::GeometricState():Variable("GeometricState") {
  reg_ors();
  MT::String ors_file = biros().getParameter<MT::String>("ors_file", NULL);
  ors.init(ors_file);
}
