#include "motion.h"

GeometricState::GeometricState():Variable("GeometricState") {
  reg_ors();
  mlr::String ors_file = biros().getParameter<mlr::String>("ors_file", NULL);
  ors.init(ors_file);
}
