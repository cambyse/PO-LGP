#include "biros_views.h"
#include "biros.h"

void GenericTextView_Process  ::write(std::ostream& os) { writeInfo(os, *((Process*)object), false); }
void GenericTextView_Variable ::write(std::ostream& os) { writeInfo(os, *((Variable*)object), false); }
void GenericTextView_FieldInfo::write(std::ostream& os) { writeInfo(os, *((FieldRegistration*)object), false); }
void GenericTextView_Parameter::write(std::ostream& os) { writeInfo(os, *((Parameter*)object), false); }

REGISTER_VIEW(GenericTextView_Process, Process)
REGISTER_VIEW(GenericTextView_Variable, Variable)
REGISTER_VIEW(GenericTextView_FieldInfo, FieldRegistration)
REGISTER_VIEW(GenericTextView_Parameter, Parameter)

