#ifndef MT_gtk_h
#define MT_gtk_h

#include <MT/array.h>

void gtkCheckInitialized();
void gtkProcessEvents(bool waitForEvents=false);
int gtkPopupMenuChoice(StringL& choices);

#endif
