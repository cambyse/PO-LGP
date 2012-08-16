#ifndef MT_gtk_h
#define MT_gtk_h

#include <MT/array.h>

typedef struct _GtkWidget GtkWidget;

void gtkCheckInitialized();
void gtkProcessEvents(bool waitForEvents=false);
int gtkPopupMenuChoice(StringL& choices);
GtkWidget *gtkTopWindow(const char* title);

#endif
