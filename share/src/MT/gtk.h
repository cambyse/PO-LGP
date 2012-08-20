#ifndef MT_gtk_h
#define MT_gtk_h

#include <MT/array.h>

typedef struct _GtkWidget GtkWidget;

void gtkLock();
void gtkUnlock();
void gtkCheckInitialized(bool userHasLocked=false);
void gtkProcessEvents(bool waitForEvents=false, bool userHasLocked=false);
int gtkPopupMenuChoice(StringL& choices);
GtkWidget *gtkTopWindow(const char* title);

#endif
