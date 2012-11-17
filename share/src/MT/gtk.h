#ifndef MT_gtk_h
#define MT_gtk_h

#include <MT/array.h>

typedef struct _GtkWidget GtkWidget;

void gtkLock(bool checkInitialized=true);
void gtkUnlock();
void gtkCheckInitialized();
void gtkEnterCallback();
void gtkLeaveCallback();

int gtkPopupMenuChoice(StringL& choices);
GtkWidget *gtkTopWindow(const char* title);

#endif
