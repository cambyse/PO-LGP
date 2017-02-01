#include "module_G4Printer.h"

#include <Gui/opengl.h>
#include <Kin/kin.h>

REGISTER_MODULE(G4Printer)

G4Printer::G4Printer() : Thread("G4Printer"){ }

void G4Printer::open(){}

void G4Printer::close(){}

void G4Printer::step(){
  uint t = g4_poses.readAccess();
  floatA p = g4_poses();
  g4_poses.deAccess();

  if(!t) return; //no revision yet -> nothing to display

  cout << "=== REV " << t << " ===" << endl;
  for(uint i = 0; i < p.d0; i++) {
    cout << "i: " << i << endl;
    cout << " * pos: "
        << p(i, 0) << " "
        << p(i, 1) << " "
        << p(i, 2) << endl;
    cout << " * quat: "
        << p(i, 3) << " "
        << p(i, 4) << " "
        << p(i, 5) << " "
        << p(i, 6) << endl;
  }
}
