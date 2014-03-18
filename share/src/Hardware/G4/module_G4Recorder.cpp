#include "module_G4Recorder.h"

REGISTER_MODULE(G4Recorder)

G4Recorder::G4Recorder(): Module("G4Recorder") {
}

void G4Recorder::open() {
  String fname = STRING("z." << poses.name << '.' << MT::getNowString() << ".dat");
  MT::open(datafile, fname);
  MT::open(tstampfile, STRING(fname << ".times"));
}

void G4Recorder::close() {
  datafile.close();
  tstampfile.close();
}

void G4Recorder::step() {
  uint rev = poses.readAccess();
  floatA data = poses();
  double tstamp = poses.tstamp();
  poses.deAccess();

  uint tstampsec = tstamp;
  uint tstampusec = (tstamp-tstampsec)*1e6;
  char tag[31];
  sprintf(tag, "%6i %13d.%06d", rev, tstampsec, tstampusec);
  datafile << data << endl;
  tstampfile << tag << endl;
}
