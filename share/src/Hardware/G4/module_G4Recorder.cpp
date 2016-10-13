#include "module_G4Recorder.h"

REGISTER_MODULE(G4Recorder)

G4Recorder::G4Recorder(): Thread("G4Recorder") {
}

void G4Recorder::open() {
  String fname = STRING("z." << g4_poses.name << '.' << mlr::getNowString() << ".dat");
  mlr::open(datafile, fname);
  mlr::open(tstampfile, STRING(fname << ".times"));
}

void G4Recorder::close() {
  datafile.close();
  tstampfile.close();
}

void G4Recorder::step() {
  uint rev = g4_poses.readAccess();
  floatA data = g4_poses();
  double tstamp = g4_poses.dataTime();
  g4_poses.deAccess();

  uint tstampsec = tstamp;
  uint tstampusec = (tstamp-tstampsec)*1e6;
  char tag[31];
  sprintf(tag, "%6i %13d.%06d", rev, tstampsec, tstampusec);
  datafile << data << endl;
  tstampfile << tag << endl;
}
