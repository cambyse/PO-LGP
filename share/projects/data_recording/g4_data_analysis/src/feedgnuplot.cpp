#include <Core/util.h>
#include "feedgnuplot.h"

using namespace std;

StreamCollector::StreamCollector(FILE *f):file(f) { }

StreamCollector::StreamCollector(const StreamCollector &sc) {
  cout << "this shouldnt happen" << endl;
}

StreamCollector::~StreamCollector() {
  fprintf(file, "%s\n", stream.str().c_str());
  fflush(file);
}

Feedgnuplot::Feedgnuplot():f(NULL) {
  setDefault();
}

Feedgnuplot::~Feedgnuplot() {
  if(f) pclose(f);
}

void Feedgnuplot::setDefault() {
  lines = true;
  points = false;
  domain = true;
  dataid = false;
  title = NULL;
  hardcopy = NULL;
  stream = .5;
}

void Feedgnuplot::setLines(bool l) {
  lines = l;
}

void Feedgnuplot::setPoints(bool p) {
  points = p;
}

void Feedgnuplot::setDomain(bool d) {
  domain = d;
}

void Feedgnuplot::setDataID(bool d) {
  dataid = d;
}

void Feedgnuplot::setAutolegend(bool al) {
  autolegend = al;
}

void Feedgnuplot::setTitle(const char *s) {
  title = s;
}

void Feedgnuplot::setHardcopy(const char *s) {
  hardcopy = s;
}

void Feedgnuplot::setStream(double s) {
  stream = s;
}

void Feedgnuplot::open() {
  std::stringstream cmd;
  cmd << "feedgnuplot";
  cmd << " " << (lines? "--lines": "--nolines");
  cmd << " " << (points? "--points": "--nopoints");
  cmd << " " << (domain? "--domain": "--nodomain");
  cmd << " " << (dataid? "--dataid": "--nodataid");
  if(autolegend)
    cmd << " --autolegend";
  if(title)
    cmd << " --title '" << title << "'";
  if(hardcopy)
    cmd << " --hardcopy " << hardcopy;
  if(stream>0)
    cmd << " --stream " << stream;

  f = popen(cmd.str().c_str(), "w");
  CHECK(f, "popen failed to connect to feedgnuplot.");
}

StreamCollector Feedgnuplot::operator()() {
  return StreamCollector(f);
}

