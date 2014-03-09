#include <Core/array.h>
#include <Core/util.h>

#include <fstream>

int main(int argc, char ** argv) {
  MT::initCmdLine(argc, argv);

  String in;
  double tf, tt;
  in = MT::getParameter<String>("in");
  MT::getParameter(tf, "f", 0.);
  MT::getParameter(tt, "t", -1.);

  ifstream is;
  MT::open(is, in);

  uint fnum;
  char tag[30];
  double tstamp;
  arr data;
  while(is.good()) {
    is >> fnum >> tstamp >> data;

    if(tstamp >= tf) {
      if(tt > 0. && tstamp > tt)
        break;
      sprintf(tag, "%6i %13.6f", fnum, tstamp);
      cout << tag << " " << data << endl;
    }
  }

  is.close();
}

