#include <Core/array.h>
#include <Core/util.h>

#include <fstream>

int main(int argc, char ** argv) {
  MT::initCmdLine(argc, argv);

  String in;
  double tf, tt;
  MT::getParameter(in, "i");
  MT::getParameter(tf, "f");
  MT::getParameter(tt, "t");

  ifstream is;
  MT::open(is, in);

  uint fnum;
  int first_fnum = -1;
  char tag[30];
  double tstamp;
  bool first_frame = false, pad_before = false, pad_after = false;
  arr data;
  while(is.good()) {
    is >> fnum >> tstamp >> data;
    if(tstamp < tf) {
      pad_before = true;
      continue;
    }
    if(tstamp > tt) {
      pad_after = true;
      break;
    }
    if(first_fnum == -1)
      first_fnum = fnum;

    sprintf(tag, "%6i %13.6f", fnum - first_fnum + 1, tstamp);
    cout << tag << " " << data << endl;
  }
  is.close();

  if(!pad_before || !pad_after)
    return 1;
  return 0;
}

