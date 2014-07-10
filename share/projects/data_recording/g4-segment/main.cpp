#include <Core/array.h>
#include <Core/util.h>

#include <fstream>

int main(int argc, char ** argv) {
  MT::initCmdLine(argc, argv);

  String in, out;
  int ff, ft;
  MT::getParameter(in, "i");
  MT::getParameter(out, "o");
  MT::getParameter(ff, "f");
  MT::getParameter(ft, "t");

  ifstream datafin, tstampfin;
  MT::open(datafin, in);
  MT::open(tstampfin, STRING(in << ".times"));

  ofstream datafout, tstampfout;
  MT::open(datafout, out);
  MT::open(tstampfout, STRING(out << ".times"));

  uint fnum;
  int first_fnum = -1;
  char tag[30];
  double tstamp;
  int tstampsec, tstampusec;
  bool pad_before = false, pad_after = false;
  arr data;
  while(datafin.good()) {
    datafin >> data;
    tstampfin >> fnum >> tstamp;

    if(fnum < ff) {
      pad_before = true;
      continue;
    }
    if(fnum > ft) {
      pad_after = true;
      break;
    }
    if(first_fnum == -1)
      first_fnum = fnum;

    tstampsec = tstamp;
    tstampusec = (tstamp - tstampsec) * 1e6;
    sprintf(tag, "%6i %13d.%06d", fnum - first_fnum + 1, tstampsec, tstampusec);
    datafout << data << endl;
    tstampfout << tag << endl;
  }
  datafin.close();
  tstampfin.close();
  datafout.close();
  tstampfout.close();

  if(!pad_before || !pad_after)
    return 1;
  return 0;
}

