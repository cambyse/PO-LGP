#include <Core/array.h>
#include <Core/util.h>

#include <fstream>

int main(int argc, char ** argv) {
  mlr::initCmdLine(argc, argv);

  String in, out;
  int ff, ft;
  mlr::getParameter(in, "i");
  mlr::getParameter(out, "o");
  mlr::getParameter(ff, "f");
  mlr::getParameter(ft, "t");

  ifstream datafin, tstampfin;
  mlr::open(datafin, in);
  mlr::open(tstampfin, STRING(in << ".times"));

  ofstream datafout, tstampfout;
  mlr::open(datafout, out);
  mlr::open(tstampfout, STRING(out << ".times"));

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

