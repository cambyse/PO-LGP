#include <Core/util.h>
#include "fgplot.h"

using namespace std;

StreamCollector::StreamCollector(FILE *f):file(f) { }

StreamCollector::StreamCollector(const StreamCollector &sc) {
  cout << "this shouldnt happen" << endl;
}

StreamCollector::~StreamCollector() {
  fprintf(file, "%s\n", stream.str().c_str());
  fflush(file);
}

StreamCollector FGPlot::operator()() {
  return StreamCollector(f);
}

FGPlot::FGPlot():f(NULL) {
  setDefault();
}

FGPlot::~FGPlot() {
  if(f) {
    pclose(f);
    f = NULL;
  }
}

void FGPlot::setDefault() {
  lines = true;
  points = false;
  domain = true;
  dim3d = false;
  dataid = false;
  title = NULL;
  hardcopy = NULL;
  stream = .5;
  ymin_b = false;
  ymax_b = false;
}

void FGPlot::setLines(bool l) {
  lines = l;
}

void FGPlot::setPoints(bool p) {
  points = p;
}

void FGPlot::setDomain(bool d) {
  domain = d;
}

void FGPlot::setDim3D(bool d) {
  dim3d = d;
}

void FGPlot::setDataID(bool d) {
  dataid = d;
}

void FGPlot::setAutolegend(bool al) {
  autolegend = al;
}

void FGPlot::setTitle(const char *s) {
  title = s;
}

void FGPlot::setHardcopy(const char *s) {
  hardcopy = s;
}

void FGPlot::setStream(double s) {
  stream = s;
}

void FGPlot::setYMin(double min) {
  ymin_b = true;
  ymin = min;
}

void FGPlot::setYMax(double max) {
  ymax_b = true;
  ymax = max;
}

bool FGPlot::isDim3D() {
  return dim3d;
}

void FGPlot::open() {
  std::stringstream cmd;
  cmd << "feedgnuplot";
  cmd << " " << (lines? "--lines": "--nolines");
  cmd << " " << (points? "--points": "--nopoints");
  cmd << " " << (domain? "--domain": "--nodomain");
  cmd << " " << (dataid? "--dataid": "--nodataid");
  if(dim3d)
    cmd << " --3d";
  if(autolegend)
    cmd << " --autolegend";
  if(title)
    cmd << " --title '" << title << "'";
  if(hardcopy)
    cmd << " --hardcopy " << hardcopy;
  if(stream>0)
    cmd << " --stream " << stream;
  if(ymin_b)
    cmd << " --ymin " << ymin;
  if(ymax_b)
    cmd << " --ymax " << ymax;

  f = popen(cmd.str().c_str(), "w");
  CHECK(f, "popen() failed to connect to feedgnuplot.");
}

struct FGPlots::sFGPlots {
  KeyValueGraph kvg, *data;
  uint nplots;
  FGPlot *plots;

  sFGPlots();
  ~sFGPlots();
};

FGPlots::sFGPlots::sFGPlots(): nplots(0), plots(NULL) {
}

FGPlots::sFGPlots::~sFGPlots() {
  if(plots)
    delete[] plots;
  if(data)
    delete[] data;
}

FGPlots::FGPlots() {
  s = new sFGPlots();
}

FGPlots::~FGPlots() {
  delete s;
}

void FGPlots::open(const KeyValueGraph &k) {
  ItemL plot_list;
  KeyValueGraph *plot_kvg;

  String *str;
  bool *b;
  double *d;

  s->kvg = k;

  plot_list = s->kvg.getItems("plot").list();
  s->nplots = plot_list.N;
  s->plots = new FGPlot[s->nplots];
  s->data = new KeyValueGraph[s->nplots];
  for(uint i = 0; i < s->nplots; i++) {
    cout << "Opening new plot:" << endl;
    plot_kvg = plot_list(i)->getValue<KeyValueGraph>();

    // title {{{
    str = plot_kvg->getValue<String>("title");
    if(str) {
      cout << " - title: " << *str << endl;
      s->plots[i].setTitle(*str);
    }
    // }}}
    // lines {{{
    b = plot_kvg->getValue<bool>("lines");
    if(b) {
      cout << " - lines: " << *b << endl;
      s->plots[i].setLines(*b);
    }
    // }}}
    // points {{{
    b = plot_kvg->getValue<bool>("points");
    if(b) {
      cout << " - points: " << *b << endl;
      s->plots[i].setPoints(*b);
    }
    // }}}
    // domain {{{
    b = plot_kvg->getValue<bool>("domain");
    if(b) {
      cout << " - domain: " << *b << endl;
      s->plots[i].setDomain(*b);
    }
    // }}}
    // dim3d {{{
    b = plot_kvg->getValue<bool>("dim3d");
    if(b) {
      cout << " - dim3d: " << *b << endl;
      s->plots[i].setDim3D(*b);
    }
    // }}}
    // dataid {{{
    b = plot_kvg->getValue<bool>("dataid");
    if(b) {
      cout << " - dataid: " << *b << endl;
      s->plots[i].setDataID(*b);
    }
    // }}}
    // autolegend {{{
    b = plot_kvg->getValue<bool>("autolegend");
    if(b) {
      cout << " - autolegend: " << *b << endl;
      s->plots[i].setAutolegend(*b);
    }
    // }}}
    // hardcopy {{{
    str = plot_kvg->getValue<String>("hardcopy");
    if(str) {
      cout << " - hardcopy: " << *str << endl;
      s->plots[i].setHardcopy(*str);
    }
    // }}}
    // stream {{{
    d = plot_kvg->getValue<double>("stream");
    if(d) {
      cout << " - stream: " << *d << endl;
      s->plots[i].setStream(*d);
    }
    // }}}
    // ymin {{{
    d = plot_kvg->getValue<double>("ymin");
    if(d) {
      cout << " - ymin: " << *d << endl;
      s->plots[i].setYMin(*d);
    }
    // }}}
    // ymax {{{
    d = plot_kvg->getValue<double>("ymax");
    if(d) {
      cout << " - ymax: " << *d << endl;
      s->plots[i].setYMax(*d);
    }
    // }}}
    // data {{{
    s->data[i] = plot_kvg->getItems("data");
    for(auto j: s->data[i])
      cout << " - data: " << *j->getValue<String>() << endl;
    // }}}

    s->plots[i].open();
  }
}

void FGPlots::step(uint t) {
  String ss, *str;
  arr *data;
  for(uint i = 0; i < s->nplots; i++) {
    if(s->plots[i].isDim3D()) {
      for(auto j: s->data[i]) {
        str = j->getValue<String>();
        data = s->kvg[*str]->getValue<arr>();
        ss.clear() << data->operator()(t, 0) << " "
                    << data->operator()(t, 1) << " "
                    << *str << " "
                    << data->operator()(t, 2)
                    ;
        //cout << "cmd: " << ss << endl;
        s->plots[i]() << ss;
      }
    }
    else {
      ss.clear() << t;
      for(auto j: s->data[i]) {
        str = j->getValue<String>();
        data = s->kvg[*str]->getValue<arr>();
        ss << " " << *str
          << " " << data->elem(t)
          ;
      }
      //cout << "cmd: " << ss << endl;
      s->plots[i]() << ss;
    }
  }
}

