#include "utilAB.h"

// StreamCollector {{{
StreamCollector::StreamCollector(FILE *_file):file(_file) { }

StreamCollector::StreamCollector(const StreamCollector &sc) {
  HALT("Copy constructor of StreamCollector should never happen.");
}

StreamCollector::~StreamCollector() {
  fprintf(file, "%s\n", stream.str().c_str());
  fflush(file);
}
// }}}
// Collector {{{
Collector::Collector(std::function<void(std::string)> _f): f(_f) {}
Collector::Collector(const Collector &c): f(c.f) {}
Collector::~Collector() { f(stream.str()); };
// }}}
// ProgressBar {{{
void ProgressBar::reset(int _N) { N = _N; n = 0; };
void ProgressBar::width(uint a, uint b) { wa = a; wb = b; }

Collector ProgressBar::step(bool done) {
  String progressbar;

  if(N > 0)
    bar(progressbar, n, N, done);
  else
    slide(progressbar, n, done);
  n++;

  return Collector([this, progressbar, done](std::string collected) {
    if(prefix.N > wa) this->prefix.resize(wa, true);
    cout << "\33[2K\r" << std::setw(this->wa) << std::left << this->prefix
          << " " << progressbar
          << " " << collected << flush;
    if(done) cout << endl;
  });
}

Collector ProgressBar::stop() {
  return step(true);
}

void ProgressBar::bar(String &str, uint n, uint N, bool done) {
  uint i;

  str.clear() << "[";
  for(i = 0; i < wb * (double)n / N; i++) str << "=";
  for(; i < wb; i++) str << " ";
  str << "] " << (done?"✔":wheel[n%4]);
}

void ProgressBar::slide(String &str, uint n, bool done) {
  // uint prog = done? wb: n%wb;
  uint i;
  str.clear() << "[";
  if(done)
    for(i = 0; i < wb; i++) str << "=";
  else
    for(i = 0; i < wb; i++) str << ((i-n)%6 < 3?"=":" ");
  str << "] " << (done?"✔":wheel[n%4]);
}
// }}}
// namespace watch {{{
namespace watch {
  std::stack<std::chrono::time_point<std::chrono::steady_clock> > stack;

  std::chrono::time_point<std::chrono::steady_clock> now() {
    return std::chrono::steady_clock::now();
  }

  void push() {
    stack.push(std::chrono::steady_clock::now());
  }
}
// }}}
// namespace hash {{{
namespace hash {
  std::hash<std::string> hash;
  std::map<std::string, size_t> map;

  void clear() {
    map.erase(map.begin(), map.end());
  }

  void clear(const std::string &key) {
    map.erase(key);
  }

  void append(const std::string &key, const std::string &value) {
    std::stringstream stream;

    auto it = map.find(key);
    if(it != map.end())
      stream << it->second;
    stream << value;

    map[key] = hash(stream.str());
  }

  std::string get(const std::string &key) {
    auto it = map.find(key);
    return it != map.end()? std::to_string(it->second): std::string();
  }
}
// }}}
