// StreamCollector {{{
template<class T>
StreamCollector& StreamCollector::operator<<(const T &t) {
  stream << t;
  return *this;
}
// }}}
// Collector {{{
template<class T>
Collector& Collector::operator<<(const T &t) {
  stream << t;
  return *this;
}
// }}}
template<class T>
void split(MT::Array<MT::Array<T*> > &trainlist, MT::Array<MT::Array<T*> > &testlist, const MT::Array<T*> &list, uint nsplits) {
  uint ndata = list.N;
  CHECK(nsplits <= ndata, STRING("Not enough data (" << ndata << ") for " << nsplits << " splits."));

  arr splits(ndata);
  for(uint i = 0; i < ndata; i++)
    splits(i) = floor((double)i * nsplits / ndata);
  std::random_shuffle(splits.begin(), splits.end());

  trainlist.resize(nsplits);
  testlist.resize(nsplits);
  for(uint i = 0; i < ndata; i++)
    for(uint split = 0; split < nsplits; split++)
      if(splits(i) == split)
        testlist(splits(i)).append(list(i));
      else
        trainlist(splits(i)).append(list(i));
}
// namespace watch {{{
using std::chrono::hours;
using std::chrono::minutes;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::microseconds;
using std::chrono::nanoseconds;

namespace watch {
  template<class TIME_T = std::chrono::seconds>
  int64_t from(std::chrono::time_point<std::chrono::steady_clock> start) {
    auto end = now();
    return std::chrono::duration_cast<TIME_T>(end - start).count();
  }
}
// }}}
