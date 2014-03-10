#pragma once

#include <iostream>
#include <time.h>

// groovy stuff. Alternatively, implement the operators inside the declaration
template<class T>
struct DataStruct;
template<class T>
std::istream& operator>>(std::istream &is, DataStruct<T> &d);
template<class T>
std::ostream &operator<<(std::ostream &os, const DataStruct<T> &d);

template<class T>
struct DataStruct {
  private:
    T data;
    struct timespec timestamp;

  public:
    T& getData();
    struct timespec& getTimestamp();

    friend std::istream &operator>> <>(std::istream &is, DataStruct<T> &d);
    friend std::ostream &operator<< <>(std::ostream &os, const DataStruct<T> &d);
};

template<class T>
T& DataStruct<T>::getData() {
  return data;
}

template<class T>
struct timespec& DataStruct<T>::getTimestamp() {
  return timestamp;
}

template<class T>
std::istream& operator>>(std::istream &is, DataStruct<T> &d) {
  double time, time_sec, time_nsec;
  is >> time >> d.data;
  time_nsec = modf(time, &time_sec);
  d.timestamp.tv_sec = time_sec;
  d.timestamp.tv_nsec = 1000000000 * time_nsec;
  return is;
}

template<class T>
std::ostream& operator<<(std::ostream &os, const DataStruct<T> &d) {
  double time = d.timestamp.tv_sec + d.timestamp.tv_nsec / 1000000000.;
  MT::String tag;
  tag.resize(30, false);
  sprintf(tag.p, "%13.6f", time);
  return os << tag << ' ' << d.data;
}

