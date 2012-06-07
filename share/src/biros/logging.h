#ifndef LOGGING_H__
#define LOGGING_H__

#include <iostream>
#include <sstream>

#define SET_LOG(name, logLevel) \
  struct _intern_ ## name ## _type {}; \
  template<>\
  struct Logger<_intern_ ## name ## _type> {\
    static const LogLevel level = logLevel;\
  };

#define DEBUG_VAR(name, var) \
  if(Logger<_intern_ ## name ## _type>::level >= DEBUG){\
  std::stringstream _intern_logging_msg; \
  _intern_logging_msg << #var << " = " << var;\
  _if<Logger<_intern_ ## name ## _type>::level >= DEBUG>::result.p(#name, __FILE__, __LINE__, _intern_logging_msg.str().c_str());\
}

#define DEBUG(name, msg) \
  _if<Logger<_intern_ ## name ## _type>::level >= DEBUG>::result.p(#name, __FILE__, __LINE__, msg);

#define INFO(name, msg) \
  _if<Logger<_intern_ ## name ## _type>::level >= INFO>::result.p(#name, __FILE__, __LINE__, msg);

#define WARN(name, msg) \
  _if<Logger<_intern_ ## name ## _type>::level >= WARN>::result.p(#name, __FILE__, __LINE__, msg);

#define ERROR(name, msg) \
  _if<Logger<_intern_ ## name ## _type>::level >= ERROR>::result.p(#name, __FILE__, __LINE__, msg);

enum LogLevel {
  ERROR = 0,
  WARN = 1,
  INFO = 2,
  DEBUG = 3
};

template<class T>
struct Logger {
  static const LogLevel level = ERROR;
};

struct OutputReal {
  inline void p(const char *name, const char *file, int line, const char *msg) { std::cout << "@[" << name << "   " << file << ":" << line << "]: " << msg << std::endl; };
};
struct OutputEmpty {
  inline void p(const char *name, const char *file, int line, const char *msg) { };
};

template <bool Condition>
struct _if;

template<>
struct _if<true> {
  static OutputReal result;  
};

template<>
struct _if<false> {
  static OutputEmpty result;  
};

#endif
