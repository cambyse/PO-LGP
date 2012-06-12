#ifndef LOGGING_H__
#define LOGGING_H__

#include <iostream>
#include <sstream>
#include <ctime>

#define SET_LOG(name, logLevel) \
  struct _logger_with_name_ ## name {}; \
  template<>\
  struct Logger<_logger_with_name_ ## name> {\
    static const LogLevel level = logLevel;\
  };

#define DEBUG_VAR(name, var) \
  if(Logger<_logger_with_name_ ## name>::level >= DEBUG){\
    std::stringstream _intern_logging_msg; \
    _intern_logging_msg << #var << " = " << var;\
    _if<Logger<_logger_with_name_ ## name>::level >= DEBUG>::result.print(#name,"DEBUG", DEBUG, __FILE__, __LINE__, _intern_logging_msg.str().c_str());\
  }

#define DEBUG(name, msg) \
  _if<Logger<_logger_with_name_ ## name>::level >= DEBUG>::result.print(#name, "DEBUG", DEBUG, __FILE__, __LINE__, msg);

#define INFO(name, msg) \
  _if<Logger<_logger_with_name_ ## name>::level >= INFO>::result.print(#name, "INFO ", INFO, __FILE__, __LINE__, msg);

#define WARN(name, msg) \
  _if<Logger<_logger_with_name_ ## name>::level >= WARN>::result.print(#name, "WARN ", WARN, __FILE__, __LINE__, msg);

#define ERROR(name, msg) \
  _if<Logger<_logger_with_name_ ## name>::level >= ERROR>::result.print(#name, "ERROR", ERROR, __FILE__, __LINE__, msg);

enum LogLevel {
  ERROR = 0,
  WARN = 1,
  INFO = 2,
  DEBUG = 3
};

template<class T>
struct Logger;

struct OutputReal {
  inline void print(const char *name, const char *level_str, const LogLevel level, const char *file, int line, const char *msg) {
    struct tm tm;
    char current_time[32];
    char current_date[32];
    time_t tt = time(NULL);
    tm = *localtime(&tt);
    strftime(current_time, 31, "%H:%M:%S", &tm);
    strftime(current_date, 31, "%Y-%m-%d", &tm);
#ifndef LOG_STRING
    (level != ERROR ? std::cout : std::cerr ) << "[@" << file << ":" << line << " | " << name << " | " << current_date << " " << current_time << " | " << level_str << " | "  << msg << " ]" <<  std::endl; };
#else
    LOG_STRING;
    };
#endif
};
struct OutputEmpty {
  inline void print(const char *name, const char *level_str, const LogLevel level, const char *file, int line, const char *msg) { };
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
