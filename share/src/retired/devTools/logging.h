#ifndef LOGGING_H__
#define LOGGING_H__

#include <iostream>
#include <sstream>

//==============================================================================
/**
 * @defgroup grpLogging Simple logging facility
 *
 * It works pretty much like every other logging facility.
 * To start logging call SET_LOG and use DEBUG, INFO, WARN and ERROR.
 * @{ */

//==============================================================================
// Convenient logging macros

/// Create a logger and set the log level (see LogLevel).
#define SET_LOG(name, logLevel) \
  struct _logger_with_name_ ## name {}; \
  template<>\
  struct Logger<_logger_with_name_ ## name> {\
    static const LogLevel level = logLevel;\
  };

/// Write the content of a Varibale (textual representation) into the log.
#define DEBUG_VAR(name, var) \
  if(Logger<_logger_with_name_ ## name>::level >= DEBUG){\
    std::stringstream _intern_logging_msg; \
    _intern_logging_msg << #var << " = " << var;\
    _if<Logger<_logger_with_name_ ## name>::level >= DEBUG>::result.print(#name,"DEBUG", DEBUG, __FILE__, __LINE__, _intern_logging_msg.str().c_str());\
  }

/// log message with log level DEBUG.
#define DEBUG(name, msg) \
  _if<Logger<_logger_with_name_ ## name>::level >= DEBUG>::result.print(#name, "DEBUG", DEBUG, __FILE__, __LINE__, msg);

/// log message with log level INFO.
#define INFO(name, msg) \
  _if<Logger<_logger_with_name_ ## name>::level >= INFO>::result.print(#name, "INFO ", INFO, __FILE__, __LINE__, msg);

/// log message with log level WARN.
#define WARN(name, msg) \
  _if<Logger<_logger_with_name_ ## name>::level >= WARN>::result.print(#name, "WARN ", WARN, __FILE__, __LINE__, msg);

/// log message with log level ERROR.
#define ERROR(name, msg) \
  _if<Logger<_logger_with_name_ ## name>::level >= ERROR>::result.print(#name, "ERROR", ERROR, __FILE__, __LINE__, msg);

//==============================================================================
enum LogLevel {
  ERROR = 0,
  WARN = 1,
  INFO = 2,
  DEBUG = 3
};

//==============================================================================
// template magic to realize the logging system
/// template magic to realize the logging system
template<class T>
struct Logger;

/// template magic to realize the logging system
struct OutputReal {
  inline void print(const char *name, const char *level_str, const LogLevel level, const char *file, int line, const char *msg) { (level != ERROR ? std::cout : std::cerr ) << "[@" << file << ":" << line << " | " << name << " | " << level_str << " | "  << msg << " ]" <<  std::endl; };
};
/// template magic to realize the logging system
struct OutputEmpty {
  inline void print(const char *name, const char *level_str, const LogLevel level, const char *file, int line, const char *msg) { };
};

/// template magic to realize the logging system
template <bool Condition>
struct _if;

/// template magic to realize the logging system
template<>
struct _if<true> {
  static OutputReal result;  
};

/// template magic to realize the logging system
template<>
struct _if<false> {
  static OutputEmpty result;  
};

/**  @} */

#endif
