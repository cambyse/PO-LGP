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
 *
 * For an example see share/examples/devTools/logging/main.cpp
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
    _if<Logger<_logger_with_name_ ## name>::level >= DEBUG>::result.print_value(#name, "DEBUG", DEBUG, __FILE__, __LINE__, #var, var);
  
#define INFO_VAR(name, var) \
    _if<Logger<_logger_with_name_ ## name>::level >= INFO>::result.print_value(#name, "INFO", INFO, __FILE__, __LINE__, #var, var);

// log message with log level DEBUG.
#define DEBUG(name, ...) \
  _if<Logger<_logger_with_name_ ## name>::level >= DEBUG>::result.print(#name, "DEBUG", DEBUG, __FILE__, __LINE__, ##__VA_ARGS__);

// log message with log level INFO.
#define INFO(name, ...) \
  _if<Logger<_logger_with_name_ ## name>::level >= INFO>::result.print(#name, "INFO", INFO, __FILE__, __LINE__, ##__VA_ARGS__);

// log message with log level WARN.
#define WARN(name, ...) \
  _if<Logger<_logger_with_name_ ## name>::level >= WARN>::result.print(#name, "WARN", WARN, __FILE__, __LINE__, ##__VA_ARGS__);

// log message with log level ERROR.
#define ERROR(name, ...) \
  _if<Logger<_logger_with_name_ ## name>::level >= ERROR>::result.print(#name, "ERROR", ERROR, __FILE__, __LINE__, ##__VA_ARGS__);

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
  template<class T1, class T2, class ... Ts>
  inline void print(const char *name, const char *level_str, const LogLevel level, const char *file, int line, T1 msg, T2 msg2, Ts ... args) { 
    std::stringstream strstr;
    strstr << msg << msg2;
    print(name, level_str, level, file, line, strstr.str(), args...);
  }

  template<class T>
  inline void print(const char *name, const char *level_str, const LogLevel level, const char *file, int line, T msg) { 
    (level != ERROR ? std::cout : std::cerr ) << "[@" << file << ":" << line << " | " << name << " | " << level_str << " | "  << msg << " ]" <<  std::endl; 
  };

  template <class T>
  inline void print_value(const char *name, const char *level_str, const LogLevel level, const char *file, int line, const char* var_name, T var) { (level != ERROR ? std::cout : std::cerr ) << "[@" << file << ":" << line << " | " << name << " | " << level_str << " | "  << var_name << " = " << var << " ]" <<  std::endl;  };
};

struct OutputEmpty {
  inline void print(const char *name, const char *level_str, const LogLevel level, const char *file, int line, ...) { };

  template <class T>
  inline void print_value(const char *name, const char *level_str, const LogLevel level, const char *file, int line, T msg) { };
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

/**  @} */

#endif
