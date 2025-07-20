#ifndef __LOG_H__
#define __LOG_H__

#include "../config.h"

typedef enum _log_level_e {
  LOG_DEBUG = 0x0,
  LOG_INFO,
  LOG_WARN,
  LOG_ERROR,
} log_level_e;

void log_write(const char* fmt, log_level_e level, const char* file, int line,
               const char* func, ...);

#ifndef LOG_LEVEL
#define LOG_LEVEL 0
#endif

#if LOG_LEVEL <= LOG_DEBUG
#define debug(fmt, ...) \
  log_write(fmt, LOG_DEBUG, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#else
#define debug(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_INFO
#define info(fmt, ...) \
  log_write(fmt, LOG_INFO, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#else
#define info(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_WARN
#define warn(fmt, ...) \
  log_write(fmt, LOG_WARN, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#else
#define warn(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL <= LOG_ERROR
#define error(fmt, ...) \
  log_write(fmt, LOG_ERROR, __FILE__, __LINE__, __FUNCTION__, ##__VA_ARGS__)
#else
#define error(fmt, ...) ((void)0)
#endif

#endif