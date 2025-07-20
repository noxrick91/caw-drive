#include "log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "../Comm/protocol_log.h"
#include "../Devices/dev_usart.h"

void log_write(const char* fmt, log_level_e level, const char* file, int line,
               const char* func, ...) {
  protocol_log_t protocol_log;
  char tmp[100];
  memset(tmp, 0, sizeof(tmp));
  va_list args;
  va_start(args, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);

  protocol_pack_log(&protocol_log, level, func, tmp);
  dev_usart_write_queue((const uint8_t*)(&protocol_log),
                        sizeof(protocol_header_t) + protocol_log.header.length);
}