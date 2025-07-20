#include "protocol_log.h"

#include <stdio.h>
#include <string.h>

#include "code.h"

#define CALCULATE_LOG_BODY_LENGTH(msg_str)        \
  (sizeof(((protocol_log_t*)0)->body.log_level) + \
   ((msg_str) ? strlen(msg_str) : 0))

void protocol_pack_log(protocol_log_t* self, uint8_t log_level,
                       const char* func_name, const char* message) {
  self->body.log_level = log_level;
  snprintf(self->body.message, MAX_MESSAGE_SIZE, "%s#%s", func_name, message);
  protocol_pack_header(&(self->header), main_code_system, sub_code_system_log,
                       (uint8_t*)(&self->body),
                       CALCULATE_LOG_BODY_LENGTH(self->body.message) + 1);
}