#include "DebugLog.h"
#include "../MarlinCore.h"


#if (DEBUG_LOG == true)

DebugLog debug;

#if defined (__GNUC__)                /* GNU GCC Compiler */
  /* the version of GNU GCC must be greater than 4.x */
  typedef __builtin_va_list       __gnuc_va_list;
  typedef __gnuc_va_list          va_list;
  #define va_start(v,l)           __builtin_va_start(v,l)
  #define va_end(v)               __builtin_va_end(v)
  #define va_arg(v,l)             __builtin_va_arg(v,l)
#else
  #error "Debug only support GNU compiler for now"
#endif

static DebugLevel pc_msg_level = DEBUG_LEVEL_INFO;
static DebugLevel sc_msg_level = DEBUG_LEVEL_INFO;
static char log_buf[LOG_BUFFER_SIZE];

void DebugLog::Log(DebugLevel level, const char *fmt, ...) {
  //int len;
  va_list args;

  if (level < pc_msg_level && level < sc_msg_level)
    return;

  va_start(args, fmt);

  vsnprintf(log_buf, LOG_BUFFER_SIZE, fmt, args);

  va_end(args);

  if (level >= pc_msg_level)
    CONSOLE_OUTPUT(log_buf);

  //if (level >= sc_msg_level)
    //SendLog2Screen(level);
}

void DebugLog::SetLevel(uint8_t port, DebugLevel level) {
  Log(DEBUG_LEVEL_INFO, "Last debug level: %d\n", port? sc_msg_level : pc_msg_level);
  
  if (level > DEBUG_LEVEL_MAX)  {
    Log(DEBUG_LEVEL_ERROR, "Error level: %d\n");
    return;
  }

  if (port) {
    sc_msg_level = level;
  }else {
    pc_msg_level = level;
  }
}

DebugLevel DebugLog::GetLevel() {
  return sc_msg_level < pc_msg_level? sc_msg_level : pc_msg_level;
}



#endif // #if (DEBUG_LOG == true)