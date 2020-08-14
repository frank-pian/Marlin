#ifndef DBG_LOG_H_
#define DBG_LOG_H_

#include <stdio.h>

#define DEBUG_LOG true

enum DebugLevel : uint8_t {
    DEBUG_LEVEL_TRACE = 0,
    DEBUG_LEVEL_VERBOSE,
    DEBUG_LEVEL_INFO,
    DEBUG_LEVEL_WARNING,
    DEBUG_LEVEL_ERROR,
    DEBUG_LEVEL_FATAL,
    DEBUG_LEVEL_MAX
};

#if (DEBUG_LOG == true)

// massage will output to this interface
#define CONSOLE_OUTPUT(log) MYSERIAL0.print(log)
// log buffer size, max length for one debug massage
#define LOG_BUFFER_SIZE 256

class DebugLog {

    public:
        void Log(DebugLevel level, const char *fmt, ...);
        void SetLevel(uint8_t port, DebugLevel level);
        DebugLevel GetLevel();

};


extern DebugLog debug;

#define DEBUG_F(...) debug.Log(DEBUG_LEVEL_FATAL, __VA_ARGS__)
#define DEBUG_E(...) debug.Log(DEBUG_LEVEL_ERROR, __VA_ARGS__)
#define DEBUG_W(...) debug.Log(DEBUG_LEVEL_WARNING, __VA_ARGS__)
#define DEBUG_I(...) debug.Log(DEBUG_LEVEL_INFO, __VA_ARGS__)
#define DEBUG_V(...) debug.Log(DEBUG_LEVEL_VERBOSE, __VA_ARGS__)
#define DEBUG_T(...) debug.Log(DEBUG_LEVEL_TRACE, __VA_ARGS__)

#define DEBUG_SET_LEVEL(p, l)        debug.SetLevel(p, l);

#endif // #ifdef DEBUG_LOG

#endif  // #ifndef SNAP_DBG_H_