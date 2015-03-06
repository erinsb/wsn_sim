#pragma once
#include "SimEnv.h"
#include <string>
#include <stdarg.h>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#ifndef LOG_ENABLE  
  #define LOG_ENABLE        (0)
#endif

extern SimEnv* pLoggerSimEnv;

#if defined(_WIN32) || defined(_WIN64)
#define LOG_COLOR(col) do{ HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); SetConsoleTextAttribute(hConsole, col); } while(0)
#define LOG_COLOR_RESET LOG_COLOR(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_INTENSITY)
#define __PRETTY_FILE__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) 
#else
#define LOG_COLOR(col)
#define LOG_COLOR_RESET
#define __PRETTY_FILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__) 
#endif

#if LOG_ENABLE
#define _LOG(str, ...) do {\
if (!LOG_ENABLE) break; \
  LOG_COLOR_RESET; \
  printf("[%s:L%d][@%d]: " str "\n", __PRETTY_FILE__, __LINE__, \
  ((pLoggerSimEnv != NULL) ? pLoggerSimEnv->getTimestamp() : 0), __VA_ARGS__); \
} while (0)

#define _ERROR(str, ...) do {\
if (!LOG_ENABLE) break; \
  LOG_COLOR(FOREGROUND_RED | FOREGROUND_INTENSITY); \
  printf("[%s:L%d][@%d][ERROR]: " str "\n", __PRETTY_FILE__, __LINE__, \
  ((pLoggerSimEnv != NULL) ? pLoggerSimEnv->getTimestamp() : 0), __VA_ARGS__); \
  system("pause"); exit(0); \
  } while (0)

#define _WARN(str, ...) do {\
  if (!LOG_ENABLE) break; \
  LOG_COLOR(FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_INTENSITY);\
  printf("[%s:L%d][@%d][WARN]: " str "\n", __PRETTY_FILE__, __LINE__,\
  ((pLoggerSimEnv != NULL)? pLoggerSimEnv->getTimestamp() : 0), __VA_ARGS__);\
  LOG_COLOR_RESET; } while(0)
#else
#define _LOG(str, ...) (void)0
#define _ERROR(str, ...) (void)0
#define _WARN(str, ...) (void)0
#endif