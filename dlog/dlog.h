#ifndef DLOG_H
#define DLOG_H

#define DEBUG

#ifdef DEBUG
  #define DLOG(...) Serial.println(__VA_ARGS__)
#else
  #define DLOG(...)
#endif

#endif