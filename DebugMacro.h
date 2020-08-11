/*
  DebugMacro.h - Defines macros for debugging via Serial puproses. Macros depends on another macro DEBUG.
*/

#ifndef DebugMacro_h
#define DebugMacro_h

#ifdef DEBUG
#define DPRINT(...)    Serial.print(__VA_ARGS__)
#define DPRINTF(...)  Serial.printf(__VA_ARGS__)
#define DPRINTLN(...)  Serial.println(__VA_ARGS__)
#define DPRINTFF(...)   Serial.print(F(__VA_ARGS__))
#define DPRINTLNF(...) Serial.println(F(__VA_ARGS__))

#else
#define DPRINT(...)     //blank line
#define DPRINTF(...)   //blank line
#define DPRINTLN(...)   //blank line
#define DPRINTFF(...)    //blank line
#define DPRINTLNF(...)  //blank line

#endif

#endif // end of DebugMacro_h


// #define DEBUG
// #define DEBUG_OI Serial

// #ifdef DEBUG
// #define   DebugBegin(...) DEBUG_OI.begin(__VA_ARGS__)
// #define   DebugPrint(...) DEBUG_OI.print(__VA_ARGS__)
// #define   DebugPrintln(...) DEBUG_OI.println(__VA_ARGS__)
// #define   DebugPrintf(...) DEBUG_OI.printf(__VA_ARGS__)
// #define   DebugWrite(...) DEBUG_OI.write(__VA_ARGS__)
// #define   DebugFlush(...) DEBUG_OI.flush(__VA_ARGS__)
// #else
// // Define the counterparts that cause the compiler to generate no code
// #define   DebugBegin(...) (void(0))
// #define   DebugPrint(...) (void(0))
// #define   DebugPrintln(...) (void(0))
// #define   DebugPrintf(...) (void(0))
// #define   DebugWrite(...) (void(0))
// #define   DebugFlush(...) (void(0))
// #endif