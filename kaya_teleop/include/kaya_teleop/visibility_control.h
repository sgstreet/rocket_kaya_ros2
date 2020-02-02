#ifndef _VISIBILITY_CONTROL_H_
#define _VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KAYA_TELEOP_EXPORT __attribute__ ((dllexport))
    #define KAYA_TELEOP_IMPORT __attribute__ ((dllimport))
  #else
    #define KAYA_TELEOP_EXPORT __declspec(dllexport)
    #define KAYA_TELEOP_IMPORT __declspec(dllimport)
  #endif
  #ifdef KAYA_TELEOP_BUILDING_LIBRARY
    #define KAYA_TELEOP_PUBLIC KAYA_TELEOP_EXPORT
  #else
    #define KAYA_TELEOP_PUBLIC KAYA_TELEOP_IMPORT
  #endif
  #define KAYA_TELEOP_PUBLIC_TYPE KAYA_TELEOP_PUBLIC
  #define KAYA_TELEOP_LOCAL
#else
  #define KAYA_TELEOP_EXPORT __attribute__ ((visibility("default")))
  #define KAYA_TELEOP_IMPORT
  #if __GNUC__ >= 4
    #define KAYA_TELEOP_PUBLIC __attribute__ ((visibility("default")))
    #define KAYA_TELEOP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KAYA_TELEOP_PUBLIC
    #define KAYA_TELEOP_LOCAL
  #endif
  #define KAYA_TELEOP_PUBLIC_TYPE
#endif

#endif  // KAYA_TELEOP__VISIBILITY_CONTROL_H_
