#ifndef VQW_SERVO_DRIVER_COMPONENT__VISIBILITY_CONTROL_H_
#define VQW_SERVO_DRIVER_COMPONENT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define VQW_SERVO_DRIVER_COMPONENT_EXPORT __attribute__ ((dllexport))
    #define VQW_SERVO_DRIVER_COMPONENT_IMPORT __attribute__ ((dllimport))
  #else
    #define VQW_SERVO_DRIVER_COMPONENT_EXPORT __declspec(dllexport)
    #define VQW_SERVO_DRIVER_COMPONENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef VQW_SERVO_DRIVER_COMPONENT_BUILDING_LIBRARY
    #define VQW_SERVO_DRIVER_COMPONENT_PUBLIC VQW_SERVO_DRIVER_COMPONENT_EXPORT
  #else
    #define VQW_SERVO_DRIVER_COMPONENT_PUBLIC VQW_SERVO_DRIVER_COMPONENT_IMPORT
  #endif
  #define VQW_SERVO_DRIVER_COMPONENT_PUBLIC_TYPE VQW_SERVO_DRIVER_COMPONENT_PUBLIC
  #define VQW_SERVO_DRIVER_COMPONENT_LOCAL
#else
  #define VQW_SERVO_DRIVER_COMPONENT_EXPORT __attribute__ ((visibility("default")))
  #define VQW_SERVO_DRIVER_COMPONENT_IMPORT
  #if __GNUC__ >= 4
    #define VQW_SERVO_DRIVER_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
    #define VQW_SERVO_DRIVER_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define VQW_SERVO_DRIVER_COMPONENT_PUBLIC
    #define VQW_SERVO_DRIVER_COMPONENT_LOCAL
  #endif
  #define VQW_SERVO_DRIVER_COMPONENT_PUBLIC_TYPE
#endif

#endif  // VQW_SERVO_DRIVER_COMPONENT__VISIBILITY_CONTROL_H_
