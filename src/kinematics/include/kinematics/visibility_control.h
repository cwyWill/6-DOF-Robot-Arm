#ifndef KINEMATICS__VISIBILITY_CONTROL_H_
#define KINEMATICS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KINEMATICS_attribute__ ((dllexport))
    #define KINEMATICS_attribute__ ((dllimport))
  #else
    #define KINEMATICS_declspec(dllexport)
    #define KINEMATICS_declspec(dllimport)
  #endif
  #ifdef KINEMATICS_BUILDING_DLL
    #define KINEMATICS_PUBLIC KINEMATICS_SERVER_EXPORT
  #else
    #define KINEMATICS_PUBLIC KINEMATICS_SERVER_IMPORT
  #endif
  #define KINEMATICS_PUBLIC_TYPE KINEMATICS_SERVER_PUBLIC
  #define KINEMATICS_LOCAL
#else
  #define KINEMATICS_EXPORT __attribute__ ((visibility("default")))
  #define KINEMATICS_IMPORT
  #if __GNUC__ >= 4
    #define KINEMATICS_PUBLIC __attribute__ ((visibility("default")))
    #define KINEMATICS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KINEMATICS_PUBLIC
    #define KINEMATICS_LOCAL
  #endif
  #define KINEMATICS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // KINEMATICS_SERVER__VISIBILITY_CONTROL_H_