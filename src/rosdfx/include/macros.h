#ifndef ROSLIB_MACROS_H_INCLUDED
#define ROSLIB_MACROS_H_INCLUDED

#if defined(__GNUC__)
#define ROS_DEPRECATED __attribute__((deprecated))
#define ROS_FORCE_INLINE __attribute__((always_inline))
#else
#define ROS_DEPRECATED
#define ROS_FORCE_INLINE inline
#endif

/*
  Windows import/export and gnu http://gcc.gnu.org/wiki/Visibility
  macros.
 */

#if __GNUC__ >= 4
#define ROS_HELPER_IMPORT __attribute__((visibility("default")))
#define ROS_HELPER_EXPORT __attribute__((visibility("default")))
#define ROS_HELPER_LOCAL __attribute__((visibility("hidden")))
#else
#define ROS_HELPER_IMPORT
#define ROS_HELPER_EXPORT
#define ROS_HELPER_LOCAL
#endif

#endif
