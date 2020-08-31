/*
 * (C) 2018 Aptpod,Inc
 */

#ifndef __basetime__basetime__
#define __basetime__basetime__

#include <time.h>

#if defined _WIN32
#pragma message(                                                               \
  "warning : _WIN32 build is not tested. Please check all functions before release.")
#ifdef _MAKEDLL
#ifdef _MSC_VER
#define DLL_PUBLIC __declspec(dllexport)
#else
#define DLL_PUBLIC __attribute__((dllexport))
#endif
#elif _NODLL
#ifdef _MSC_VER
#define DLL_PUBLIC
#else
#define DLL_PUBLIC
#endif
#else
#ifdef _MSC_VER
#define DLL_PUBLIC __declspec(dllimport)
#else
#define DLL_PUBLIC __attribute__((dllimport))
#endif
#endif
#define DLL_LOCAL
#else
#if __GNUC__ >= 4
#define DLL_PUBLIC __attribute__((visibility("default")))
#define DLL_LOCAL __attribute__((visibility("hidden")))
#else
#define DLL_PUBLIC
#define DLL_LOCAL
#endif
#endif

#define BT_VERSION "1.1.0"

#define TIME_ADD(src_sec, src_nsec, add_sec, add_nsec, dst_sec, dst_nsec)      \
    do {                                                                       \
        (dst_sec) = (src_sec) + (add_sec);                                     \
        (dst_nsec) = (src_nsec) + (add_nsec);                                  \
        if ((dst_nsec) >= 1000000000) {                                        \
            (dst_sec)++;                                                       \
            (dst_nsec) -= 1000000000;                                          \
        }                                                                      \
    } while (0)

#define TIME_SUB(src_sec, src_nsec, sub_sec, sub_nsec, dst_sec, dst_nsec)      \
    do {                                                                       \
        (dst_sec) = (src_sec) - (sub_sec);                                     \
        if ((src_nsec) < (sub_nsec)) {                                         \
            (dst_sec)--;                                                       \
            (dst_nsec) = 1000000000 + (src_nsec) - (sub_nsec);                 \
        } else {                                                               \
            (dst_nsec) = (src_nsec) - (sub_nsec);                              \
        }                                                                      \
    } while (0)

#ifdef __cplusplus
extern "C"
{
#endif

    enum bt_err
    {
        bt_ok = 0,
        bt_failed,
    };

    DLL_PUBLIC extern const char* bt_get_version();

    /*
     * @param btfile file path. Set NULL if you use default value,
     * /var/run/basetime.
     * @param create_base 0:not create edge base time, other: create edge base
     * time.
     */
    DLL_PUBLIC extern enum bt_err bt_init(const char* btfile, int create_base);

    DLL_PUBLIC extern void bt_uninit();

    /*
     * @param sec pointer to receive output value of sec.
     * @param nsec pointer to receive output value of nsec.
     */
    DLL_PUBLIC extern enum bt_err bt_get_relative_time(unsigned int* sec,
                                                       unsigned int* nsec);

    /*
     * @param base pointer to receive output value as timespec.
     */
    DLL_PUBLIC extern enum bt_err bt_get_monotonic(struct timespec* base);

    /*
     * @param base pointer to receive output value as timespec.
     */
    DLL_PUBLIC extern enum bt_err bt_get_edge_time(struct timespec* base);

    /*
     * @param base pointer to receive output value as timespec.
     */
    DLL_PUBLIC extern enum bt_err bt_update_ntp_time(const char* ntpserver);

    /*
     * @param base pointer to receive output value as timespec. tv_sec == 0 and
     * tv_nsec == 0 means ntp time is not set.
     */
    DLL_PUBLIC extern enum bt_err bt_get_ntp_time(struct timespec* base);

    /*
     * @param base pointer to receive output value as timespec.
     */
    DLL_PUBLIC extern enum bt_err bt_update_manual_time(
      const struct timespec* current);

    /*
     * @param base pointer to set input value as timespec. tv_sec == 0 and
     * tv_nsec == 0 means manual time is not set.
     */
    DLL_PUBLIC extern enum bt_err bt_get_manual_time(struct timespec* base);

    /*
     * @param now_sec input value of current sec.
     * @param now_nsec input value of current nsec.
     * @param basetime_sec input value of basetime sec.
     * @param basetime_nsec input value of basetime nsec.
     * @param sec pointer to receive output value of sec.
     * @param nsec pointer to receive output value of nsec.
     */
    DLL_PUBLIC extern enum bt_err bt_get_relative_time_sync(
      unsigned int now_sec,
      unsigned int now_nsec,
      unsigned int basetime_sec,
      unsigned int basetime_nsec,
      unsigned int* sec,
      unsigned int* nsec);

#ifdef __cplusplus
}
#endif

#endif /* __basetime__basetime__ */
