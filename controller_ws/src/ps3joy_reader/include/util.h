#ifndef __UTIL_H__
#define __UTIL_H__

#include <stdarg.h>
#include <stdio.h>

#define PRINT_DEBUG(...)                                                       \
    {                                                                          \
        printf("[%s:%4d:%s()] ", __FILE__, __LINE__, __func__);                \
        printf(__VA_ARGS__);                                                   \
        printf("\n");                                                          \
    }

#define PRINT_ERROR(...)                                                       \
    {                                                                          \
        fprintf(stderr, "[%s:%4d:%s()] ", __FILE__, __LINE__, __func__);       \
        fprintf(stderr, __VA_ARGS__);                                          \
        fprintf(stderr, "\n");                                                 \
    }

#endif /* __UTIL_H__ */
