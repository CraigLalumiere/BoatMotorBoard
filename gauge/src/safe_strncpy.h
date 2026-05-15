#ifndef SAFE_STRNCPY_H_
#define SAFE_STRNCPY_H_

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Guarantees the 'dest' string is null-terminated
char *safe_strncpy(char *restrict dest, const char *restrict src, size_t count);

#ifdef __cplusplus
}
#endif
#endif // SAFE_STRNCPY_H_
