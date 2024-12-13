#include "safe_strncpy.h"
#include "string.h"

char *safe_strncpy(char *restrict dest, const char *restrict src, size_t count)
{
    char *result    = strncpy(dest, src, count);
    dest[count - 1] = '\0';
    return result;
}
