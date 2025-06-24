#ifndef CC_H
#define CC_H

#include <stdio.h>
#include <stdlib.h>

#define LWIP_PLATFORM_DIAG(x)    do {printf x;} while(0)
#define LWIP_PLATFORM_ASSERT(x)  do {printf("Assert \"%s\" failed\n", x);} while(0)

#define U32_F "u"
#define S32_F "d"
#define X32_F "x"

#endif // CC_H
