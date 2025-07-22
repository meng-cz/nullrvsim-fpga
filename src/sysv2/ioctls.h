#ifndef NULRMSIL_IOCTLS
#define NULRMSIL_IOCTLS

#include "common.h"

#define IOC_V       (1<<0)
#define IOC_R       (1<<1)
#define IOC_W       (1<<2)

#define IOC_NOARG   (1<<4)

typedef uint32_t IoctlT;

std::pair<IoctlT, uint64_t> get_ioctl_cmd_info(uint32_t cmd);


#endif
