
#include "ioctls.h"
#include "simroot.h"

#include <sys/ioctl.h>
#include <sys/termios.h>

std::pair<IoctlT, uint64_t> get_ioctl_cmd_info(uint32_t cmd) {

#define GEN_IOCTL(cmd, flag, sz) case cmd: return std::make_pair<IoctlT, uint64_t>((flag)|IOC_V, (sz));

    switch (cmd)
    {
    GEN_IOCTL(TCGETS, IOC_R, 36);
    GEN_IOCTL(TCSETS, IOC_W, 36);
    GEN_IOCTL(TCSETSF, IOC_W, 36);
    GEN_IOCTL(TCSETSW, IOC_W, 36);
    default:
        return std::make_pair<IoctlT, uint64_t>((0), (0));
    }
}


