#ifndef HOST_FILE_H
#define HOST_FILE_H

#include "common.h"

typedef struct {
    int32_t             host_fd;
    uint32_t            ref_cnt;
    uint64_t            st_size;
    uint64_t            usr_seek;
    string              path;
    unordered_map<VPageIndexT, PageIndexT>  file_buffers;
} FileDescriptor;


#endif
