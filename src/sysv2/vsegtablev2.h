#ifndef THREAD_VSEG_TABLE_V2_H
#define THREAD_VSEG_TABLE_V2_H

#include "common.h"
#include "hostfile.h"

typedef struct {
    VPageIndexT     vpindex;
    uint64_t        vpcnt;
    PageFlagT       flag;
    string          info;
    FileDescriptor *fd;
    uint64_t        offset;
} VMSegInfo;

class VirtMemSegTable {

public:

    VirtMemSegTable(VPageIndexT vpstart, VPageIndexT vpend);
    VirtMemSegTable(VirtMemSegTable *parent, vector<std::pair<VPageIndexT, VPageIndexT>> &shared_interval);

    VPageIndexT find_pos(uint64_t vpcnt);
    void insert(VMSegInfo &seg);
    void erase(VPageIndexT pgstart, uint64_t pgcnt, vector<VMSegInfo> &poped);
    VMSegInfo * get(VPageIndexT vpn);
    void getrange(VPageIndexT pgstart, uint64_t pgcnt, vector<VMSegInfo*> &out);

protected:

    VPageIndexT Start = 0;
    VPageIndexT End = 0;

    vector<VMSegInfo> segs;

};



#endif
