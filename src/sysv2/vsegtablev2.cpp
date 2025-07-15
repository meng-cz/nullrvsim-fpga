
#include "vsegtablev2.h"

#include "simroot.h"

VirtMemSegTable::VirtMemSegTable(VPageIndexT vpstart, VPageIndexT vpend) : Start(vpstart), End(vpend) {

}

VirtMemSegTable::VirtMemSegTable(VirtMemSegTable *parent, vector<std::pair<VPageIndexT, VPageIndexT>> &shared_interval) {
    this->Start = parent->Start;
    this->End = parent->End;
    this->segs = parent->segs;

    for(auto &s: segs) {
        if(s.flag & PGFLAG_SHARE) {
            shared_interval.emplace_back(s.vpindex, s.vpindex + s.vpcnt);
        }
        if(s.fd) {
            s.fd->ref_cnt++;
        }
    }
}

VPageIndexT VirtMemSegTable::find_pos(uint64_t vpcnt) {

    VPageIndexT valid_start = Start;
    for(auto &s : segs) {
        if(s.vpindex >= valid_start + vpcnt) {
            return valid_start;
        } else {
            valid_start = s.vpindex + s.vpcnt;
        }
    }
    simroot_assertf(End >= valid_start + vpcnt, "Virtual Memory Space RUN OUT: MMAP");
    return valid_start;
}

void VirtMemSegTable::insert(VMSegInfo &seg) {

    simroot_assertf(seg.vpindex >= Start && seg.vpindex + seg.vpcnt <= End, "Virtual Memory Segment Out of Range @0x%lx, len 0x%lx", seg.vpindex, seg.vpcnt);

    for(auto &s : segs) {
        simroot_assertf(s.vpindex + s.vpcnt <= seg.vpindex || seg.vpindex + seg.vpcnt <= s.vpindex,
            "Virtual Memory Segment Overlapped @0x%lx, len 0x%lx, (with @0x%lx, len 0x%lx)",
            seg.vpindex << PAGE_ADDR_OFFSET, seg.vpcnt << PAGE_ADDR_OFFSET,
            s.vpindex << PAGE_ADDR_OFFSET, s.vpcnt << PAGE_ADDR_OFFSET
        );
    }

    auto iter = segs.begin();
    for( ; iter != segs.end(); iter++) {
        if(iter->vpindex > seg.vpindex) break;
    }
    segs.insert(iter, seg);
}

void VirtMemSegTable::erase(VPageIndexT pgstart, uint64_t pgcnt, vector<VMSegInfo> &poped) {

    if(pgcnt == 0) return;

    vector<VMSegInfo> newsegs;
    newsegs.reserve(segs.size());

    for(auto &s : segs) {
        if(s.vpindex + s.vpcnt <= pgstart || pgstart + pgcnt <= s.vpindex) {
            // not overlaped
            newsegs.push_back(s);
            continue;
        }
        else if(s.vpindex >= pgstart && s.vpindex + s.vpcnt <= pgstart + pgcnt) {
            // included
            poped.push_back(s);
            continue;
        }
        else if(pgstart > s.vpindex && pgstart + pgcnt < s.vpindex + s.vpcnt) {
            // internal overlaped
            // left & right remained, conter poped
            VMSegInfo left = s, center = s, right = s;

            left.vpcnt = pgstart - s.vpindex;
            newsegs.push_back(left);

            center.vpindex = pgstart;
            center.vpcnt = pgcnt;
            if(center.fd) {
                center.offset += ((pgstart - s.vpindex) * PAGE_LEN_BYTE);
                center.fd->ref_cnt++;
            }
            poped.push_back(center);

            right.vpindex = pgstart + pgcnt;
            right.vpcnt = (s.vpindex + s.vpcnt) - (pgstart + pgcnt);
            if(right.fd) {
                right.offset += ((right.vpcnt - s.vpindex) * PAGE_LEN_BYTE);
                right.fd->ref_cnt++;
            }
            newsegs.push_back(right);
            continue;
        }
        else if(pgstart > s.vpindex) {
            simroot_assert(pgstart < s.vpindex + s.vpcnt);

            // left remained
            VMSegInfo popseg = s;
            uint64_t shift_right = pgstart - s.vpindex;
            popseg.vpindex += shift_right;
            popseg.vpcnt = popseg.vpcnt - shift_right;
            if(popseg.fd) {
                popseg.offset += (shift_right * PAGE_LEN_BYTE);
                popseg.fd->ref_cnt++;
            }
            poped.push_back(popseg);

            s.vpcnt = pgstart - s.vpindex;
            newsegs.push_back(s);
            continue;
        } else {
            simroot_assert(pgstart + pgcnt < s.vpindex + s.vpcnt && pgstart + pgcnt > s.vpindex);

            // right remained
            VMSegInfo popseg = s;
            popseg.vpcnt = (pgstart + pgcnt) - popseg.vpindex;
            poped.push_back(popseg);

            uint64_t shift_right = (pgstart + pgcnt) - s.vpindex;
            s.vpindex += shift_right;
            s.vpcnt = s.vpcnt - shift_right;
            if(s.fd) {
                s.offset += (shift_right * PAGE_LEN_BYTE);
                s.fd->ref_cnt++;
            }
            newsegs.push_back(s);
            continue;
        }

    }

    segs.swap(newsegs);
}

VMSegInfo * VirtMemSegTable::get(VPageIndexT vpn) {
    for(auto &s : segs) {
        if(vpn >= s.vpindex && vpn < s.vpindex + s.vpcnt) {
            return &s;
        }
    }
    return nullptr;
}

void VirtMemSegTable::getrange(VPageIndexT pgstart, uint64_t pgcnt, vector<VMSegInfo*> &out) {
    for(auto &s : segs) {
        if(s.vpindex + s.vpcnt <= pgstart || pgstart + pgcnt <= s.vpindex) {
            // not overlaped
            continue;
        }
        out.push_back(&s);
    }
}
