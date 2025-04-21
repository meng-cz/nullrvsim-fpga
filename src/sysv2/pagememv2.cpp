
#include "simroot.h"

#include "pagememv2.h"

SV48PageTable::SV48PageTable(PhysPageAllocatorV2 * ppman, TgtMemSetList *stlist) : ppman(ppman) {

    ptroot = new SV48PageTable::PAGE3;

    PageIndexT ppn = ppman->alloc(stlist);
    ptroot->target_ppn = ppn;
    ptroot->vldcnt = 0;
    ptroot->content.assign(PAGE_LEN_BYTE/8, nullptr);
    stlist->emplace_back(TgtMemSet64{.base = (ppn * PAGE_LEN_BYTE), .dwords = PAGE_LEN_BYTE/8, .value = 0});

}

PTET SV48PageTable::pt_get(VPageIndexT vpg, PhysAddrT *tgtaddr) {
    PAGE2 * p2 = ptroot->content[(vpg >> 27) & 0x1ffUL];
    if(p2 == nullptr) return 0;
    PAGE1 * p1 = p2->content[(vpg >> 18) & 0x1ffUL];
    if(p1 == nullptr) return 0;
    PAGE0 * p0 = p1->content[(vpg >> 9) & 0x1ffUL];
    if(p0 == nullptr) return 0;
    if(tgtaddr) {
        *tgtaddr = (p0->target_ppn * PAGE_LEN_BYTE) + (vpg & 0x1ffUL) * 8;
    }
    return p0->content[vpg & 0x1ffUL];
}

void SV48PageTable::pt_update(VPageIndexT vpg, PTET pte, TgtMemSetList *stlist) {
    simroot_assert(pte & PTE_V);
    PAGE2 * p2 = ptroot->content[(vpg >> 27) & 0x1ffUL];
    simroot_assert(p2 != nullptr);
    PAGE1 * p1 = p2->content[(vpg >> 18) & 0x1ffUL];
    simroot_assert(p1 != nullptr);
    PAGE0 * p0 = p1->content[(vpg >> 9) & 0x1ffUL];
    simroot_assert(p0 != nullptr);
    simroot_assert(p0->content[vpg & 0x1ffUL] & PTE_V);
    p0->content[vpg & 0x1ffUL] = pte;
    stlist->emplace_back(TgtMemSet64{
        .base = (p0->target_ppn * PAGE_LEN_BYTE) + (vpg & 0x1ffUL) * 8,
        .dwords = 1,
        .value = pte
    });
}

void SV48PageTable::pt_insert(VPageIndexT vpg, PTET pte, TgtMemSetList *stlist) {
    simroot_assert(pte & PTE_V);
    PAGE2 * &p2 = ptroot->content[(vpg >> 27) & 0x1ffUL];
    if(p2 == nullptr) {
        p2 = new PAGE2;
        p2->target_ppn = ppman->alloc(stlist);
        p2->vldcnt = 0;
        p2->content.assign(PAGE_LEN_BYTE/8, nullptr);
        p2->vpn = ((vpg >> 27) << 27);
        stlist->emplace_back(TgtMemSet64{
            .base = (p2->target_ppn * PAGE_LEN_BYTE),
            .dwords = PAGE_LEN_BYTE/8,
            .value = 0
        });
        stlist->emplace_back(TgtMemSet64{
            .base = (ptroot->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 27) & 0x1ffUL) * 8,
            .dwords = 1,
            .value = (p2->target_ppn << 10) + PTE_V
        });
        ptroot->vldcnt++;
    }
    PAGE1 * &p1 = p2->content[(vpg >> 18) & 0x1ffUL];
    if(p1 == nullptr) {
        p1 = new PAGE1;
        p1->target_ppn = ppman->alloc(stlist);
        p1->vldcnt = 0;
        p1->content.assign(PAGE_LEN_BYTE/8, nullptr);
        p1->vpn = ((vpg >> 18) << 18);
        stlist->emplace_back(TgtMemSet64{
            .base = (p1->target_ppn * PAGE_LEN_BYTE),
            .dwords = PAGE_LEN_BYTE/8,
            .value = 0
        });
        stlist->emplace_back(TgtMemSet64{
            .base = (p2->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 18) & 0x1ffUL) * 8,
            .dwords = 1,
            .value = (p1->target_ppn << 10) + PTE_V
        });
        p2->vldcnt++;
    }
    PAGE0 * &p0 = p1->content[(vpg >> 9) & 0x1ffUL];
    if(p0 == nullptr) {
        p0 = new PAGE0;
        p0->target_ppn = ppman->alloc(stlist);
        p0->vldcnt = 0;
        p0->content.assign(PAGE_LEN_BYTE/8, 0);
        p0->vpn = ((vpg >> 9) << 9);
        stlist->emplace_back(TgtMemSet64{
            .base = (p0->target_ppn * PAGE_LEN_BYTE),
            .dwords = PAGE_LEN_BYTE/8,
            .value = 0
        });
        stlist->emplace_back(TgtMemSet64{
            .base = (p1->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 9) & 0x1ffUL) * 8,
            .dwords = 1,
            .value = (p0->target_ppn << 10) + PTE_V
        });
        p1->vldcnt++;
    }
    PTET &item = p0->content[vpg & 0x1ffUL];
    simroot_assert(!(item & PTE_V));
    item = pte;
    stlist->emplace_back(TgtMemSet64{
        .base = (p0->target_ppn * PAGE_LEN_BYTE) + (vpg & 0x1ffUL) * 8,
        .dwords = 1,
        .value = pte
    });
    p0->vldcnt++;
}

void SV48PageTable::pt_erase(VPageIndexT vpg, TgtMemSetList *stlist) {
    PAGE2 * p2 = nullptr;
    PAGE1 * p1 = nullptr;
    PAGE0 * p0 = nullptr;
    if(p2 = ptroot->content[(vpg >> 27) & 0x1ffUL]) {
        if(p1 = p2->content[(vpg >> 18) & 0x1ffUL]) {
            p0 = p1->content[(vpg >> 9) & 0x1ffUL];
        }
    }
    if(p0 && (p0->content[vpg & 0x1ffUL] & PTE_V)) {
        p0->content[vpg & 0x1ffUL] = 0;
        p0->vldcnt --;
        if(p0->vldcnt != 0) {
            stlist->emplace_back(TgtMemSet64{
                .base = (p0->target_ppn * PAGE_LEN_BYTE) + (vpg & 0x1ffUL) * 8,
                .dwords = 1,
                .value = 0
            });
        } else {
            // Delete PO
            ppman->free(p0->target_ppn, stlist);
            delete p0;
            p1->content[(vpg >> 9) & 0x1ffUL] = nullptr;
            p1->vldcnt --;
            if(p1->vldcnt != 0) {
                stlist->emplace_back(TgtMemSet64{
                    .base = (p1->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 9) & 0x1ffUL) * 8,
                    .dwords = 1,
                    .value = 0
                });
            } else {
                // Delete P1
                ppman->free(p1->target_ppn, stlist);
                delete p1;
                p2->content[(vpg >> 18) & 0x1ffUL] = nullptr;
                p2->vldcnt --;
                if(p2->vldcnt != 0) {
                    stlist->emplace_back(TgtMemSet64{
                        .base = (p2->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 18) & 0x1ffUL) * 8,
                        .dwords = 1,
                        .value = 0
                    });
                } else {
                    ppman->free(p2->target_ppn, stlist);
                    delete p2;
                    ptroot->content[(vpg >> 27) & 0x1ffUL] = nullptr;
                    ptroot->vldcnt --;
                    stlist->emplace_back(TgtMemSet64{
                        .base = (ptroot->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 27) & 0x1ffUL) * 8,
                        .dwords = 1,
                        .value = 0
                    });
                }
            }
        }
    }
}

SV48PageTable::~SV48PageTable() {
    TgtMemSetList tmp;
    if(!ptroot) return;
    for(auto p3 : ptroot->content) {
        if(!p3) continue;
        for(auto p2 : p3->content) {
            if(!p2) continue;
            for(auto p1 : p2->content) {
                if(!p1) continue;
                for(auto pte : p1->content) {
                    if((pte & PTE_V) && !(pte & PTE_NALLOC)) {
                        PageIndexT ppn = (pte >> 10);
                        ppman->free(ppn, &tmp);
                    }
                }
                ppman->free(p1->target_ppn, &tmp);
                delete p1;
            }
            ppman->free(p2->target_ppn, &tmp);
            delete p2;
        }
        ppman->free(p3->target_ppn, &tmp);
        delete p3;
    }
    ppman->free(ptroot->target_ppn, &tmp);
    delete ptroot;
}



PhysPageAllocatorV2::PhysPageAllocatorV2(PhysAddrT start, uint64_t size, TgtMemSetList *stlist) {

    assert((start & (PAGE_LEN_BYTE - 1)) == 0);
    assert((size & (PAGE_LEN_BYTE - 1)) == 0);
    assert(start + size < (1UL << 48));
    for(PageIndexT ppn = (start >> PAGE_ADDR_OFFSET); ppn < ((start + size) >> PAGE_ADDR_OFFSET); ppn++) {
        valid_pages.push_back(ppn);
    }
}


PageIndexT PhysPageAllocatorV2::alloc(TgtMemSetList * stlist) {
    if(valid_pages.empty()) {
        simroot_assert(0);
    }
    PageIndexT ret = valid_pages.front();
    valid_pages.pop_front();
    alloced_pages.insert(std::make_pair(ret, 1));
    return ret;
}

void PhysPageAllocatorV2::reuse(PageIndexT ppindex) {
    auto res = alloced_pages.find(ppindex);
    assert(res != alloced_pages.end());
    res->second ++;
}

bool PhysPageAllocatorV2::free(PageIndexT ppindex, TgtMemSetList * stlist) {
    bool ret = true;
    auto res = alloced_pages.find(ppindex);
    assert(res != alloced_pages.end());
    if(res->second <= 1) {
        alloced_pages.erase(res);
        valid_pages.push_back(ppindex);
    }
    else {
        res->second --;
        ret = false;
    }
    return ret;
}

bool PhysPageAllocatorV2::is_shared(PageIndexT ppindex) {
    auto res = alloced_pages.find(ppindex);
    return (res != alloced_pages.end() && (res->second) > 1);
}





