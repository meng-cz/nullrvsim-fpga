
#include "simroot.h"

#include "pagememv2.h"

PageTable4K::PageTable4K(PTType type, PhysPageAllocatorV2 * ppman, TgtMemSetList *stlist) : ppman(ppman), type(type) {

    PageIndexT ppn = ppman->alloc();
    stlist->emplace_back(TgtMemSet64{.base = (ppn * PAGE_LEN_BYTE), .dwords = PAGE_LEN_BYTE/8, .value = 0});

    if(type == PTType::SV48) {
        root_sv48 = new PAGE3;
        root_sv48->target_ppn = ppn;
        root_sv48->vldcnt = 0;
        root_sv48->content.assign(PAGE_LEN_BYTE/8, nullptr);
    } else {
        root_sv39 = new PAGE2;
        root_sv39->target_ppn = ppn;
        root_sv39->vldcnt = 0;
        root_sv39->content.assign(PAGE_LEN_BYTE/8, nullptr);
    }
}

PageTable4K::PageTable4K(PageTable4K * forked_pt, vector<std::pair<VPageIndexT, VPageIndexT>> &shared_interval, TgtMemSetList *stlist) {

    ppman = forked_pt->ppman;
    type = forked_pt->type;

    PageIndexT root_ppn = ppman->alloc();
    if(type == PTType::SV48) {
        root_sv48 = new PAGE3;
        root_sv48->target_ppn = root_ppn;
        root_sv48->vldcnt = 0;
        root_sv48->content.assign(PAGE_LEN_BYTE/8, nullptr);
    } else {
        root_sv39 = new PAGE2;
        root_sv39->target_ppn = root_ppn;
        root_sv39->vldcnt = 0;
        root_sv39->content.assign(PAGE_LEN_BYTE/8, nullptr);
    }
    
    vector<RawDataT> p3_topush;
    p3_topush.assign(PAGE_LEN_BYTE/8, 0);

    for(uint64_t i2 = 0; i2 < (root_sv48?(PAGE_LEN_BYTE/8):1); i2++) {
        PAGE2 * parent_p2 = forked_pt->root_sv39, * p2 = root_sv39;
        if(root_sv48) {
            parent_p2 = forked_pt->root_sv48->content[i2];
            if(!parent_p2) continue;

            p2 = new PAGE2;
            p2->vpn = (i2 << 27);
            p2->target_ppn = ppman->alloc();
            p2->vldcnt = 0;
            p2->content.assign(PAGE_LEN_BYTE/8, nullptr);
            root_sv48->content[i2] = p2;
            root_sv48->vldcnt ++;
            p3_topush[i2] = (p2->target_ppn << 10) + PTE_V;
        }

        vector<RawDataT> p2_topush;
        p2_topush.assign(PAGE_LEN_BYTE/8, 0);

        for(uint64_t i1 = 0; i1 < PAGE_LEN_BYTE/8; i1++) {
            PAGE1 * parent_p1 = parent_p2->content[i1];
            if(!parent_p1) continue;

            PAGE1 * p1 = new PAGE1;
            p1->vpn = (i2 << 27) + (i1 << 18);
            p1->target_ppn = ppman->alloc();
            p1->vldcnt = 0;
            p1->content.assign(PAGE_LEN_BYTE/8, nullptr);
            p2->content[i1] = p1;
            p2->vldcnt++;
            p2_topush[i1] = (p1->target_ppn << 10) + PTE_V;

            vector<RawDataT> p1_topush;
            p1_topush.assign(PAGE_LEN_BYTE/8, 0);

            for(uint64_t i0 = 0; i0 < PAGE_LEN_BYTE/8; i0++) {
                PAGE0 * parent_p0 = parent_p1->content[i0];
                if(!parent_p0) continue;
    
                PAGE0 * p0 = new PAGE0;
                p0->vpn = (i2 << 27) + (i1 << 18) + (i0 << 9);
                p0->target_ppn = ppman->alloc();
                p0->vldcnt = 0;
                p0->content.assign(PAGE_LEN_BYTE/8, 0);
                p1->content[i0] = p0;
                p1->vldcnt++;
                p1_topush[i0] = (p0->target_ppn << 10) + PTE_V;
    
                for(uint64_t i = 0; i < PAGE_LEN_BYTE/8; i++) {
                    PTET &parent_pte = parent_p0->content[i];
                    if(!(parent_pte & PTE_V)) continue;

                    ppman->reuse(parent_pte >> 10);

                    // Set COW in both parent table and child table
                    if(parent_pte & PTE_W) {
                        VPageIndexT vpn = p0->vpn + i;
                        bool shared = false;
                        for(auto &vseg : shared_interval) {
                            if(vpn >= vseg.first && vpn < vseg.second) {
                                shared = true;
                                break;
                            }
                        }
                        if(!shared) {
                            parent_pte &= (~PTE_W);
                            parent_pte |= PTE_COW;
                            stlist->emplace_back(TgtMemSet64{.base = (parent_p0->target_ppn * PAGE_LEN_BYTE) + (i * 8), .dwords = 1, .value = parent_pte});
                        }
                    }
                }

                p0->content = parent_p0->content;

                stlist->emplace_back(TgtMemSet64{.base = (p0->target_ppn * PAGE_LEN_BYTE), .dwords = PAGE_LEN_BYTE/8, .value = 0, .multivalue = p0->content});
            }

            stlist->emplace_back(TgtMemSet64{.base = (p1->target_ppn * PAGE_LEN_BYTE), .dwords = PAGE_LEN_BYTE/8, .value = 0, .multivalue = p1_topush});
        }

        stlist->emplace_back(TgtMemSet64{.base = (p2->target_ppn * PAGE_LEN_BYTE), .dwords = PAGE_LEN_BYTE/8, .value = 0, .multivalue = p2_topush});
    }
    if(root_sv48) stlist->emplace_back(TgtMemSet64{.base = (root_ppn * PAGE_LEN_BYTE), .dwords = PAGE_LEN_BYTE/8, .value = 0, .multivalue = p3_topush});
}

PTET PageTable4K::pt_get(VPageIndexT vpg, PhysAddrT *tgtaddr) {
    PAGE2 * p2 = (root_sv48?(root_sv48->content[(vpg >> 27) & 0x1ffUL]):root_sv39);
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

void PageTable4K::pt_insert(VPageIndexT vpg, PTET pte, TgtMemSetList *stlist) {
    simroot_assert(pte & PTE_V);
    PAGE2 * &p2 = (root_sv48?(root_sv48->content[(vpg >> 27) & 0x1ffUL]):root_sv39);
    if(p2 == nullptr) {
        p2 = new PAGE2;
        p2->target_ppn = ppman->alloc();
        p2->vldcnt = 0;
        p2->content.assign(PAGE_LEN_BYTE/8, nullptr);
        p2->vpn = ((vpg >> 27) << 27);
        stlist->emplace_back(TgtMemSet64{
            .base = (p2->target_ppn * PAGE_LEN_BYTE),
            .dwords = PAGE_LEN_BYTE/8,
            .value = 0
        });
        stlist->emplace_back(TgtMemSet64{
            .base = (root_sv48->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 27) & 0x1ffUL) * 8,
            .dwords = 1,
            .value = (p2->target_ppn << 10) + PTE_V
        });
        root_sv48->vldcnt++;
    }
    PAGE1 * &p1 = p2->content[(vpg >> 18) & 0x1ffUL];
    if(p1 == nullptr) {
        p1 = new PAGE1;
        p1->target_ppn = ppman->alloc();
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
        p0->target_ppn = ppman->alloc();
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

void PageTable4K::pt_update(VPageIndexT vpg, PTET pte, TgtMemSetList *stlist) {
    simroot_assert(pte & PTE_V);
    PAGE2 * p2 = (root_sv48?(root_sv48->content[(vpg >> 27) & 0x1ffUL]):root_sv39);
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

void PageTable4K::pt_erase(VPageIndexT vpg, TgtMemSetList *stlist) {
    PAGE2 * p2 = (root_sv48?(root_sv48->content[(vpg >> 27) & 0x1ffUL]):root_sv39);
    PAGE1 * p1 = nullptr;
    PAGE0 * p0 = nullptr;
    if(p2) {
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
            ppman->free(p0->target_ppn);
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
                ppman->free(p1->target_ppn);
                delete p1;
                p2->content[(vpg >> 18) & 0x1ffUL] = nullptr;
                p2->vldcnt --;
                if(p2->vldcnt != 0) {
                    stlist->emplace_back(TgtMemSet64{
                        .base = (p2->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 18) & 0x1ffUL) * 8,
                        .dwords = 1,
                        .value = 0
                    });
                } else if(root_sv48) {
                    ppman->free(p2->target_ppn);
                    delete p2;
                    root_sv48->content[(vpg >> 27) & 0x1ffUL] = nullptr;
                    root_sv48->vldcnt --;
                    stlist->emplace_back(TgtMemSet64{
                        .base = (root_sv48->target_ppn * PAGE_LEN_BYTE) + ((vpg >> 27) & 0x1ffUL) * 8,
                        .dwords = 1,
                        .value = 0
                    });
                }
            }
        }
    }
}

PageTable4K::~PageTable4K() {
    if(!root_sv48 && !root_sv39) return;
    for(uint64_t i = 0; i < (root_sv48?(PAGE_LEN_BYTE/8):1); i++) {
        PAGE2 * p3 = (root_sv48?(root_sv48->content[i]):root_sv39);
        if(!p3) continue;
        for(auto p2 : p3->content) {
            if(!p2) continue;
            for(auto p1 : p2->content) {
                if(!p1) continue;
                for(auto pte : p1->content) {
                    if((pte & PTE_V) && !(pte & PTE_NALLOC)) {
                        PageIndexT ppn = (pte >> 10);
                        ppman->free(ppn);
                    }
                }
                ppman->free(p1->target_ppn);
                delete p1;
            }
            ppman->free(p2->target_ppn);
            delete p2;
        }
        ppman->free(p3->target_ppn);
        delete p3;
    }
    if(root_sv48) {
        ppman->free(root_sv48->target_ppn);
        delete root_sv48;
    }
}

PhysAddrT PageTable4K::get_page_table_base() {
    return (root_sv48?(root_sv48->target_ppn):(root_sv39->target_ppn)) << PAGE_ADDR_OFFSET;
}

void PageTable4K::debug_print_pgtable() {
    printf("Page Table Base PPN: 0x%lx\n", get_page_table_base() >> PAGE_ADDR_OFFSET);
    for(uint64_t i = 0; i < (root_sv48?(PAGE_LEN_BYTE/8):1); i++) {
        PAGE2 * p2 = (root_sv48?(root_sv48->content[i]):root_sv39);
        if(!p2) continue;
        printf("  Enter L2 Page: 0x%lx\n", p2->target_ppn);
        for(auto p1 : p2->content) {
            if(!p1) continue;
            printf("    Enter L1 Page: 0x%lx\n", p1->target_ppn);
            for(auto p0 : p1->content) {
                if(!p0) continue;
                printf("      Enter L0 Page: 0x%lx\n", p0->target_ppn);
                for(uint64_t i = 0; i < PAGE_LEN_BYTE/8; i++) {
                    PTET pte = p0->content[i];
                    if(!(pte & PTE_V)) continue;
                    VPageIndexT vpn = p0->vpn + (i);
                    PageIndexT ppn = (pte >> 10);
                    printf("        VP: 0x%lx -> PP: 0x%lx", vpn, ppn);
                    if(pte & PTE_COW) printf(", COW");
                    if(pte & PTE_NALLOC) printf(", NALLOC");
                    if(pte & PTE_R) printf(", R");
                    if(pte & PTE_W) printf(", W");
                    if(pte & PTE_X) printf(", X");
                    printf("\n");
                }
            }
        }
    }
}


PhysPageAllocatorV2::PhysPageAllocatorV2(PhysAddrT start, uint64_t size) {

    assert((start & (PAGE_LEN_BYTE - 1)) == 0);
    assert((size & (PAGE_LEN_BYTE - 1)) == 0);
    assert(start + size < (1UL << 48));
    for(PageIndexT ppn = (start >> PAGE_ADDR_OFFSET); ppn < ((start + size) >> PAGE_ADDR_OFFSET); ppn++) {
        valid_pages.push_back(ppn);
    }
}


PageIndexT PhysPageAllocatorV2::alloc() {
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

bool PhysPageAllocatorV2::free(PageIndexT ppindex) {
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





