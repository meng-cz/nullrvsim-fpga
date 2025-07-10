
#include "simroot.h"

#include "pagetablev2.h"

#include <sys/mman.h>


ThreadPageTableV2::ThreadPageTableV2(PTType pt_type, PhysPageAllocatorV2 *ppman, TgtMemSetList *stlist) {
    
    this->ppman = ppman;

    if(pt_type == PTType::SV39) {
        MIN_MMAP_VADDR = 0x400000000UL;
        MAX_MMAP_VADDR = 0xe00000000UL;
        MAX_VADDR = 0xf00000000UL;
    } else if(pt_type == PTType::SV48) {
        MIN_MMAP_VADDR = 0x40000000000UL;
        MAX_MMAP_VADDR = 0xe0000000000UL;
        MAX_VADDR = 0xf0000000000UL;
    }

    pt = make_unique<PageTable4K>(pt_type, ppman,stlist);
    mmap_table = make_unique<VirtMemSegTable>(MIN_MMAP_VADDR >> PAGE_ADDR_OFFSET, MAX_MMAP_VADDR >> PAGE_ADDR_OFFSET);
}

ThreadPageTableV2::~ThreadPageTableV2() {
    TgtMemSetList tmp;
    free_mmap(MIN_MMAP_VADDR, MAX_MMAP_VADDR - MIN_MMAP_VADDR, &tmp);
}

ThreadPageTableV2::ThreadPageTableV2(ThreadPageTableV2 *parent, TgtMemSetList *stlist) {
    
    this->ppman = parent->ppman;

    vector<std::pair<VPageIndexT, VPageIndexT>> shared_interval;

    // copy mmap segments
    mmap_table = make_unique<VirtMemSegTable>(parent->mmap_table.get(), shared_interval);

    // fork entire page table
    pt = make_unique<PageTable4K>(parent->pt.get(), shared_interval, stlist);

    this->brk_va = parent->brk_va;
    this->dyn_brk_va = parent->dyn_brk_va;
}


VirtAddrT ThreadPageTableV2::alloc_mmap_fixed(VirtAddrT addr, uint64_t size, PageFlagT flag, FileDescriptor* fd, uint64_t offset, string info, TgtMemSetList *stlist) {

    simroot_assert(!(addr % PAGE_LEN_BYTE));
    // simroot_assert(!(size % PAGE_LEN_BYTE));
    simroot_assert(!(offset % PAGE_LEN_BYTE));
    simroot_assert(size);

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    VPageIndexT vpi2 = (ALIGN(addr + size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);
    simroot_assertf(MIN_MMAP_VADDR > brk_va, "Virtual Memory Space RUN OUT : Heap overhead");
    if(vpi * PAGE_LEN_BYTE < MIN_MMAP_VADDR || vpi2 * PAGE_LEN_BYTE >= MAX_MMAP_VADDR) {
        simroot_assertf(0, "Bad MMAP Range @0x%lx, len 0x%lx", addr, size);
    }
    
    free_mmap(addr, size, stlist);

    VMSegInfo seg;
    seg.vpindex = vpi;
    seg.vpcnt = vpi2 - vpi;
    seg.info = info;
    seg.flag = flag;
    seg.fd = fd;
    seg.offset = offset;

    mmap_table->insert(seg);
    

    uint32_t pte_flgs = PTE_LEAF_V;
    if(flag & PGFLAG_R) pte_flgs |= PTE_R;
    if(flag & PGFLAG_W) pte_flgs |= PTE_W;
    if(flag & PGFLAG_X) pte_flgs |= PTE_X;

    if(flag & PGFLAG_STACK) {
        simroot_assert(fd == nullptr);
        // Stack should be immediatly allocated
        for(VPageIndexT vpn = vpi; vpn < vpi2; vpn++) {
            PageIndexT ppn = ppman->alloc();
            pt->pt_insert(vpn, (ppn << 10) + pte_flgs, stlist);
        }
    }
    else if(fd && (flag & PGFLAG_SHARE)) {
        // Shared file mapping, mapped to global file buffers
        for(VPageIndexT vpn = vpi; vpn < vpi2; vpn++) {
            VPageIndexT vpn_in_file = (vpn - vpi) + offset / PAGE_LEN_BYTE;
            auto iter = fd->file_buffers.find(vpn_in_file);
            if(iter == fd->file_buffers.end()) {
                pt->pt_insert(vpn, PTE_COW | PTE_NALLOC | PTE_V, stlist);
            } else {
                PageIndexT ppn = iter->second;
                ppman->reuse(ppn);
                pt->pt_insert(vpn, (ppn << 10) | pte_flgs, stlist);
            }
        }
        fd->ref_cnt++;
    }
    else if(fd) {
        // Private file mapping
        for(VPageIndexT vpn = vpi; vpn < vpi2; vpn++) {
            pt->pt_insert(vpn, PTE_COW | PTE_NALLOC | PTE_V, stlist);
        }
        fd->ref_cnt++;
    } else {
        // ANON mapping
        simroot_assert(flag & PGFLAG_ANON);
        for(VPageIndexT vpn = vpi; vpn < vpi2; vpn++) {
            pt->pt_insert(vpn, PTE_COW | PTE_NALLOC | PTE_V, stlist);
        }
    }

    return addr;
}

VirtAddrT ThreadPageTableV2::alloc_mmap(uint64_t size, PageFlagT flag, FileDescriptor* fd, uint64_t offset, string info, TgtMemSetList *stlist) {
    
    // simroot_assert(!(size % PAGE_LEN_BYTE));
    simroot_assert(!(offset % PAGE_LEN_BYTE));
    simroot_assert(size);
    
    uint64_t vpcnt = (ALIGN(size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);
    VPageIndexT vpindex = mmap_table->find_pos(vpcnt);

    return alloc_mmap_fixed(vpindex << PAGE_ADDR_OFFSET, size, flag, fd, offset, info, stlist);
}

void ThreadPageTableV2::free_mmap(VirtAddrT addr, uint64_t size, TgtMemSetList *stlist) {
    
    simroot_assert(!(addr % PAGE_LEN_BYTE));
    // simroot_assert(!(size % PAGE_LEN_BYTE));
    simroot_assert(size);

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    uint64_t vpcnt = (ALIGN(size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);

    vector<VMSegInfo> poped;
    mmap_table->erase(vpi, vpcnt, poped);
    
    for(auto &s : poped) {
        // free file
        if(s.fd) {
            if(s.fd->ref_cnt > 1) {
                s.fd->ref_cnt --;
            } else {
                if(s.fd->host_fd) close(s.fd->host_fd);
                delete s.fd;
            }
        }
        // free page table
        for(VPageIndexT vpn = s.vpindex; vpn < s.vpindex + s.vpcnt; vpn++) {
            PTET pte = pt_get(vpn, nullptr);
            simroot_assert(pte & PTE_V);
            if(!(pte & PTE_NALLOC)) ppman->free(pte >> 10);
            pt->pt_erase(vpn, stlist);
        }
    }
}

void ThreadPageTableV2::msync_get_ppns(VirtAddrT addr, uint64_t size, vector<PageIndexT> *out) {

    simroot_assert(!(addr % PAGE_LEN_BYTE));
    // simroot_assert(!(size % PAGE_LEN_BYTE));
    simroot_assert(size);

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    VPageIndexT vpi2 = (ALIGN(addr + size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);

    vector<VMSegInfo*> overlaped;
    mmap_table->getrange(vpi, vpi2 - vpi, overlaped);
    for(auto &s : overlaped) {
        if(!((s->flag & PGFLAG_SHARE) && (s->flag & PGFLAG_W) && (s->fd) && (s->fd->host_fd))) {
            continue;
        }
        VPageIndexT i = s->vpindex, i2 = s->vpindex + s->vpcnt;
        VPageIndexT begin = std::max<VPageIndexT>(i, vpi), end = std::min<VPageIndexT>(i2, vpi2);
        for(VPageIndexT vpn = begin; vpn < end; vpn++) {
            VPageIndexT vpn_in_file = (vpn - s->vpindex) + (s->offset / PAGE_LEN_BYTE);
            auto iter2 = s->fd->file_buffers.find(vpn_in_file);
            if(iter2 != s->fd->file_buffers.end()) {
                out->push_back(iter2->second);
            }
        }
    }    
}

void ThreadPageTableV2::msync_writeback(VirtAddrT addr, uint64_t size, unordered_map<PageIndexT, vector<RawDataT>> *pages) {
    simroot_assert(!(addr % PAGE_LEN_BYTE));
    // simroot_assert(!(size % PAGE_LEN_BYTE));
    simroot_assert(size);

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    VPageIndexT vpi2 = (ALIGN(addr + size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);
    
    vector<VMSegInfo*> overlaped;
    mmap_table->getrange(vpi, vpi2 - vpi, overlaped);
    for(auto &s : overlaped) {
        if(!((s->flag & PGFLAG_SHARE) && (s->flag & PGFLAG_W) && (s->fd) && (s->fd->host_fd))) {
            continue;
        }
        VPageIndexT i = s->vpindex, i2 = s->vpindex + s->vpcnt;
        VPageIndexT begin = std::max<VPageIndexT>(i, vpi), end = std::min<VPageIndexT>(i2, vpi2);
        for(VPageIndexT vpn = begin; vpn < end; vpn++) {
            VPageIndexT vpn_in_file = (vpn - s->vpindex) + (s->offset / PAGE_LEN_BYTE);
            auto iter2 = s->fd->file_buffers.find(vpn_in_file);
            if(iter2 != s->fd->file_buffers.end()) {
                PageIndexT ppn = iter2->second;
                auto iter3 = pages->find(ppn);
                simroot_assert(iter3 != pages->end());
                auto &d = iter3->second;
                uint64_t off = vpn_in_file * PAGE_LEN_BYTE;
                simroot_assert(off <= s->fd->st_size);
                uint64_t sz = std::min<uint64_t>(PAGE_LEN_BYTE, s->fd->st_size - off);
                lseek64(s->fd->host_fd, off, SEEK_SET);
                if(write(s->fd->host_fd, d.data(), sz) < 0) {
                    printf("Warnning: Write to host file %d failed\n", s->fd->host_fd);
                }
            }
        }
    }
}


void ThreadPageTableV2::apply_cow(VirtAddrT addr, TgtMemSetList *stlist, vector<TgtPgCpy> *cplist) {

    PhysAddrT pte_tgt_addr = 0;
    VPageIndexT vpn = (addr >> PAGE_ADDR_OFFSET);
    PTET pte = pt_get(vpn, &pte_tgt_addr);
    simroot_assert(pte & PTE_V);
    simroot_assert(pte & PTE_COW);

    if(pte & PTE_NALLOC) {
        // This is allocated by mmap without initialization
        VMSegInfo *vseg = mmap_table->get(vpn);
        simroot_assert(vseg);

        uint64_t pte_flgs = PTE_LEAF_V;
        if(vseg->flag & PGFLAG_R) pte_flgs |= PTE_R;
        if(vseg->flag & PGFLAG_W) pte_flgs |= PTE_W;
        if(vseg->flag & PGFLAG_X) pte_flgs |= PTE_X;

        if(vseg->fd) {
            if(vseg->flag & PGFLAG_SHARE) {
                // Search global file descriptor
                VPageIndexT vpn_in_file = (vpn - vseg->vpindex) + vseg->offset / PAGE_LEN_BYTE;
                auto iter = vseg->fd->file_buffers.find(vpn_in_file);
                if(iter == vseg->fd->file_buffers.end()) {
                    PageIndexT ppn = ppman->alloc();
                    vseg->fd->file_buffers.emplace(vpn_in_file, ppn);
                    ppman->reuse(ppn);
                    pt->pt_update(vpn, (ppn << 10) | pte_flgs, stlist);
                    stlist->emplace_back(TgtMemSet64{
                        .base = (ppn * PAGE_LEN_BYTE),
                        .dwords = PAGE_LEN_BYTE/8,
                        .value = 0
                    });
                    if(vseg->fd->host_fd) { // Host_fd is 0 for shm fake file
                        auto &st = stlist->back();
                        st.multivalue.assign(PAGE_LEN_BYTE/8, 0);
                        lseek64(vseg->fd->host_fd, vpn_in_file + PAGE_LEN_BYTE, SEEK_SET);
                        if(read(vseg->fd->host_fd, st.multivalue.data(), PAGE_LEN_BYTE) < 0) {
                            printf("Warnning: Read host file %d failed\n", vseg->fd->host_fd);
                        }
                    }
                    return ;
                } else {
                    // Other thread has loaded this shared page
                    PageIndexT ppn = iter->second;
                    ppman->reuse(ppn);
                    pt->pt_update(vpn, (ppn << 10) | pte_flgs, stlist);
                    return ;
                }
            } else {
                // loaded file to local mapping
                PageIndexT ppn = ppman->alloc();
                pt->pt_update(vpn, (ppn << 10) | pte_flgs, stlist);
                stlist->emplace_back(TgtMemSet64{
                    .base = (ppn * PAGE_LEN_BYTE),
                    .dwords = PAGE_LEN_BYTE/8,
                    .value = 0
                });
                if(vseg->fd) {
                    auto &st = stlist->back();
                    st.multivalue.assign(PAGE_LEN_BYTE/8, 0);
                    lseek64(vseg->fd->host_fd, vseg->offset + PAGE_LEN_BYTE * (vpn - vseg->vpindex), SEEK_SET);
                    if(read(vseg->fd->host_fd, st.multivalue.data(), PAGE_LEN_BYTE) < 0) {
                        printf("Warnning: Read host file %d failed\n", vseg->fd->host_fd);
                    }
                }
                return ;
            }
        } else {
            // Zero-filled
            PageIndexT ppn = ppman->alloc();
            pt->pt_update(vpn, (ppn << 10) | pte_flgs, stlist);
            stlist->emplace_back(TgtMemSet64{
                .base = (ppn * PAGE_LEN_BYTE),
                .dwords = PAGE_LEN_BYTE/8,
                .value = 0
            });
            return ;
        }
    } else {
        // This page is COWed by clone
        PageIndexT ppn = (pte >> 10);
        uint32_t newpteflg = ((pte & 0xff) | PTE_W);
        if(!ppman->is_shared(ppn)) {
            PTET newpte = (((pte >> 10) << 10) | newpteflg);
            pt->pt_update(vpn, newpte, stlist);
        } else {
            PageIndexT newppn = ppman->alloc();
            ppman->free(ppn);
            PTET newpte = ((newppn << 10) | newpteflg);
            pt->pt_update(vpn, newpte, stlist);
            cplist->emplace_back(TgtPgCpy{
                .src = ppn,
                .dst = newppn
            });
        }    
    }
}

void ThreadPageTableV2::mprotect(VirtAddrT addr, uint64_t size, uint32_t prot_flag, TgtMemSetList *stlist) {
    
    simroot_assert(!(addr % PAGE_LEN_BYTE));
    simroot_assert(size);

    uint32_t pte_flgs = 0;
    if(prot_flag & PROT_READ) pte_flgs |= PTE_R;
    if(prot_flag & PROT_WRITE) pte_flgs |= PTE_W;
    if(prot_flag & PROT_EXEC) pte_flgs |= PTE_X;
    uint32_t rwxmask = (PTE_R | PTE_W | PTE_X);

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    VPageIndexT vpcnt = (ALIGN(size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);

    uint32_t pgflag = 0;
    if(prot_flag & PROT_READ) pgflag |= PGFLAG_R;
    if(prot_flag & PROT_WRITE) pgflag |= PGFLAG_W;
    if(prot_flag & PROT_EXEC) pgflag |= PGFLAG_X;
    uint32_t pgflag_mask = (PGFLAG_R | PGFLAG_W | PGFLAG_X);

    vector<VMSegInfo> poped;
    mmap_table->erase(vpi, vpcnt, poped);
    for(auto &s : poped) {
        s.flag = (pgflag | (s.flag & (~pgflag_mask)));
        mmap_table->insert(s);

        for(VPageIndexT vpn = s.vpindex; vpn < s.vpindex + s.vpcnt; vpn++) {
            PTET pte = pt->pt_get(vpn, nullptr);
            simroot_assert(pte & PTE_V);
            if(pte & PTE_NALLOC) {
                continue;
            }
            if(pte & PTE_COW) {
                pte &= (~rwxmask);
                pte |= (pte_flgs & (~PTE_W));
                pt->pt_update(vpn, pte, stlist);
            } else if((pte & rwxmask) != pte_flgs) {
                pte &= (~rwxmask);
                if(!(s.flag & PGFLAG_SHARE) && !(pte & PTE_W) && (pte_flgs & PTE_W) && ppman->is_shared(pte >> 10)) {
                    pte |= ((pte_flgs & (~PTE_W)) | PTE_COW);
                } else {
                    pte |= pte_flgs;
                }
                pt->pt_update(vpn, pte, stlist);
            }
        }
    }

}

VirtAddrT ThreadPageTableV2::alloc_brk(VirtAddrT brk, TgtMemSetList *stlist) {
    VirtAddrT ret = 0;
    if(brk <= brk_va) {
        ret = brk_va;
    }
    else {
        simroot_assertf(!(brk & 7), "Brk with unaligned value 0x%lx", brk);
        VPageIndexT vpindex = CEIL_DIV(brk_va, PAGE_LEN_BYTE);
        uint64_t vpcnt = CEIL_DIV(brk, PAGE_LEN_BYTE) - vpindex;
        for(VPageIndexT vpn = vpindex; vpn < vpindex + vpcnt; vpn++) {
            PageIndexT ppn = ppman->alloc();
            pt->pt_insert(vpn, (ppn << 10) | (PTE_LEAF_V | PTE_W | PTE_R), stlist);
            stlist->emplace_back();
            auto &st = stlist->back();
            st.base = (ppn << PAGE_ADDR_OFFSET);
            st.dwords = (PAGE_LEN_BYTE / 8);
            st.value = 0;
        }
        brk_va = brk;
        ret = brk_va;
    }
    return ret;
}

// VirtAddrT ThreadPageTableV2::alloc_dyn(uint64_t size, TgtMemSetList *stlist) {

//     simroot_assert(!(size % PAGE_LEN_BYTE));

//     VirtAddrT ret = dyn_brk_va;
//     VPageIndexT vpindex = (ALIGN(dyn_brk_va, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);
//     uint64_t vpcnt = (ALIGN(size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);
//     for(VPageIndexT vpn = vpindex; vpn < vpindex + vpcnt; vpn++) {
//         PageIndexT ppn = ppman->alloc(stlist);
//         pt->pt_insert(vpn, (ppn << 10) | (PTE_V | PTE_W | PTE_R | PTE_X), stlist);
//     }
//     dyn_brk_va += size;
//     return ret;
// }

void ThreadPageTableV2::init_elf_seg(VirtAddrT addr, uint64_t size, PageFlagT flag, string info, uint8_t *data, uint64_t filesz, TgtMemSetList *stlist) {

    // simroot_assert(!(addr % PAGE_LEN_BYTE));
    // simroot_assert(!(size % PAGE_LEN_BYTE));

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    uint64_t vpcnt = CEIL_DIV(addr + size, PAGE_LEN_BYTE) - vpi;
    uint64_t off_in_page = (addr & (PAGE_LEN_BYTE - 1));

    uint64_t pte_flgs = (PTE_LEAF_V);
    if(flag & PGFLAG_R) pte_flgs |= PTE_R;
    if(flag & PGFLAG_W) pte_flgs |= PTE_W;
    if(flag & PGFLAG_X) pte_flgs |= PTE_X;

    for(uint64_t i = 0; i < vpcnt; i++) {

        PageIndexT ppn = ppman->alloc();
        pt->pt_insert(vpi + i, (ppn << 10) + pte_flgs, stlist);

        stlist->emplace_back(TgtMemSet64{
            .base = ppn * PAGE_LEN_BYTE,
            .dwords = PAGE_LEN_BYTE/8,
            .value = 0
        });
        auto &st = stlist->back();
        st.multivalue.assign(PAGE_LEN_BYTE/8, 0);
        if(filesz + off_in_page > i * PAGE_LEN_BYTE) {
            uint64_t sz = std::min<uint64_t>(PAGE_LEN_BYTE, filesz + off_in_page - i * PAGE_LEN_BYTE);
            if(i == 0) {
                memcpy((uint8_t*)(st.multivalue.data()) + off_in_page, data, sz - off_in_page);
            } else {
                memcpy(st.multivalue.data(), data + (i * PAGE_LEN_BYTE - off_in_page), sz);
            }
        }
    }

    if(addr + size > dyn_brk_va) {
        dyn_brk_va = ALIGN(addr + size, PAGE_LEN_BYTE);
    } else {
        brk_va = std::max<VirtAddrT>(brk_va, ALIGN(addr + size, PAGE_LEN_BYTE));
    }
}


