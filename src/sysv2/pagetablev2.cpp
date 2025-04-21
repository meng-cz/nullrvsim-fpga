
#include "simroot.h"

#include "pagetablev2.h"

#include <sys/mman.h>


ThreadPageTableV2::ThreadPageTableV2(PTType pt_type, PhysPageAllocatorV2 *ppman, TgtMemSetList *stlist) {
    
    this->ppman = ppman;

    pt = make_unique<PageTable4K>(pt_type, ppman,stlist);
}

ThreadPageTableV2::~ThreadPageTableV2() {
    TgtMemSetList tmp;
    while(!mmap_segments.empty()) {
        auto &back = mmap_segments.back();
        free_mmap(back.vpindex * PAGE_LEN_BYTE, back.vpcnt * PAGE_LEN_BYTE, &tmp);
    }
}

ThreadPageTableV2::ThreadPageTableV2(ThreadPageTableV2 *parent, TgtMemSetList *stlist) {
    
    this->ppman = parent->ppman;

    vector<std::pair<VPageIndexT, VPageIndexT>> shared_interval;
    for(auto &vseg : parent->mmap_segments) {
        if(vseg.flag & PGFLAG_SHARE) {
            shared_interval.emplace_back(vseg.vpindex, vseg.vpindex + vseg.vpcnt);
        }
    }

    // fork entire page table
    pt = make_unique<PageTable4K>(parent->pt.get(), shared_interval,stlist);

    // Copy mmap segments
    this->mmap_segments = parent->mmap_segments;
    for(auto &seg : this->mmap_segments) {
        if(seg.fd) {
            seg.fd->ref_cnt++;
        }
    }

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
    if((vpi << PAGE_ADDR_OFFSET) <= brk_va) {
        return 0;
    }
    
    free_mmap(addr, size, stlist);

    {
        auto iter = mmap_segments.begin();
        for(; iter != mmap_segments.end(); iter++) {
            if(iter->vpindex + iter->vpcnt <= vpi) break;
        }
        mmap_segments.emplace(iter, VirtSeg{
            .vpindex = vpi,
            .vpcnt = vpi2 - vpi,
            .info = info,
            .flag = flag,
            .fd = fd,
            .offset = offset
        });
    }

    uint32_t pte_flgs = PTE_V;
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
    VPageIndexT top = (MAX_MMAP_VADDR >> PAGE_ADDR_OFFSET);

    auto iter = mmap_segments.begin();
    for(; iter != mmap_segments.end(); iter++) {
        if(iter->vpindex + iter->vpcnt + vpcnt <= top) {
            break;
        }
        top = iter->vpindex;
    }
    VPageIndexT vpindex = top - vpcnt;
    if((vpindex << PAGE_ADDR_OFFSET) <= brk_va) {
        LOG(ERROR) << "Virt Addr Space Run out";
        simroot_assert(0);
    }

    return alloc_mmap_fixed(vpindex << PAGE_ADDR_OFFSET, size, flag, fd, offset, info, stlist);
}

void ThreadPageTableV2::free_mmap(VirtAddrT addr, uint64_t size, TgtMemSetList *stlist) {
    
    simroot_assert(!(addr % PAGE_LEN_BYTE));
    // simroot_assert(!(size % PAGE_LEN_BYTE));
    simroot_assert(size);

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    VPageIndexT vpi2 = (ALIGN(addr + size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);
    for(auto iter = mmap_segments.begin(); iter != mmap_segments.end(); ) {
        VPageIndexT i = iter->vpindex, i2 = iter->vpindex + iter->vpcnt;
        if(i >= vpi2 || i2 <= vpi) {
            iter++;
            continue;
        }
        if(vpi > i && vpi2 < i2) {
            VirtSeg tmp = *iter;
            iter->vpcnt = vpi - i;
            tmp.vpindex = vpi2; // tmp is higher addr
            tmp.vpcnt = i2 - vpi2;
            tmp.offset += ((vpi2 - vpi) * PAGE_LEN_BYTE);
            if(tmp.fd) tmp.fd->ref_cnt++;
            iter = mmap_segments.insert(iter, tmp);
            iter++;
            iter++;
            continue;
        }
        if(vpi <= i && vpi2 >= i2) {
            if(iter->fd) {
                if(iter->fd->ref_cnt) iter->fd->ref_cnt--;
                if(iter->fd->ref_cnt == 0) {
                    if(iter->fd->host_fd) close (iter->fd->host_fd);
                    delete iter->fd;
                }
            }
            iter = mmap_segments.erase(iter);
            continue;
        }
        if(vpi2 < i2) {
            iter->vpindex = vpi2;
            iter->vpcnt = i2 - vpi2;
            iter->offset += ((vpi2 - i) * PAGE_LEN_BYTE);
        }
        else if(vpi > i) {
            iter->vpcnt = vpi - i;
        }
        iter++;
        continue;
    }
    for(VPageIndexT i = vpi; i < vpi2; i++) {
        PTET pte = pt_get(i, nullptr);
        if(pte & PTE_V) {
            if(!(pte & PTE_NALLOC)) ppman->free(pte >> 10);
            pt->pt_erase(i, stlist);
        }
    }
}

void ThreadPageTableV2::msync_get_ppns(VirtAddrT addr, uint64_t size, vector<PageIndexT> *out) {

    simroot_assert(!(addr % PAGE_LEN_BYTE));
    // simroot_assert(!(size % PAGE_LEN_BYTE));
    simroot_assert(size);

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    VPageIndexT vpi2 = (ALIGN(addr + size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);
    
    for(auto iter = mmap_segments.begin(); iter != mmap_segments.end(); ) {
        if(!((iter->flag & PGFLAG_SHARE) && (iter->flag & PGFLAG_W) && (iter->fd) && (iter->fd->host_fd))) {
            iter++;
            continue;
        }
        VPageIndexT i = iter->vpindex, i2 = iter->vpindex + iter->vpcnt;
        if(i >= vpi2 || i2 <= vpi) {
            iter++;
            continue;
        }

        VPageIndexT begin = std::max<VPageIndexT>(i, vpi), end = std::min<VPageIndexT>(i2, vpi2);
        for(VPageIndexT vpn = begin; vpn < end; vpn++) {
            VPageIndexT vpn_in_file = (vpn - iter->vpindex) + (iter->offset / PAGE_LEN_BYTE);
            auto iter2 = iter->fd->file_buffers.find(vpn_in_file);
            if(iter2 != iter->fd->file_buffers.end()) {
                out->push_back(iter2->second);
            }
        }
        iter++;
    }
    
}

void ThreadPageTableV2::msync_writeback(VirtAddrT addr, uint64_t size, unordered_map<PageIndexT, vector<RawDataT>> *pages) {
    simroot_assert(!(addr % PAGE_LEN_BYTE));
    // simroot_assert(!(size % PAGE_LEN_BYTE));
    simroot_assert(size);

    VPageIndexT vpi = (addr >> PAGE_ADDR_OFFSET);
    VPageIndexT vpi2 = (ALIGN(addr + size, PAGE_LEN_BYTE) >> PAGE_ADDR_OFFSET);
    
    for(auto iter = mmap_segments.begin(); iter != mmap_segments.end(); ) {
        if(!((iter->flag & PGFLAG_SHARE) && (iter->flag & PGFLAG_W) && (iter->fd) && (iter->fd->host_fd))) {
            iter++;
            continue;
        }
        VPageIndexT i = iter->vpindex, i2 = iter->vpindex + iter->vpcnt;
        if(i >= vpi2 || i2 <= vpi) {
            iter++;
            continue;
        }

        VPageIndexT begin = std::max<VPageIndexT>(i, vpi), end = std::min<VPageIndexT>(i2, vpi2);
        for(VPageIndexT vpn = begin; vpn < end; vpn++) {
            VPageIndexT vpn_in_file = (vpn - iter->vpindex) + (iter->offset / PAGE_LEN_BYTE);
            auto iter2 = iter->fd->file_buffers.find(vpn_in_file);
            if(iter2 != iter->fd->file_buffers.end()) {
                PageIndexT ppn = iter2->second;
                auto iter3 = pages->find(ppn);
                simroot_assert(iter3 != pages->end());
                auto &d = iter3->second;
                uint64_t off = vpn_in_file * PAGE_LEN_BYTE;
                simroot_assert(off <= iter->fd->st_size);
                uint64_t sz = std::min<uint64_t>(PAGE_LEN_BYTE, iter->fd->st_size - off);
                lseek64(iter->fd->host_fd, off, SEEK_SET);
                if(write(iter->fd->host_fd, d.data(), sz) < 0) {
                    printf("Warnning: Write to host file %d failed\n", iter->fd->host_fd);
                }
            }
        }
        iter++;
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
        VirtSeg * vseg = nullptr;
        for(auto &seg : mmap_segments) {
            if(vpn >= seg.vpindex && vpn < seg.vpindex + seg.vpcnt) {
                vseg = &seg;
                break;
            }
        }
        simroot_assert(vseg);

        uint64_t pte_flgs = PTE_V;
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

    // Update page table
    for(VPageIndexT vpn = vpi; vpn < vpi + vpcnt; vpn++) {
        PTET pte = pt->pt_get(vpn, nullptr);
        if((pte & PTE_V) && !(pte & PTE_NALLOC)) {
            if(pte & PTE_COW) {
                pte &= (~rwxmask);
                pte |= (pte_flgs & (~PTE_W));
                if(!(pte_flgs & PTE_W)) {
                    pte &= (~PTE_COW);
                }
                pt->pt_update(vpn, pte, stlist);
            } else if((pte & rwxmask) != pte_flgs) {
                pte &= (~rwxmask);
                if(!(pte & PTE_W) && (pte_flgs & PTE_W) && ppman->is_shared(pte >> 10)) {
                    pte |= ((pte_flgs & (~PTE_W)) | PTE_COW);
                } else {
                    pte |= pte_flgs;
                }
                pt->pt_update(vpn, pte, stlist);
            }
        }
    }

    // Update mmap segments
    VPageIndexT vpi2 = vpi + vpcnt;

    uint32_t pgflag = 0;
    if(prot_flag & PROT_READ) pgflag |= PGFLAG_R;
    if(prot_flag & PROT_WRITE) pgflag |= PGFLAG_W;
    if(prot_flag & PROT_EXEC) pgflag |= PGFLAG_X;
    uint32_t pgflag_mask = (PGFLAG_R | PGFLAG_W | PGFLAG_X);

    for(auto iter = mmap_segments.begin(); iter != mmap_segments.end(); ) {
        VPageIndexT i = iter->vpindex, i2 = iter->vpindex + iter->vpcnt;
        if(i >= vpi2 || i2 <= vpi) {
            iter++;
            continue;
        }

        // divide at higher addr
        if(i2 > vpi2) {
            VirtSeg tmp = *iter;
            tmp.offset += (vpi2 - i) * PAGE_LEN_BYTE;
            tmp.vpindex = vpi2;
            tmp.vpindex = i2 - vpi2;
            if(tmp.fd) tmp.fd->ref_cnt++;
            iter->vpcnt = vpi2 - i;
            iter = mmap_segments.insert(iter, tmp);
            iter++; // point to original pos
            i2 = vpi2;
        }

        // divide at lower addr
        if(vpi > i) {
            VirtSeg tmp = *iter;
            tmp.offset += (vpi - i) * PAGE_LEN_BYTE;
            tmp.vpindex = vpi;
            tmp.vpcnt = i2 - vpi;
            if(tmp.fd) tmp.fd->ref_cnt++;
            iter->vpcnt = vpi - i;
            iter = mmap_segments.insert(iter, tmp);
            i = vpi;
        }

        iter->flag &= (~pgflag_mask);
        iter->flag |= pgflag;
        iter++;
    }

}

VirtAddrT ThreadPageTableV2::alloc_brk(VirtAddrT brk, TgtMemSetList *stlist) {
    VirtAddrT ret = 0;
    if(brk <= brk_va) {
        ret = brk_va;
    }
    else {
        VPageIndexT vpindex = CEIL_DIV(brk_va, PAGE_LEN_BYTE);
        uint64_t vpcnt = CEIL_DIV(brk, PAGE_LEN_BYTE) - vpindex;
        for(VPageIndexT vpn = vpindex; vpn < vpindex + vpcnt; vpn++) {
            PageIndexT ppn = ppman->alloc();
            pt->pt_insert(vpn, (ppn << 10) | (PTE_V | PTE_W | PTE_R), stlist);
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

    uint64_t pte_flgs = (PTE_V);
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


