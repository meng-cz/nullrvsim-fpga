#ifndef THREAD_PAGE_TABLE_V2_H
#define THREAD_PAGE_TABLE_V2_H

#include "pagememv2.h"


/**
 * For static elf
 * 0                                                                        MAX_MMAP_VADDR
 * | -- .text .rodata -- | -- .bss ... -- | ... heap ... --->  <---  ... mmap ... |
 * | ---------- Load from ELF ----------- | - alloc by brk ->  <- alloc by mmap - |
*/

/**
 * For dynamic elf
 * MIN_VADDR                                                            MAX_MMAP_VADDR    MAX_VADDR
 * | -- .text .rodata -- | -- .bss ... -- | ... heap ... --->  <---  ... mmap ... |   ld.so   |
 * | ---------- Load from ELF ----------- | - alloc by brk ->  <- alloc by mmap - |           |
*/

#define MIN_VADDR (0x100000000000UL)
#define MAX_MMAP_VADDR (0xb00000000000UL)
#define MAX_VADDR (0xc00000000000UL)

/**
 * RV64 SV48 4KB
 */
class ThreadPageTableV2 {

public:

    ThreadPageTableV2(PhysPageAllocatorV2 *ppman, TgtMemSetList *stlist);
    ThreadPageTableV2(ThreadPageTableV2 *parent, TgtMemSetList *stlist);
    ~ThreadPageTableV2();

    PTET pt_get(VPageIndexT vpg, PhysAddrT *tgtaddr) { return pt.pt_get(vpg, tgtaddr); }
    PageIndexT get_page_table_base() { return pt.get_page_table_base(); }

    VirtAddrT alloc_mmap_fixed(VirtAddrT addr, uint64_t size, PageFlagT flag, FileDescriptor* fd, uint64_t offset, string info, TgtMemSetList *stlist);
    VirtAddrT alloc_mmap(uint64_t size, PageFlagT flag, FileDescriptor* fd, uint64_t offset, string info, TgtMemSetList *stlist);
    void free_mmap(VirtAddrT addr, uint64_t size, TgtMemSetList *stlist);
    void msync_get_ppns(VirtAddrT addr, uint64_t size, vector<PageIndexT> *out);
    void msync_writeback(VirtAddrT addr, uint64_t size, unordered_map<PageIndexT, vector<RawDataT>> *pages);

    void mprotect(VirtAddrT addr, uint64_t size, uint32_t prot_flag, TgtMemSetList *stlist);

    void apply_cow(VirtAddrT addr, TgtMemSetList *stlist, vector<TgtPgCpy> *cplist);

    void init_brk(VirtAddrT brk) { brk_va = brk; };
    VirtAddrT alloc_brk(VirtAddrT brk, TgtMemSetList *stlist);
    // VirtAddrT alloc_dyn(uint64_t size, TgtMemSetList *stlist);

    void init_elf_seg(VirtAddrT addr, uint64_t size, PageFlagT flag, string info, uint8_t *data, uint64_t filesz, TgtMemSetList *stlist);
    VirtAddrT get_dyn_load_addr() { return dyn_brk_va; }

    // uint32_t ref_cnt = 0;

    void debug_print_pgtable();

protected:

    PhysPageAllocatorV2 *ppman;

    SV48PageTable pt;

    typedef struct {
        VPageIndexT     vpindex;
        uint64_t        vpcnt;
        string          info;
        PageFlagT       flag;
        FileDescriptor* fd;
        uint64_t        offset;
    } VirtSeg;
    vector<VirtSeg> mmap_segments;  // Ordered from high addr to low addr
    
    VirtAddrT brk_va = 0;
    VirtAddrT dyn_brk_va = MAX_MMAP_VADDR;

};



#endif
