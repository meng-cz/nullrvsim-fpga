#ifndef PAGE_MMAP_V2_H
#define PAGE_MMAP_V2_H


#include "common.h"
#include "hostfile.h"

class PhysPageAllocatorV2;

#define PTE_COW     (1<<8)
#define PTE_NALLOC  (1<<9)

typedef uint64_t PTET;

typedef struct {
    PhysAddrT   base;
    uint64_t    dwords;
    RawDataT    value;
    vector<RawDataT> multivalue;
} TgtMemSet64;
typedef vector<TgtMemSet64> TgtMemSetList;

typedef struct {
    PageIndexT  src;
    PageIndexT  dst;
} TgtPgCpy;

class SV48PageTable {

public:

    SV48PageTable(PhysPageAllocatorV2 * ppman, TgtMemSetList *stlist);
    ~SV48PageTable();
    
    PTET pt_get(VPageIndexT vpg, PhysAddrT *tgtaddr);
    
    void pt_insert(VPageIndexT vpg, PTET pte, TgtMemSetList *stlist);
    void pt_update(VPageIndexT vpg, PTET pte, TgtMemSetList *stlist);
    void pt_erase(VPageIndexT vpg, TgtMemSetList *stlist);

    PhysAddrT get_page_table_base() {
        return (ptroot->target_ppn * PAGE_LEN_BYTE);
    }

    typedef struct {
        PageIndexT          target_ppn;
        VPageIndexT         vpn;
        uint32_t            vldcnt;
        vector<PTET>        content;
    } PAGE0;

    typedef struct {
        PageIndexT          target_ppn;
        VPageIndexT         vpn;
        uint32_t            vldcnt;
        vector<PAGE0*>      content;
    } PAGE1;

    typedef struct {
        PageIndexT          target_ppn;
        VPageIndexT         vpn;
        uint32_t            vldcnt;
        vector<PAGE1*>      content;
    } PAGE2;

    typedef struct {
        PageIndexT          target_ppn;
        uint32_t            vldcnt;
        vector<PAGE2*>      content;
    } PAGE3;

    PAGE3 *ptroot = nullptr;

    PhysPageAllocatorV2 * ppman;

};


class PhysPageAllocatorV2 {

public:

    PhysPageAllocatorV2(PhysAddrT start, uint64_t size, TgtMemSetList *stlist);

    PageIndexT alloc(TgtMemSetList * stlist);
    void reuse(PageIndexT ppindex);
    bool is_shared(PageIndexT ppindex);
    bool free(PageIndexT ppindex, TgtMemSetList * stlist);

    // PageIndexT get_system_page_table_base() {
    //     return sys_pgtable->target_ppn;
    // }

protected:

    // SV48PageTable::PAGE3 *sys_pgtable;

    std::list<PageIndexT> valid_pages;
    std::unordered_map<PageIndexT, uint32_t> alloced_pages;

};


#endif
