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
    bool        _skip;
} TgtMemSet64;
typedef vector<TgtMemSet64> TgtMemSetList;

typedef struct {
    PageIndexT  src;
    PageIndexT  dst;
} TgtPgCpy;

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

typedef enum {
    SV39 = 0,
    SV48,
} PTType;

class PageTable4K {

public:
    PageTable4K(PTType type, PhysPageAllocatorV2 * ppman, TgtMemSetList *stlist);
    PageTable4K(PageTable4K * forked_pt, vector<std::pair<VPageIndexT, VPageIndexT>> &shared_interval, TgtMemSetList *stlist);
    ~PageTable4K();

    PTET pt_get(VPageIndexT vpg, PhysAddrT *tgtaddr);
    
    void pt_insert(VPageIndexT vpg, PTET pte, TgtMemSetList *stlist);
    void pt_update(VPageIndexT vpg, PTET pte, TgtMemSetList *stlist);
    void pt_erase(VPageIndexT vpg, TgtMemSetList *stlist);

    PhysAddrT get_page_table_base();

    void debug_print_pgtable();

private:

    PhysPageAllocatorV2 * ppman;
    PTType type;

    PAGE3 *root_sv48 = nullptr;
    PAGE2 *root_sv39 = nullptr;
};


class PhysPageAllocatorV2 {

public:

    PhysPageAllocatorV2(PhysAddrT start, uint64_t size);

    PageIndexT alloc();
    void reuse(PageIndexT ppindex);
    bool is_shared(PageIndexT ppindex);
    bool free(PageIndexT ppindex);

    uint64_t free_size() {
        return valid_pages.size() << PAGE_ADDR_OFFSET;
    }

protected:

    std::list<PageIndexT> valid_pages;
    std::unordered_map<PageIndexT, uint32_t> alloced_pages;

};


#endif
