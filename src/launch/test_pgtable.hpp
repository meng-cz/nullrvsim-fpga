
#include "sysv2/pagetablev2.h"

#include <stdarg.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>

#include <sys/random.h>
#include <sys/resource.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <net/if.h>

#include <linux/futex.h>
#include <linux/limits.h>
#include <linux/sched.h>

void apply(TgtMemSetList &stlist, unordered_map<PageIndexT, TgtMemSetList> &pgstlists) {
    for(auto &st : stlist) {
        auto iter = pgstlists.find(st.base >> PAGE_ADDR_OFFSET);
        if(iter == pgstlists.end()) {
            iter = pgstlists.emplace(st.base >> PAGE_ADDR_OFFSET, TgtMemSetList()).first;
        }
        iter->second.push_back(st);
        iter->second.back().base -= ((st.base >> PAGE_ADDR_OFFSET) << PAGE_ADDR_OFFSET);
    }
}
void printpgs(unordered_map<PageIndexT, TgtMemSetList> &pgstlists) {
    for(auto &entry : pgstlists) {
        printf("PG 0x%lx: ", entry.first);
        for(auto &st : entry.second) {
            if(st.multivalue.empty() && st.dwords == PAGE_LEN_BYTE/8) printf("Fill 0x%lx, ", st.value);
            else if(st.dwords == 1) printf("Set 0x%lx as 0x%lx, ", st.base, st.value);
            else {
                printf("Fill value 0x%lx 0x%lx ... , ", st.multivalue[0], st.multivalue[1]);
            }
        }
        printf("\n");
    }
}
void printcps(vector<TgtPgCpy> &cplist) {
    for(auto &cp : cplist) {
        printf("Copy Page 0x%lx to 0x%lx\n", cp.src, cp.dst);
    }
}

bool test_pgtable() {
    vector<uint64_t> sample_page;
    sample_page.reserve(PAGE_LEN_BYTE/8);
    for(uint64_t i = 0; i < PAGE_LEN_BYTE/8; i++) {
        sample_page.push_back(i);
    }
    


    TgtMemSetList stlist1;

    PhysPageAllocatorV2 *ppman = new PhysPageAllocatorV2(0, 1<<30);

    TgtMemSetList stlist2;

    ThreadPageTableV2 * pgtable = new ThreadPageTableV2(PTType::SV39, ppman, &stlist2);
    pgtable->init_brk(0x10000UL);
    assert(pgtable->alloc_brk(0x12000UL, &stlist2) == 0x12000UL);

    VirtAddrT rwpriv = pgtable->alloc_mmap(0x2000UL, PGFLAG_R | PGFLAG_W | PGFLAG_ANON | PGFLAG_PRIV, nullptr, 0, "mmap", &stlist2);
    printf("Alloc RWANON @0x%lx\n", rwpriv);
    assert(rwpriv);

    FileDescriptor *shm = new FileDescriptor;
    shm->host_fd = 0;
    shm->st_size = 0x2000UL;
    shm->path = "shm";
    shm->ref_cnt = 1;
    shm->usr_seek = 0;

    VirtAddrT rwshm = pgtable->alloc_mmap(0x2000UL, PGFLAG_R | PGFLAG_W | PGFLAG_ANON | PGFLAG_SHARE, shm, 0, "shm", &stlist2);
    printf("Alloc RWSHM @0x%lx\n", rwshm);
    assert(rwshm);

    FileDescriptor *file = new FileDescriptor;
    file->host_fd = openat(-100, "example/i2048-a.txt", 0);
    file->st_size = 0x1000UL;
    file->path = "example/i2048-a.txt";
    file->ref_cnt = 1;
    file->usr_seek = 0;
    
    VirtAddrT rfile = pgtable->alloc_mmap(0x2000UL, PGFLAG_R | PGFLAG_PRIV, file, 0, "example/i2048-a.txt", &stlist2);
    printf("Alloc RFILE @0x%lx\n", rfile);
    assert(rfile);

    pgtable->debug_print_pgtable();

    printf("\n");

    TgtMemSetList stlist3;
    vector<TgtPgCpy> cplist3;
    pgtable->apply_cow(rwpriv, &stlist3, &cplist3);
    pgtable->apply_cow(rwshm, &stlist3, &cplist3);
    pgtable->apply_cow(rfile, &stlist3, &cplist3);

    printf("Apply COWs\n");

    pgtable->debug_print_pgtable();

    printf("\n");


    unordered_map<PageIndexT, TgtMemSetList> pgstlists1;

    printf("Current Pages:\n");
    apply(stlist1, pgstlists1);
    apply(stlist2, pgstlists1);
    apply(stlist3, pgstlists1);
    printpgs(pgstlists1);

    printf("\n");
    printf("Fork page table\n\n");

    TgtMemSetList stlist4;

    ThreadPageTableV2 * pgtable2 = new ThreadPageTableV2(pgtable, &stlist4);

    printf("Now Page Table 1:\n");
    pgtable->debug_print_pgtable();
    printf("\nNow Page Table 2:\n");
    pgtable2->debug_print_pgtable();

    printf("\nNewly Changed Pages:\n");
    unordered_map<PageIndexT, TgtMemSetList> pgstlists2;
    apply(stlist4, pgstlists2);
    printpgs(pgstlists2);

    printf("\n");
    printf("COW on BRK\n\n");

    TgtMemSetList stlist5;
    vector<TgtPgCpy> cplist5;

    pgtable->apply_cow(0x10000UL, &stlist5, &cplist5);
    pgtable2->apply_cow(0x10000UL, &stlist5, &cplist5);
    pgtable->apply_cow(rwshm + 0x1000UL, &stlist5, &cplist5);
    pgtable2->apply_cow(rwshm + 0x1000UL, &stlist5, &cplist5);
    pgtable->apply_cow(rwpriv, &stlist5, &cplist5);
    pgtable2->apply_cow(rwpriv, &stlist5, &cplist5);
    pgtable->apply_cow(rfile + 0x1000UL, &stlist5, &cplist5);
    pgtable2->apply_cow(rfile + 0x1000UL, &stlist5, &cplist5);

    printf("Now Page Table 1:\n");
    pgtable->debug_print_pgtable();
    printf("\nNow Page Table 2:\n");
    pgtable2->debug_print_pgtable();

    printf("\nNewly Changed Pages:\n");
    unordered_map<PageIndexT, TgtMemSetList> pgstlists3;
    apply(stlist5, pgstlists3);
    printpgs(pgstlists3);
    printcps(cplist5);



    return true;
}
