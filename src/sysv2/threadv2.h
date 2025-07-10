#ifndef THREAD_V2_H
#define THREAD_V2_H

#include "common.h"
#include "simerror.h"

#include "cpu/isa.h"

#include "pagememv2.h"
#include "pagetablev2.h"
#include "hostfile.h"

#include <signal.h>
#include <sys/resource.h>

using isa::RVRegArray;

typedef sigset_t TgtSigsetT;

#ifndef DEFAULT_PID
#define DEFAULT_PID (10000)
#endif

enum class ThreadState {
    ready = 0,
    exec,
    wait
};

typedef struct {
    uint64_t    clone_flags;
    VirtAddrT   newsp;
    VirtAddrT   parent_tidptr;
    VirtAddrT   tls;
    VirtAddrT   child_tidptr;
} CloneParams;

typedef struct {
    __sighandler_t k_sa_handler;
    unsigned long sa_flags;
    sigset_t sa_mask;
} KernelSigaction;

typedef struct {
    VirtAddrT vaddr;
    vector<uint8_t> data;
} TgtVMemSet;

typedef uint64_t TgtTidT;
typedef uint64_t TgtTGidT;

class ThreadV2 {

public:

    ThreadV2(const ThreadV2& t) = delete;
    ThreadV2& operator=(const ThreadV2& t) = delete;

    ThreadV2(SimWorkload workload, PhysPageAllocatorV2 *ppman, VirtAddrT *out_entry, VirtAddrT *out_sp, TgtMemSetList * stlist);
    ThreadV2(ThreadV2 *parent_thread, TgtTidT newtid, uint64_t fork_flag, TgtMemSetList * stlist);

    VirtAddrT elf_load_dyn_lib(string elfpath, VirtAddrT *out_entry, TgtMemSetList * stlist);
    void elf_exec(SimWorkload &param, VirtAddrT *out_entry, VirtAddrT *out_sp, TgtMemSetList * stlist);

    PhysPageAllocatorV2 *ppman;

    TgtTidT tid;
    TgtTGidT tgid;
    AsidT asid;

    set<TgtTidT> childs;
    TgtTidT parent;

    ThreadState state = ThreadState::ready;

    shared_ptr<ThreadPageTableV2> pgtable;
    shared_ptr<unordered_map<int32_t, FileDescriptor*>> fdtable;
    shared_ptr<int32_t> alloc_fd;
    shared_ptr<unordered_map<int32_t, KernelSigaction>> sig_actions;
    shared_ptr<TgtSigsetT> sig_proc_mask;

    inline FileDescriptor* fdtable_trans(int32_t user_fd) {
        auto iter = fdtable->find(user_fd);
        if(iter == fdtable->end()) {
            return nullptr;
        }
        return iter->second;
    }
    inline int32_t fdtable_insert(FileDescriptor *new_fd) {
        int32_t ret = (*alloc_fd);
        (*alloc_fd)++;
        simroot_assertf(ret < (1 << 30), "File descripter run out");
        fdtable->insert(std::pair<int32_t, FileDescriptor*>(ret, new_fd));
        return ret;
    }
    inline FileDescriptor * fdtable_pop(int32_t user_fd) {
        auto iter = fdtable->find(user_fd);
        if(iter != fdtable->end()) {
            FileDescriptor * ret = iter->second;
            fdtable->erase(iter);
            return ret;
        }
        return nullptr;
    }

    vector<TgtVMemSet> stlist_on_ready;
    
    vector<RVRegArray> context_stack;

    uint64_t set_child_tid = 0UL;
    uint64_t clear_child_tid = 0UL;
    bool do_child_cleartid = false;

    rlimit rlimit_values[RLIM_NLIMITS];
};

#endif
