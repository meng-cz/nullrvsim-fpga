#ifndef SYSTEM_V2_H
#define SYSTEM_V2_H

#include "common.h"
#include "simroot.h"
#include "spinlocks.h"

#include "threadv2.h"

#include "cpuv2/cpuinterface2.h"

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

using simcpu::CPUGroupInterface;

using isa::RVRegArray;
using isa::ireg_index_of;
using isa::freg_index_of;

class SMPSystemV2 {

public:

    SMPSystemV2(SimWorkload &workload, CPUGroupInterface *cpus, uint32_t cpu_num, uint64_t membase, uint64_t memsz);

    void run_sim();

protected:

    void init_target_memory(TgtMemSetList &stlist);

    void _perform_target_memset(uint32_t cpu_id, TgtMemSet64 &st);
    void _perform_target_pagecpy(uint32_t cpu_id, TgtPgCpy &cp);

    VirtAddrT _pop_context_and_execute(uint32_t cpu_id);
    void _push_context_stack(uint32_t cpu_id, VirtAddrT nextpc);

    bool _memcpy_to_target(uint32_t cpu_id, VirtAddrT tgt_dst, void * src, uint64_t size);
    bool _memcpy_from_target(uint32_t cpu_id, void * dst, VirtAddrT tgt_src, uint64_t size);
    bool _strcpy_from_target(uint32_t cpu_id, char * dst, VirtAddrT tgt_src);


    bool has_init = false;
    bool has_hard_fp = false;

    uint64_t start_tick = 0;
    vector<uint64_t> start_uticks;

    PhysPageAllocatorV2 *ppman;

    uint32_t cpu_num = 0;
    CPUGroupInterface *cpus;

    uint64_t cur_tid_alloc = DEFAULT_PID;
    AsidT cur_asid_alloc = 1;

    DefaultLock sch_lock;

    unordered_map<TgtTidT, ThreadV2*> thread_objs;
    unordered_map<TgtTidT, int32_t> thread_exit_codes;
    unordered_map<TgtTGidT, vector<TgtTidT>> thread_groups;

    vector<ThreadV2*> running_threads;
    std::list<ThreadV2*> ready_threads;
    std::set<ThreadV2*> waiting_threads;

    /**
     * 核心上正在运行的进程需要换出
     * @return 核心是否有后续需要运行的进程,如果返回>0则running_threads[cpu_id]一定有效
    */
    VirtAddrT switch_next_thread_and_execute(uint32_t cpuid, uint32_t flag);
    const uint32_t SWFLAG_EXIT      = (1 << 0); // 该进程由于调用了Exit系统调用而被换出
    const uint32_t SWFLAG_YIELD     = (1 << 1); // 该进程由于调用了Yield系统调用而被换出,可被立即换回
    const uint32_t SWFLAG_WAIT      = (1 << 2); // 该进程由于等待被换出,不可被立即换回
    void insert_ready_thread_and_execute(ThreadV2 *thread, uint32_t prefered_cpu);




    typedef struct {
        ThreadV2 *      thread = nullptr;
        uint32_t        futex_mask = 0;
        uint32_t        last_cpu_id = 0;
    } FutexWaitThread;

    unordered_map<PhysAddrT, list<FutexWaitThread>> futex_wait_threads;
    inline void futex_wait_thread_insert(PhysAddrT paddr, ThreadV2 * thread, uint32_t futex_mask, uint32_t cpu_id) {
        auto res = futex_wait_threads.find(paddr);
        if(res == futex_wait_threads.end()) {
            futex_wait_threads.emplace(paddr, list<FutexWaitThread>());
            res = futex_wait_threads.find(paddr);
        }
        res->second.emplace_back(FutexWaitThread {
            .thread = thread, .futex_mask = futex_mask, .last_cpu_id = cpu_id
        });
    }
    inline bool futex_wait_thread_pop(PhysAddrT paddr, uint32_t futex_mask, FutexWaitThread* out) {
        auto res = futex_wait_threads.find(paddr);
        if(res != futex_wait_threads.end() && !res->second.empty()) {
            auto iter = res->second.begin();
            if(futex_mask != 0) {
                for(; iter != res->second.end(); iter++) {
                    if(iter->futex_mask == futex_mask) break;
                }
            }
            if(iter != res->second.end()) {
                if(out) *out = *iter;
                res->second.erase(iter);
                if(res->second.empty()) futex_wait_threads.erase(res);
                return true;
            }
        }
        return false;
    }

    typedef struct {
        SMPSystemV2 *   sys = nullptr;
        uint32_t        cpuid = 0;
        pthread_t       th;
        ThreadV2 *      thread = nullptr;
        VirtAddrT       tgt_fds = 0;
        int64_t         timeout = 0;
        vector<struct pollfd>   host_fds;
    } PollWaitThread;
    std::unordered_map<ThreadV2 *, PollWaitThread> poll_wait_threads;
    static void * poll_wait_thread_function(void * param);

    typedef struct {
        SMPSystemV2 *   sys = nullptr;
        uint32_t        cpuid = 0;
        pthread_t       th;
        ThreadV2 *      thread = nullptr;
        VirtAddrT       readfds = 0;
        VirtAddrT       writefds = 0;
        VirtAddrT       exceptfds = 0;
        fd_set          host_readfds;
        fd_set          host_writefds;
        fd_set          host_exceptfds;
        unordered_map<int, int> hostfd_to_simfd;
        int32_t         nfds = 0;
        int32_t         host_nfds = 0;
        int64_t         timeout = 0;
    } SelectWaitThread;
    std::unordered_map<ThreadV2 *, SelectWaitThread> select_wait_threads;
    static void * select_wait_thread_function(void * param);

    typedef struct {
        SMPSystemV2 *   sys = nullptr;
        uint32_t        cpuid = 0;
        pthread_t       th;
        ThreadV2 *      thread = nullptr;
        struct timespec host_time;
    } SleepWaitThread;
    std::unordered_map<ThreadV2 *, SleepWaitThread> sleep_wait_threads;
    static void * sleep_wait_thread_function(void * param);

    typedef struct {
        SMPSystemV2 *   sys = nullptr;
        uint32_t        cpuid = 0;
        pthread_t       th;
        ThreadV2 *      thread = nullptr;
        int32_t         simfd = 0;
        int32_t         hostfd = 0;
        VirtAddrT       buf = 0;
        uint64_t        bufsz = 0;
    } BlockreadWaitThread;
    std::unordered_map<ThreadV2 *, BlockreadWaitThread> blkread_wait_threads;
    static void * blkread_wait_thread_function(void * param);

    typedef struct {
        SMPSystemV2 *   sys = nullptr;
        uint32_t        cpuid = 0;
        pthread_t       th;
        ThreadV2 *      thread = nullptr;
        int32_t         simfd = 0;
        int32_t         hostfd = 0;
        uint32_t        flags = 0;
        VirtAddrT       buf;
        uint64_t        size;
        vector<uint8_t> addr;
    } SockSendWaitThread;
    std::unordered_map<ThreadV2 *, SockSendWaitThread> socksend_wait_threads;
    static void * socksend_wait_thread_function(void * param);

    typedef struct {
        SMPSystemV2 *   sys = nullptr;
        uint32_t        cpuid = 0;
        pthread_t       th;
        ThreadV2 *      thread = nullptr;
        int32_t         simfd = 0;
        int32_t         hostfd = 0;
        uint32_t        flags = 0;
        VirtAddrT       tgt_msg_hdr;
        struct msghdr   tgt_msg;
        vector<struct iovec> tgt_iovecs;
        vector<uint8_t> msg_name;
        vector<uint8_t> msg_control;
    } SockRecvMsgWaitThread;
    std::unordered_map<ThreadV2 *, SockRecvMsgWaitThread> sockrecvmsg_wait_threads;
    static void * sockrecvmsg_wait_thread_function(void * param);

    typedef struct {
        uint32_t        cpuid = 0;
        ThreadV2 *      thread = nullptr;
        int64_t         pid = 0;
        VirtAddrT       status = 0;
    } Wait4Thread;
    vector<Wait4Thread> wait4_wait_threads;
    typedef struct {
        int64_t tid;
        int64_t tgid;
        int64_t parent_tid;
        int32_t status;
    } ExitThreadToBeWaited;
    vector<ExitThreadToBeWaited> exited_threads_info;
    void wake_up_wait_threads(int64_t tid, int64_t tgid, int64_t parent_tid, int32_t status);
    bool try_wait(uint32_t cpuid, int64_t pid, ExitThreadToBeWaited *out);

    inline void cancle_wait_thread_nolock(ThreadV2 *thread) {
        for(auto &e : futex_wait_threads) {
            for(auto iter = e.second.begin(); iter != e.second.end(); ) {
                if(iter->thread == thread) iter = e.second.erase(iter);
                else iter++;
            }
        }
        poll_wait_threads.erase(thread);
        select_wait_threads.erase(thread);
        sleep_wait_threads.erase(thread);
        blkread_wait_threads.erase(thread);
        socksend_wait_threads.erase(thread);
        sockrecvmsg_wait_threads.erase(thread);
        waiting_threads.erase(thread);
    }



#define SYSCALL_FUNC_NAME_V2(num, name) syscall_##num##_##name
#define SYSCALL_CLAIM_V2(num, name) VirtAddrT SYSCALL_FUNC_NAME_V2(num,name)(uint32_t cpu_id, VirtAddrT pc)
#define SYSCALL_DEFINE_V2(num, name) VirtAddrT SMPSystemV2::SYSCALL_FUNC_NAME_V2(num,name)(uint32_t cpu_id, VirtAddrT pc)

    SYSCALL_CLAIM_V2(17, getcwd);
    SYSCALL_CLAIM_V2(48, faccessat);
    SYSCALL_CLAIM_V2(56, openat);
    SYSCALL_CLAIM_V2(57, close);
    SYSCALL_CLAIM_V2(62, lseek);
    SYSCALL_CLAIM_V2(63, read);
    SYSCALL_CLAIM_V2(64, write);
    SYSCALL_CLAIM_V2(72, pselect6);
    SYSCALL_CLAIM_V2(73, ppoll);
    SYSCALL_CLAIM_V2(78, readlinkat);
    SYSCALL_CLAIM_V2(79, newfstatat);
    SYSCALL_CLAIM_V2(80, fstat);
    SYSCALL_CLAIM_V2(93, exit);
    SYSCALL_CLAIM_V2(94, exitgroup);
    SYSCALL_CLAIM_V2(96, set_tid_address);
    SYSCALL_CLAIM_V2(98, futex);
    SYSCALL_CLAIM_V2(99, set_robust_list);
    SYSCALL_CLAIM_V2(113, clock_gettime);
    SYSCALL_CLAIM_V2(115, clock_nanosleep);
    SYSCALL_CLAIM_V2(124, sched_yield);
    SYSCALL_CLAIM_V2(134, sigaction);
    SYSCALL_CLAIM_V2(135, sigprocmask);
    SYSCALL_CLAIM_V2(160, uname);
    SYSCALL_CLAIM_V2(172, getpid);
    SYSCALL_CLAIM_V2(173, getppid);
    SYSCALL_CLAIM_V2(174, getuid);
    SYSCALL_CLAIM_V2(175, geteuid);
    SYSCALL_CLAIM_V2(176, getgid);
    SYSCALL_CLAIM_V2(177, getegid);
    SYSCALL_CLAIM_V2(178, gettid);
    SYSCALL_CLAIM_V2(206, sendto);
    SYSCALL_CLAIM_V2(212, recvmsg);
    SYSCALL_CLAIM_V2(214, brk);
    SYSCALL_CLAIM_V2(215, munmap);
    SYSCALL_CLAIM_V2(220, clone);
    SYSCALL_CLAIM_V2(222, mmap);
    SYSCALL_CLAIM_V2(226, mprotect);
    SYSCALL_CLAIM_V2(233, madvise);
    SYSCALL_CLAIM_V2(260, wait4);
    SYSCALL_CLAIM_V2(261, prlimit);
    SYSCALL_CLAIM_V2(278, getrandom);
    
    VirtAddrT _page_fault_rx(uint32_t cpu_id, VirtAddrT pc, VirtAddrT badaddr, bool isx);
    VirtAddrT _page_fault_w(uint32_t cpu_id, VirtAddrT pc, VirtAddrT badaddr);


    bool log_syscall = true;
};

#endif
