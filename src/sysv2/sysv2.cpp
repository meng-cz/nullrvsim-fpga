
#include "simroot.h"
#include "configuration.h"

#include "sysv2.h"

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

SMPSystemV2::SMPSystemV2(SimWorkload &workload, CPUGroupInterface *cpus, uint32_t cpu_num, uint64_t membase, uint64_t memsz)
: cpus(cpus), cpu_num(cpu_num) {
    
    running_threads.assign(cpu_num, nullptr);

    has_hard_fp = conf::get_int("root", "hard_fp", 1);

    TgtMemSetList stlist;

    ppman = new PhysPageAllocatorV2(membase, memsz);

    uint64_t entry = 0, sp = 0;
    ThreadV2 *init_thread = new ThreadV2(workload, ppman, &entry, &sp, &stlist);
    init_thread->asid = cur_asid_alloc;
    cur_asid_alloc++;
    init_thread->context_stack.emplace_back();
    isa::zero_regs(init_thread->context_stack.back());
    init_thread->context_stack.back()[0] = entry;
    init_thread->context_stack.back()[ireg_index_of("sp")] = sp;

    cur_tid_alloc = init_thread->tid + 1;
    init_thread->tgid = init_thread->tid;
    thread_groups.emplace(init_thread->tgid, vector<TgtTidT>());
    thread_groups[init_thread->tgid].push_back(init_thread->tid);
    thread_objs.emplace(init_thread->tid, init_thread);

    init_target_memory(stlist);

    if(conf::get_int("root", "debug_runtime", 0)) {
        simroot::debug_trace_object(true);
    }

    for(int i = 0; i < cpu_num; i++) {
        cpus->sync_inst_stream(i);
    }

    start_uticks.assign(cpu_num, 0);
    for(uint32_t i = 0; i < cpu_num; i++) {
        start_uticks[i] = cpus->get_current_utick(i);
    }
    start_tick = cpus->get_current_tick();

    printf("Init Simulation System With Entry 0x%lx, Stack 0x%lx\n", entry, sp);
    printf("Start Simulation at %ld Ticks\n", start_tick);

    insert_ready_thread_and_execute(init_thread, 0);

}

void SMPSystemV2::init_target_memory(TgtMemSetList &stlist) {

    typedef struct {
        bool full_init;
        unordered_map<uint32_t, uint64_t> set_words;
        vector<uint64_t> full_data;
    } SetPage;

    unordered_map<PageIndexT, SetPage> mod_pages;

    uint64_t total_set_sz = 0, current_set_sz = 0;
    for(auto &st : stlist) {
        total_set_sz += st.dwords * 8;
    }
    printf("Init Target Memory: (0/%ld) KB", total_set_sz / 1024);
    for(int i = 0; i < stlist.size(); i++) {
        auto &st = stlist[i];
        _perform_target_memset(0, stlist[i]);
        current_set_sz += stlist[i].dwords * 8;
        printf("\rInit Target Memory: (%ld/%ld) KB", current_set_sz / 1024, total_set_sz / 1024);
        fflush(stdout);
        
        PageIndexT pg = (st.base >> 12);
        uint64_t offset = (st.base & (PAGE_LEN_BYTE - 1));
        auto iter = mod_pages.find(pg);
        if(iter == mod_pages.end()) {
            iter = mod_pages.emplace(pg, SetPage()).first;
            iter->second.full_init = false;
        }
        if(st.dwords == PAGE_LEN_BYTE/8 && !offset) {
            if(st.multivalue.size() == st.dwords) {
                iter->second.full_init = true;
                iter->second.full_data = st.multivalue;
            } else {
                iter->second.full_init = true;
                iter->second.full_data.assign(PAGE_LEN_BYTE/8, st.value);
            }
        } else if (iter->second.full_init) {
            iter->second.full_data[offset >> 3] = st.value;
        } else {
            iter->second.set_words.emplace(offset >> 3, st.value);
        }
    }
    printf("\n");

    uint64_t total_page = mod_pages.size(), cur_page = 0;
    printf("Validate Target Memory: Page (%ld/%ld)", cur_page, total_page);
    for(auto &entry : mod_pages) {
        if(entry.second.full_init) {
            vector<uint64_t> rd;
            rd.assign(PAGE_LEN_BYTE/8, 0);
            cpus->pxymem_page_read(0, entry.first, rd.data());
            for(uint64_t i = 0; i < PAGE_LEN_BYTE/8; i++) {
                simroot_assertf(rd[i] == entry.second.full_data[i], "Validation Failed @0x%lx: Required 0x%lx, but Read 0x%lx\n",
                    entry.first * PAGE_LEN_BYTE + i * 8, entry.second.full_data[i], rd[i]);
            }
        } else {
            for(auto &s : entry.second.set_words) {
                uint64_t addr = entry.first * PAGE_LEN_BYTE + s.first * 8;
                uint64_t rd = cpus->pxymem_read(0, addr);
                simroot_assertf(rd == s.second, "Validation Failed @0x%lx: Required 0x%lx, but Read 0x%lx\n", addr, s.second, rd);
            }
        }

        printf("\rValidate Target Memory: Page (%ld/%ld)", cur_page, total_page);
        fflush(stdout);
        cur_page ++;
    }
    printf("\n");

}

VirtAddrT SMPSystemV2::_pop_context_and_execute(uint32_t cpu_id) {
    assert(running_threads[cpu_id]);
    ThreadV2 *thread = running_threads[cpu_id];

    // Set MMU
    cpus->set_mmu(cpu_id, thread->pgtable->get_page_table_base(), 0);
    cpus->flush_tlb_all(cpu_id);

    // Flush memory operation during waiting
    for(auto &st : thread->stlist_on_ready) {
        simroot_assert(_memcpy_to_target(cpu_id, st.vaddr, st.data.data(), st.data.size()));
    }
    thread->stlist_on_ready.clear();

    // Recover registers
    RVRegArray context = thread->context_stack.back();
    thread->context_stack.pop_back();
    uint32_t regnum = (has_hard_fp?64:32);
    for(uint32_t i = 1; i < regnum; i++) {
        cpus->regacc_write(cpu_id, i, context[i]);
    }

    // Redirect
    VirtAddrT pc = context[0];
    cpus->redirect(cpu_id, pc);

    return pc;
}

void SMPSystemV2::_push_context_stack(uint32_t cpu_id, VirtAddrT nextpc) {
    assert(running_threads[cpu_id]);
    ThreadV2 *thread = running_threads[cpu_id];

    thread->context_stack.emplace_back();
    RVRegArray &context = thread->context_stack.back();
    uint32_t regnum = (has_hard_fp?64:32);
    for(uint32_t i = 1; i < regnum; i++) {
        context[i] = cpus->regacc_read(cpu_id, i);
    }
    context[0] = nextpc;
}

VirtAddrT SMPSystemV2::switch_next_thread_and_execute(uint32_t cpuid, uint32_t flag) {
    VirtAddrT ret = 0;
    ThreadV2 *thread = running_threads[cpuid];
    if(flag == SWFLAG_EXIT) {
        if(!ready_threads.empty()) {
            running_threads[cpuid] = ready_threads.front();
            ready_threads.pop_front();
            ret = _pop_context_and_execute(cpuid);
            if(log_syscall) {
                printf("%ld: SCHD: Exit thread @CPU %d -> thread %ld\n", cpus->get_current_tick(), cpuid, ret?(running_threads[cpuid]->tid):0);
            }
        }
    }
    else if(flag == SWFLAG_WAIT) {
        assert(thread);
        uint64_t tid = thread->tid;
        waiting_threads.insert(thread);
        running_threads[cpuid] = nullptr;
        if(!ready_threads.empty()) {
            running_threads[cpuid] = ready_threads.front();
            ready_threads.pop_front();
            ret = _pop_context_and_execute(cpuid);
        }
        if(log_syscall) {
            printf("%ld: SCHD: Waited thread %ld @CPU %d -> thread %ld\n", cpus->get_current_tick(), tid, cpuid, ret?(running_threads[cpuid]->tid):0);
        }
    }
    else if(flag == SWFLAG_YIELD) {
        assert(thread);
        uint64_t tid = thread->tid;
        ready_threads.push_back(thread);
        running_threads[cpuid] = ready_threads.front();
        ready_threads.pop_front();
        ret = _pop_context_and_execute(cpuid);
        if(log_syscall) {
            printf("%ld: SCHD: Yield thread %ld @CPU %d -> thread %ld\n", cpus->get_current_tick(), tid, cpuid, running_threads[cpuid]->tid);
        }
        return true;
    }
    else {
        LOG(ERROR) << "Unsupported switch flag " << flag;
        simroot_assert(0);
    }
    return ret;
}

void SMPSystemV2::insert_ready_thread_and_execute(ThreadV2 *thread, uint32_t prefered_cpu) {
    uint32_t cpuid = cpu_num;
    if(running_threads[prefered_cpu] == nullptr) {
        cpuid = prefered_cpu;
    }
    else {
        for(uint32_t i = 0; i < cpu_num; i++) {
            if(running_threads[i] == nullptr) {
                cpuid = i;
                break;
            }
        }
    }
    waiting_threads.erase(thread);
    if(cpuid < cpu_num) {
        running_threads[cpuid] = thread;
        VirtAddrT nextpc = _pop_context_and_execute(cpuid);
    }
    else {
        ready_threads.push_back(thread);
    }
    if(log_syscall) {
        if(cpuid < cpu_num) printf("%ld: SCHD: Ready thread %ld -> CPU %d\n", cpus->get_current_tick(), thread->tid, cpuid);
        else printf("%ld: SCHD: Ready thread %ld -> Wait Queue\n", cpus->get_current_tick(), thread->tid);
    }
}


bool SMPSystemV2::_memcpy_to_target(uint32_t cpu_id, VirtAddrT tgt_dst, void * src, uint64_t size) {
    ThreadV2 *curt = running_threads[cpu_id];

    VPageIndexT vpn = 0;
    PageIndexT ppn = 0;

    auto check_vaddr_prot = [&](VirtAddrT va) -> bool {
        if(vpn != (va >> PAGE_ADDR_OFFSET)) {
            vpn = (va >> PAGE_ADDR_OFFSET);
            PTET pte = curt->pgtable->pt_get(vpn, nullptr);
            if((pte & PTE_V) && (pte & PTE_COW)) {
                TgtMemSetList stlist;
                vector<TgtPgCpy> cplist;
                curt->pgtable->apply_cow(vpn << PAGE_ADDR_OFFSET, &stlist, &cplist);
                for(auto &st : stlist) {
                    _perform_target_memset(cpu_id, st);
                }
                for(auto &cp : cplist) {
                    _perform_target_pagecpy(cpu_id, cp);
                }
                // cpus->flush_tlb_vpgidx(cpu_id, vpn << PAGE_ADDR_OFFSET, curt->asid);
                cpus->flush_tlb_all(cpu_id);
                pte = curt->pgtable->pt_get(vpn, nullptr);
            }
            else if(!((pte & PTE_V) && (pte & PTE_W))) {
                return false;
            }
            ppn = (pte >> 10);
        }
        return true;
    };

    const uint64_t align = 8;
    uint64_t bytes = 0;
    
    VirtAddrT start = (tgt_dst & (~(align - 1)));
    if(start < tgt_dst) {
        if(!check_vaddr_prot(start)) return false;
        PhysAddrT pa = (start & (PAGE_LEN_BYTE - 1)) + (ppn << PAGE_ADDR_OFFSET);
        uint64_t off = tgt_dst - start;
        uint64_t end = std::min<uint64_t>(align, size + off);
        RawDataT v = cpus->pxymem_read(cpu_id, pa);
        uint8_t *p2 = (uint8_t*)(&v);
        uint8_t *p1 = (uint8_t*)(src);
        for(uint64_t i = off; i < end; i++) {
            p2[i] = p1[i - off];
        }
        cpus->pxymem_write(cpu_id, pa, v);
        bytes += (end - off);
    }

    while(bytes < size) {
        uint64_t sz = std::min<uint64_t>(align, size - bytes);
        VirtAddrT va = tgt_dst + bytes;
        if(!check_vaddr_prot(va)) return false;
        PhysAddrT pa = (va & (PAGE_LEN_BYTE - 1)) + (ppn << PAGE_ADDR_OFFSET);
        RawDataT v = 0;
        if(sz < align) {
            v = cpus->pxymem_read(cpu_id, pa);
        }
        memcpy(&v, (char*)src + bytes, sz);
        cpus->pxymem_write(cpu_id, pa, v);
        bytes += sz;
    }

    return true;
}

bool SMPSystemV2::_memcpy_from_target(uint32_t cpu_id, void * dst, VirtAddrT tgt_src, uint64_t size) {
    ThreadV2 *curt = running_threads[cpu_id];

    VPageIndexT vpn = 0;
    PageIndexT ppn = 0;

    auto check_vaddr_prot = [&](VirtAddrT va) -> bool {
        if(vpn != (va >> PAGE_ADDR_OFFSET)) {
            vpn = (va >> PAGE_ADDR_OFFSET);
            PTET pte = curt->pgtable->pt_get(vpn, nullptr);
            if((pte & PTE_V) && (pte & PTE_NALLOC)) {
                TgtMemSetList stlist;
                vector<TgtPgCpy> cplist;
                curt->pgtable->apply_cow(vpn << PAGE_ADDR_OFFSET, &stlist, &cplist);
                for(auto &st : stlist) {
                    _perform_target_memset(cpu_id, st);
                }
                for(auto &cp : cplist) {
                    _perform_target_pagecpy(cpu_id, cp);
                }
                // cpus->flush_tlb_vpgidx(cpu_id, vpn << PAGE_ADDR_OFFSET, curt->asid);
                cpus->flush_tlb_all(cpu_id);
                pte = curt->pgtable->pt_get(vpn, nullptr);
            }
            else if(!(pte & PTE_V)) {
                return false;
            }
            ppn = (pte >> 10);
        }
        return true;
    };

    const uint64_t align = 8;
    uint64_t bytes = 0;

    VirtAddrT start = (tgt_src & (~(align - 1)));
    if(start < tgt_src) {
        if(!check_vaddr_prot(start)) return false;
        PhysAddrT pa = (start & (PAGE_LEN_BYTE - 1)) + (ppn << PAGE_ADDR_OFFSET);
        uint64_t off = tgt_src - start;
        uint64_t end = std::min<uint64_t>(align, size + off);
        RawDataT v = (cpus->pxymem_read(cpu_id, pa) >> (off * 8));
        memcpy(dst, &v, end - off);
        bytes += (end - off);
    }

    while(bytes < size) {
        uint64_t sz = std::min<uint64_t>(align, size - bytes);
        VirtAddrT va = tgt_src + bytes;
        if(!check_vaddr_prot(va)) return false;
        PhysAddrT pa = (va & (PAGE_LEN_BYTE - 1)) + (ppn << PAGE_ADDR_OFFSET);
        RawDataT v = cpus->pxymem_read(cpu_id, pa);
        memcpy((char*)dst + bytes, &v, sz);
        bytes += sz;
    }

    return true;
}

bool SMPSystemV2::_strcpy_from_target(uint32_t cpu_id, char * dst, VirtAddrT tgt_src) {
    ThreadV2 *curt = running_threads[cpu_id];

    VPageIndexT vpn = 0;
    PageIndexT ppn = 0;

    auto check_vaddr_prot = [&](VirtAddrT va) -> bool {
        if(vpn != (va >> PAGE_ADDR_OFFSET)) {
            vpn = (va >> PAGE_ADDR_OFFSET);
            PTET pte = curt->pgtable->pt_get(vpn, nullptr);
            if((pte & PTE_V) && (pte & PTE_NALLOC)) {
                TgtMemSetList stlist;
                vector<TgtPgCpy> cplist;
                curt->pgtable->apply_cow(vpn << PAGE_ADDR_OFFSET, &stlist, &cplist);
                for(auto &st : stlist) {
                    _perform_target_memset(cpu_id, st);
                }
                for(auto &cp : cplist) {
                    _perform_target_pagecpy(cpu_id, cp);
                }
                // cpus->flush_tlb_vpgidx(cpu_id, vpn << PAGE_ADDR_OFFSET, curt->asid);
                cpus->flush_tlb_all(cpu_id);
                pte = curt->pgtable->pt_get(vpn, nullptr);
            }
            else if(!(pte & PTE_V)) {
                return false;
            }
            ppn = (pte >> 10);
        }
        return true;
    };

    const uint64_t align = 8;
    uint64_t bytes = 0;

    VirtAddrT start = (tgt_src & (~(align - 1)));
    if(start < tgt_src) {
        if(!check_vaddr_prot(start)) return false;
        PhysAddrT pa = (start & (PAGE_LEN_BYTE - 1)) + (ppn << PAGE_ADDR_OFFSET);
        uint64_t off = tgt_src - start;
        RawDataT v = cpus->pxymem_read(cpu_id, pa);
        char *p = (char *)(&v);
        for(uint64_t i = off; i < align; i++, bytes++) {
            dst[bytes] = p[i];
            if(p[i] == 0) return true;
        }
    }

    while(1) {
        VirtAddrT va = tgt_src + bytes;
        if(!check_vaddr_prot(va)) return false;
        PhysAddrT pa = (va & (PAGE_LEN_BYTE - 1)) + (ppn << PAGE_ADDR_OFFSET);
        RawDataT v = cpus->pxymem_read(cpu_id, pa);
        char *p = (char *)(&v);
        for(uint64_t i = 0; i < align; i++, bytes++) {
            dst[bytes] = p[i];
            if(p[i] == 0) return true;
        }
    }
}

void SMPSystemV2::_perform_target_memset(uint32_t cpu_id, TgtMemSet64 &st) {
    if(st.dwords == PAGE_LEN_BYTE/8 && !(st.base & (PAGE_LEN_BYTE - 1))) {
        if(st.multivalue.size() == st.dwords) {
            cpus->pxymem_page_write(cpu_id, st.base >> PAGE_ADDR_OFFSET, st.multivalue.data());
        } else {
            cpus->pxymem_page_set(cpu_id, st.base >> PAGE_ADDR_OFFSET, st.value);
        }
    } else {
        simroot_assertf((st.dwords == 1) && !(st.base & 7), "Bad target memset on 0x%lx with length %ld dwords", st.base, st.dwords);
        // if(st.multivalue.size() == st.dwords) {
        //     for(uint64_t i = 0; i < st.dwords; i++) {
        //         cpus->pxymem_write(cpu_id, st.base + i * 8, st.multivalue[i]);
        //     }
        // } else {
        //     for(uint64_t i = 0; i < st.dwords; i++) {
        //         cpus->pxymem_write(cpu_id, st.base + i * 8, st.value);
        //     }
        // }
        cpus->pxymem_write(cpu_id, st.base, st.value);
    }
}

void SMPSystemV2::_perform_target_pagecpy(uint32_t cpu_id, TgtPgCpy &cp) {
    cpus->pxymem_page_copy(cpu_id, cp.dst, cp.src);
}

void SMPSystemV2::wake_up_wait_threads(int64_t tid, int64_t tgid, int64_t parent_tid, int32_t status) {
    for(auto iter = wait4_wait_threads.begin(); iter != wait4_wait_threads.end(); iter++) {
        auto &wt = *iter;
        if(parent_tid != wt.thread->tid) continue;
        int64_t pid = wt.pid;
        bool success = false;
        if(pid < -1) {
            TgtTGidT target_tgid = (-pid);
            if(target_tgid == tgid) {
                success = true;
            }
        }
        else if(pid == -1) {
            success = true;
        }
        else if(pid == 0) {
            TgtTGidT target_tgid = wt.thread->tgid;
            if(target_tgid == tgid) {
                success = true;
            }
        }
        else {
            if(tid == pid) {
                success = true;
            }
        }
        if(success) {
            TgtVMemSet st;
            st.vaddr = wt.status;
            st.data.assign(sizeof(status), 0);
            memcpy(st.data.data(), &status, sizeof(status));
            ThreadV2 *thread = wt.thread;
            thread->stlist_on_ready.push_back(st);
            thread->context_stack.back()[ireg_index_of("a0")] = tid;
            insert_ready_thread_and_execute(thread, wt.cpuid);
            wait4_wait_threads.erase(iter);
            return;
        }
    }
    // No one catched this thread
    exited_threads_info.emplace_back();
    auto &i = exited_threads_info.back();
    i.tid = tid;
    i.tgid = tgid;
    i.parent_tid = parent_tid;
    i.status = status;
}

bool SMPSystemV2::try_wait(uint32_t cpuid, int64_t pid, ExitThreadToBeWaited *out) {
    ThreadV2 * curt = running_threads[cpuid];
    for(auto iter = exited_threads_info.begin(); iter != exited_threads_info.end(); iter++) {
        auto &info = *iter;
        if(info.parent_tid != curt->tid) continue;
        bool success = false;
        if(pid < -1) {
            TgtTGidT target_tgid = (-pid);
            if(target_tgid == info.tgid) {
                success = true;
            }
        }
        else if(pid == -1) {
            success = true;
        }
        else if(pid == 0) {
            TgtTGidT target_tgid = curt->tgid;
            if(target_tgid == info.tgid) {
                success = true;
            }
        }
        else {
            if(info.tid == pid) {
                success = true;
            }
        }
        if(success) {
            *out = info;
            exited_threads_info.erase(iter);
            return true;
        }
    }
    return false;
}

void * SMPSystemV2::poll_wait_thread_function(void * _param) {
    PollWaitThread *p = (PollWaitThread*)_param;
    SMPSystemV2 * sys = p->sys;

    int64_t ret = poll(p->host_fds.data(), p->host_fds.size(), p->timeout);
    if(ret < 0) ret = -errno;
    else {
        for(uint64_t i = 0; i < p->host_fds.size(); i++) {
            TgtVMemSet st;
            st.vaddr = p->tgt_fds + i * sizeof(struct pollfd) + 6;
            st.data.assign(sizeof((p->host_fds)[i].revents), 0);
            memcpy(st.data.data(), &((p->host_fds)[i].revents), sizeof((p->host_fds)[i].revents));
            p->thread->stlist_on_ready.emplace_back(st);
        }
    }
    p->thread->context_stack.back()[ireg_index_of("a0")] = ret;

    if(sys->log_syscall) {
        printf("Thread %ld Syscall ppoll Return %ld", p->thread->tid, ret);
    }

    sys->sch_lock.lock();
    sys->poll_wait_threads.erase(p->thread);
    sys->insert_ready_thread_and_execute(p->thread, p->cpuid);
    sys->sch_lock.unlock();

    return nullptr;
}

void * SMPSystemV2::select_wait_thread_function(void * _param) {
    SelectWaitThread *p = (SelectWaitThread*)_param;
    SMPSystemV2 * sys = p->sys;

    timespec timeout;
    timeout.tv_sec = p->timeout / 1000UL;
    timeout.tv_nsec = 1000UL * p->timeout;
    
    int64_t ret = pselect(
        p->host_nfds,
        ((p->readfds)?(&p->host_readfds):nullptr),
        ((p->writefds)?(&p->host_writefds):nullptr),
        ((p->exceptfds)?(&p->host_exceptfds):nullptr),
        (p->timeout >= 0)?(&timeout):nullptr,
        nullptr
    );
    if(ret < 0) ret = -errno;
    p->thread->context_stack.back()[ireg_index_of("a0")] = ret;

    if(ret > 0) {
        if(p->readfds) {
            TgtVMemSet st;
            st.vaddr = p->readfds;
            st.data.assign(sizeof(fd_set), 0);
            FD_ZERO((fd_set*)st.data.data());
            for(int i = 0; i < p->host_nfds; i++) {
                if(FD_ISSET(i, &p->host_readfds)) FD_SET(p->hostfd_to_simfd[i], (fd_set*)st.data.data());
            }
            p->thread->stlist_on_ready.emplace_back(st);
        }
        if(p->writefds) {
            TgtVMemSet st;
            st.vaddr = p->readfds;
            st.data.assign(sizeof(fd_set), 0);
            FD_ZERO((fd_set*)st.data.data());
            for(int i = 0; i < p->host_nfds; i++) {
                if(FD_ISSET(i, &p->host_writefds)) FD_SET(p->hostfd_to_simfd[i], (fd_set*)st.data.data());
            }
            p->thread->stlist_on_ready.emplace_back(st);
        }
        if(p->exceptfds) {
            TgtVMemSet st;
            st.vaddr = p->readfds;
            st.data.assign(sizeof(fd_set), 0);
            FD_ZERO((fd_set*)st.data.data());
            for(int i = 0; i < p->host_nfds; i++) {
                if(FD_ISSET(i, &p->host_exceptfds)) FD_SET(p->hostfd_to_simfd[i], (fd_set*)st.data.data());
            }
            p->thread->stlist_on_ready.emplace_back(st);
        }
    }

    if(sys->log_syscall) {
        printf("Thread %ld Syscall pselect6 Return %ld", p->thread->tid, ret);
    }

    sys->sch_lock.lock();
    sys->select_wait_threads.erase(p->thread);
    sys->insert_ready_thread_and_execute(p->thread, p->cpuid);
    sys->sch_lock.unlock();

    return nullptr;
}

void * SMPSystemV2::sleep_wait_thread_function(void * _param) {
    SleepWaitThread *p = (SleepWaitThread*)_param;
    SMPSystemV2 * sys = p->sys;

    int64_t ret = clock_nanosleep(0, 0, &p->host_time, 0);
    if(ret < 0) ret = -errno;
    p->thread->context_stack.back()[ireg_index_of("a0")] = ret;

    if(sys->log_syscall) {
        printf("Thread %ld Syscall clock_nanosleep Return %ld", p->thread->tid, ret);
    }

    sys->sch_lock.lock();
    sys->sleep_wait_threads.erase(p->thread);
    sys->insert_ready_thread_and_execute(p->thread, p->cpuid);
    sys->sch_lock.unlock();

    return nullptr;
}

void * SMPSystemV2::blkread_wait_thread_function(void * _param) {
    BlockreadWaitThread *p = (BlockreadWaitThread*)_param;
    SMPSystemV2 * sys = p->sys;

    TgtVMemSet st;
    st.vaddr = p->buf;
    st.data.assign(p->bufsz, 0);

    int64_t ret = read(p->hostfd, st.data.data(), p->bufsz);
    if(ret < 0) ret = -errno;
    else {
        p->thread->stlist_on_ready.emplace_back(st);
    }
    p->thread->context_stack.back()[ireg_index_of("a0")] = ret;

    if(sys->log_syscall) {
        printf("Thread %ld Syscall clock_nanosleep Return %ld", p->thread->tid, ret);
    }

    sys->sch_lock.lock();
    sys->blkread_wait_threads.erase(p->thread);
    sys->insert_ready_thread_and_execute(p->thread, p->cpuid);
    sys->sch_lock.unlock();

    return nullptr;
}

void * SMPSystemV2::socksend_wait_thread_function(void * _param) {
    SockSendWaitThread *p = (SockSendWaitThread*)_param;
    SMPSystemV2 * sys = p->sys;

    TgtVMemSet st;
    st.vaddr = p->buf;
    st.data.assign(p->size, 0);

    int64_t ret = sendto(p->hostfd, st.data.data(), p->size, p->flags, (sockaddr*)(p->addr.data()), p->addr.size());
    if(ret < 0) ret = -errno;
    else {
        p->thread->stlist_on_ready.emplace_back(st);
    }
    p->thread->context_stack.back()[ireg_index_of("a0")] = ret;

    if(sys->log_syscall) {
        printf("Thread %ld Syscall sendto Return %ld", p->thread->tid, ret);
    }

    sys->sch_lock.lock();
    sys->socksend_wait_threads.erase(p->thread);
    sys->insert_ready_thread_and_execute(p->thread, p->cpuid);
    sys->sch_lock.unlock();

    return nullptr;
}

void * SMPSystemV2::sockrecvmsg_wait_thread_function(void * _param) {
    SockRecvMsgWaitThread *p = (SockRecvMsgWaitThread*)_param;
    SMPSystemV2 * sys = p->sys;

    struct msghdr host_msg;
    vector<struct iovec> iovec_buf;
    vector<vector<uint8_t>> data_buf;

    for(uint64_t i = 0; i < p->tgt_iovecs.size(); i++) {
        data_buf.emplace_back();
        data_buf.back().assign(p->tgt_iovecs[i].iov_len, 0);
        struct iovec tmp;
        tmp.iov_base = data_buf.back().data();
        tmp.iov_len = p->tgt_iovecs[i].iov_len;
        iovec_buf.push_back(tmp);
    }

    host_msg.msg_name = ((p->msg_name.empty())?nullptr:(p->msg_name.data()));
    host_msg.msg_namelen = p->msg_name.size();
    host_msg.msg_control = ((p->msg_control.empty())?nullptr:(p->msg_control.data()));
    host_msg.msg_controllen = p->msg_control.size();
    host_msg.msg_flags = p->tgt_msg.msg_flags;
    host_msg.msg_iov = ((iovec_buf.empty())?nullptr:(iovec_buf.data()));
    host_msg.msg_iovlen = iovec_buf.size();

    int64_t ret = recvmsg(p->hostfd, &host_msg, p->flags);
    if(ret < 0) ret = -errno;
    else {
        int64_t sum = 0;
        for(uint64_t i = 0; i < iovec_buf.size() && sum < ret; i++) {
            int64_t step = ret - sum;
            if(step > iovec_buf[i].iov_len) step = iovec_buf[i].iov_len;
            TgtVMemSet st;
            st.vaddr = (VirtAddrT)(p->tgt_iovecs[i].iov_base);
            st.data = data_buf[i];
            st.data.resize(step);
            p->thread->stlist_on_ready.emplace_back(st);
            sum += step;
        }
        if(host_msg.msg_controllen) {
            TgtVMemSet st;
            st.vaddr = (VirtAddrT)(p->tgt_msg.msg_control);
            st.data = p->msg_control;
            st.data.resize(host_msg.msg_controllen);
            p->thread->stlist_on_ready.emplace_back(st);
            
            TgtVMemSet st2;
            st2.vaddr = (p->tgt_msg_hdr) + offsetof(struct msghdr, msg_controllen);
            st2.data.assign(sizeof(host_msg.msg_controllen), 0);
            memcpy(st2.data.data(), &(host_msg.msg_controllen), sizeof(host_msg.msg_controllen));
            p->thread->stlist_on_ready.emplace_back(st2);
        }
        if(host_msg.msg_namelen) {
            TgtVMemSet st;
            st.vaddr = (VirtAddrT)(p->tgt_msg.msg_name);
            st.data = p->msg_name;
            st.data.resize(host_msg.msg_namelen);
            p->thread->stlist_on_ready.emplace_back(st);
            
            TgtVMemSet st2;
            st2.vaddr = (p->tgt_msg_hdr) + offsetof(struct msghdr, msg_namelen);
            st2.data.assign(sizeof(host_msg.msg_namelen), 0);
            memcpy(st2.data.data(), &(host_msg.msg_namelen), sizeof(host_msg.msg_namelen));
            p->thread->stlist_on_ready.emplace_back(st2);
        }
        TgtVMemSet st2;
        st2.vaddr = (p->tgt_msg_hdr) + offsetof(struct msghdr, msg_flags);
        st2.data.assign(sizeof(host_msg.msg_flags), 0);
        memcpy(st2.data.data(), &(host_msg.msg_flags), sizeof(host_msg.msg_flags));
        p->thread->stlist_on_ready.emplace_back(st2);
    }

    p->thread->context_stack.back()[ireg_index_of("a0")] = ret;

    if(sys->log_syscall) {
        printf("Thread %ld Syscall recvmsg Return %ld", p->thread->tid, ret);
    }

    sys->sch_lock.lock();
    sys->sockrecvmsg_wait_threads.erase(p->thread);
    sys->insert_ready_thread_and_execute(p->thread, p->cpuid);
    sys->sch_lock.unlock();

    return nullptr;
}








void SMPSystemV2::run_sim() {
    assert(running_threads[0]);

    uint32_t cpu_id = 0;
    uint32_t cause = 0;
    RawDataT arg = 0;
    VirtAddrT pc = 0;

#define SYSCALL_CASE_V2(num, name) case num: nextpc = SYSCALL_FUNC_NAME_V2(num, name)(cpu_id, pc); break;

    while(cpus->next(&cpu_id, &pc, &cause, &arg)) {
        RawDataT ecallid = cpus->regacc_read(cpu_id, ireg_index_of("a7"));
        VirtAddrT nextpc = 0;
        switch (cause)
        {
        case ITR_USR_ECALL:
            switch (ecallid)
            {
            SYSCALL_CASE_V2(17, getcwd);
            SYSCALL_CASE_V2(48, faccessat);
            SYSCALL_CASE_V2(56, openat);
            SYSCALL_CASE_V2(57, close);
            SYSCALL_CASE_V2(62, lseek);
            SYSCALL_CASE_V2(63, read);
            SYSCALL_CASE_V2(64, write);
            SYSCALL_CASE_V2(72, pselect6);
            SYSCALL_CASE_V2(73, ppoll);
            SYSCALL_CASE_V2(78, readlinkat);
            SYSCALL_CASE_V2(79, newfstatat);
            SYSCALL_CASE_V2(80, fstat);
            SYSCALL_CASE_V2(93, exit);
            SYSCALL_CASE_V2(94, exitgroup);
            SYSCALL_CASE_V2(96, set_tid_address);
            SYSCALL_CASE_V2(98, futex);
            SYSCALL_CASE_V2(99, set_robust_list);
            SYSCALL_CASE_V2(113, clock_gettime);
            SYSCALL_CASE_V2(115, clock_nanosleep);
            SYSCALL_CASE_V2(124, sched_yield);
            SYSCALL_CASE_V2(134, sigaction);
            SYSCALL_CASE_V2(135, sigprocmask);
            SYSCALL_CASE_V2(160, uname);
            SYSCALL_CASE_V2(172, getpid);
            SYSCALL_CASE_V2(173, getppid);
            SYSCALL_CASE_V2(174, getuid);
            SYSCALL_CASE_V2(175, geteuid);
            SYSCALL_CASE_V2(176, getgid);
            SYSCALL_CASE_V2(177, getegid);
            SYSCALL_CASE_V2(178, gettid);
            SYSCALL_CASE_V2(206, sendto);
            SYSCALL_CASE_V2(212, recvmsg);
            SYSCALL_CASE_V2(214, brk);
            SYSCALL_CASE_V2(215, munmap);
            SYSCALL_CASE_V2(220, clone);
            SYSCALL_CASE_V2(222, mmap);
            SYSCALL_CASE_V2(226, mprotect);
            SYSCALL_CASE_V2(233, madvise);
            SYSCALL_CASE_V2(260, wait4);
            SYSCALL_CASE_V2(261, prlimit);
            SYSCALL_CASE_V2(278, getrandom);
            default:
                printf("CPU%d Raise an Unkonwn ECALL %ld @0x%lx, arg:0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
                cpu_id, ecallid, pc,
                cpus->regacc_read(cpu_id, ireg_index_of("a0")),
                cpus->regacc_read(cpu_id, ireg_index_of("a1")),
                cpus->regacc_read(cpu_id, ireg_index_of("a2")),
                cpus->regacc_read(cpu_id, ireg_index_of("a3")),
                cpus->regacc_read(cpu_id, ireg_index_of("a4")),
                cpus->regacc_read(cpu_id, ireg_index_of("a5")));
                simroot_assert(0);
            }
            break;
        case ITR_INST_PGFAULT:
            nextpc = _page_fault_rx(cpu_id, pc, arg, true);
            break;
        case ITR_LD_PGFAULT:
            nextpc = _page_fault_rx(cpu_id, pc, arg, false);
            break;
        case ITR_ST_PGFAULT:
            nextpc = _page_fault_w(cpu_id, pc, arg);
            break;
        default:
            printf("CPU%d Raise an Unexpected Exception %d @0x%lx, arg:0x%lx\n", cpu_id, cause, pc, arg);
            simroot_assert(0);
        }

        if(nextpc) {
            cpus->redirect(cpu_id, nextpc);
        } else {
            cpus->halt(cpu_id);
        }
    }

    uint64_t end_tick = cpus->get_current_tick();
    printf("%ld: All cores halted, exit\n", end_tick);
    printf("Time Statistic:\n");
    printf("    Global Ticks: %ld\n", end_tick - start_tick);

    vector<uint64_t> end_uticks = start_uticks;
    uint64_t utick_sum = 0;
    for(uint32_t i = 0; i < cpu_num; i++) {
        end_uticks[i] = cpus->get_current_utick(i);
        utick_sum += (end_uticks[i] - start_uticks[i]);
    }
    printf("    User Ticks: %ld\n", utick_sum);
    printf("    UTick per Core:\n");
    for(uint32_t i = 0; i < cpu_num; i++) {
        printf("        %d: %ld\n", i, end_uticks[i] - start_uticks[i]);
    }

    if(!waiting_threads.empty()) {
        printf("Warnning: These Thread is Still WAITING:");
        for(auto t : waiting_threads) {
            printf(" %ld(%ld)", t->tid, t->tgid);
        }
        printf("\n");
    }
}


#define CURT (running_threads[cpu_id])

#define LOG_SYSCALL_1(name, f0, a0, rf0, r) if(log_syscall) {\
printf("%ld: CPU %d (T %ld) Syscall " name "(" f0 ") -> " rf0 "\n", cpus->get_current_tick(), cpu_id, CURT->tid, a0, r);\
}

#define LOG_SYSCALL_2(name, f0, a0, f1, a1, rf0, r) if(log_syscall) {\
printf("%ld: CPU %d (T %ld) Syscall " name "(" f0 ", " f1 ") -> " rf0 "\n", cpus->get_current_tick(), cpu_id, CURT->tid, a0, a1, r);\
}

#define LOG_SYSCALL_3(name, f0, a0, f1, a1, f2, a2, rf0, r) if(log_syscall) {\
printf("%ld: CPU %d (T %ld) Syscall " name "(" f0 ", " f1 ", " f2 ") -> " rf0 "\n", cpus->get_current_tick(), cpu_id, CURT->tid, a0, a1, a2, r);\
}

#define LOG_SYSCALL_4(name, f0, a0, f1, a1, f2, a2, f3, a3, rf0, r) if(log_syscall) {\
printf("%ld: CPU %d (T %ld) Syscall " name "(" f0 ", " f1 ", " f2 ", " f3 ") -> " rf0 "\n", cpus->get_current_tick(), cpu_id, CURT->tid, a0, a1, a2, a3, r);\
}

#define LOG_SYSCALL_5(name, f0, a0, f1, a1, f2, a2, f3, a3, f4, a4, rf0, r) if(log_syscall) {\
printf("%ld: CPU %d (T %ld) Syscall " name "(" f0 ", " f1 ", " f2 ", " f3 ", " f4 ") -> " rf0 "\n", cpus->get_current_tick(), cpu_id, CURT->tid, a0, a1, a2, a3, a4, r);\
}

#define LOG_SYSCALL_6(name, f0, a0, f1, a1, f2, a2, f3, a3, f4, a4, f5, a5, rf0, r) if(log_syscall) {\
printf("%ld: CPU %d (T %ld) Syscall " name "(" f0 ", " f1 ", " f2 ", " f3 ", " f4 ", " f5 ") -> " rf0 "\n", cpus->get_current_tick(), cpu_id, CURT->tid, a0, a1, a2, a3, a4, a5, r);\
}

#define IREG_V(rname) (cpus->regacc_read(cpu_id, ireg_index_of(#rname)))
#define ECALL_RET(value, pc) do { cpus->regacc_write(cpu_id, ireg_index_of("a0"), value); return (pc); } while(0)
    
SYSCALL_DEFINE_V2(17, getcwd) {
    VirtAddrT buf = IREG_V(a0);
    uint64_t len = IREG_V(a1);

    if(len == 0 || buf == 0) {
        LOG_SYSCALL_2("getcwd", "0x%lx", buf, "0x%lx", len, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }

    char retbuf[PATH_MAX];
    char* retp = getcwd(retbuf, PATH_MAX);
    uint64_t retlen = std::min<uint64_t>(strlen(retbuf) + 1, len);

    if(!_memcpy_to_target(cpu_id, buf, retp, retlen)) {
        LOG_SYSCALL_2("getcwd", "0x%lx", buf, "0x%lx", len, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    LOG_SYSCALL_2("getcwd", "0x%lx", buf, "0x%lx", len, "%s", retp);
    ECALL_RET(buf, pc+4);
}

SYSCALL_DEFINE_V2(48, faccessat) {
    int32_t dirfd = IREG_V(a0);
    VirtAddrT pathname = IREG_V(a1);
    uint32_t mode = IREG_V(a2);
    uint32_t flags = IREG_V(a3);
    int64_t ret = 0;

    char buf[PATH_MAX];
    if(!_strcpy_from_target(cpu_id, buf, pathname)) {
        LOG_SYSCALL_4("host_faccessat", "%d", dirfd, "0x%lx", pathname, "0x%x", mode, "0x%x", flags, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    FileDescriptor * fd = CURT->fdtable_trans(dirfd);
    ret = faccessat((fd?(fd->host_fd):(dirfd)), buf, mode, flags);
    if(ret < 0) {
        ret = -errno;
    }

    LOG_SYSCALL_4("host_faccessat", "%d", dirfd, "%s", buf, "0x%x", mode, "0x%x", flags, "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(56, openat) {
    int32_t dirfd = IREG_V(a0);
    VirtAddrT pathname = IREG_V(a1);
    uint32_t flags = IREG_V(a2);
    uint64_t mode = IREG_V(a3);
    int64_t ret = 0;

    char buf[PATH_MAX];
    if(!_strcpy_from_target(cpu_id, buf, pathname)) {
        LOG_SYSCALL_4("host_openat", "%d", dirfd, "0x%lx", pathname, "0x%x", flags, "0x%lx", mode, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    FileDescriptor * fd = CURT->fdtable_trans(dirfd);

    ret = openat((fd?(fd->host_fd):(dirfd)), buf, flags, mode);
    if(ret < 0) {
        ret = -errno;
        LOG_SYSCALL_4("host_openat", "%d", dirfd, "%s", buf, "0x%x", flags, "%ld", mode, "%ld", ret);
        ECALL_RET(ret, pc+4);
    }

    FileDescriptor * newfd = new FileDescriptor;
    newfd->host_fd = ret;
    newfd->path = string(buf);
    newfd->ref_cnt ++;
    newfd->usr_seek = 0;

    struct stat64 s;
    fstat64(ret, &s);
    newfd->st_size = s.st_size;

    ret = CURT->fdtable_insert(newfd);
    LOG_SYSCALL_5("host_openat", "%d", dirfd, "%s", buf, "0x%x", flags, "%ld", mode, "HostFD: %d", newfd->host_fd, "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(57, close) {
    int32_t usr_fd = IREG_V(a0);

    FileDescriptor *fd = CURT->fdtable_pop(usr_fd);
    if(fd == nullptr) {
        LOG_SYSCALL_1("close", "%d", usr_fd, "%s", "EBADF");
        ECALL_RET(-EBADF, pc+4);
    }
    fd->ref_cnt --;
    if(fd->ref_cnt == 0) {
        close(fd->host_fd);
        delete fd;
    }
    LOG_SYSCALL_1("close", "%d", usr_fd, "%d", 0);
    ECALL_RET(0, pc+4);
}

SYSCALL_DEFINE_V2(62, lseek) {
    int32_t usr_fd = IREG_V(a0);
    int64_t offset = IREG_V(a1);
    uint32_t whence = IREG_V(a2);

    int64_t offset_bak = offset;
    FileDescriptor *fd = CURT->fdtable_trans(usr_fd);
    if(fd == nullptr) {
        LOG_SYSCALL_3("lseek", "%d", usr_fd, "%ld", offset_bak, "%d", whence, "%s", "EBADF");
        ECALL_RET(-EBADF, pc+4);
    }
    switch (whence)
    {
    case SEEK_SET: break;
    case SEEK_CUR: offset = ((int64_t)(fd->usr_seek)) + offset; break;
    case SEEK_END: offset = ((int64_t)(fd->st_size)) + offset; break;
    default:
        LOG_SYSCALL_3("lseek", "%d", usr_fd, "%ld", offset_bak, "%d", whence, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }
    if(offset < 0) offset = 0;
    if(offset > fd->st_size) offset = fd->st_size;
    fd->usr_seek = offset;
    LOG_SYSCALL_3("lseek", "%d", usr_fd, "%ld", offset_bak, "%d", whence, "%ld", offset);
    ECALL_RET(offset, pc+4);
}

SYSCALL_DEFINE_V2(63, read) {
    int32_t usr_fd = IREG_V(a0);
    VirtAddrT usr_buf = IREG_V(a1);
    uint64_t count = IREG_V(a2);
    int64_t ret = 0;

    FileDescriptor * fd = CURT->fdtable_trans(usr_fd);
    if(fd == nullptr) {
        LOG_SYSCALL_3("read", "%d", usr_fd, "0x%lx", usr_buf, "0x%lx", count, "%s", "EBADF");
        ECALL_RET(-EBADF, pc+4);
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd->host_fd, &fds);
    struct timeval zerotime;
    zerotime.tv_sec = zerotime.tv_usec = 0;
    int64_t select_ret = select(fd->host_fd + 1, &fds, nullptr, nullptr, &zerotime);
    if(select_ret == 0) {
        BlockreadWaitThread wt;
        wt.thread = CURT;
        wt.sys = this;
        wt.cpuid = cpu_id;
        wt.simfd = usr_fd;
        wt.hostfd = fd->host_fd;
        wt.buf = usr_buf;
        wt.bufsz = count;

        LOG_SYSCALL_3("read", "%d", usr_fd, "0x%lx", usr_buf, "0x%lx", count, "%s", "BLOCKED");

        _push_context_stack(cpu_id, pc+4);

        sch_lock.lock();
        auto iter = blkread_wait_threads.emplace(CURT, wt).first;
        pthread_create(&wt.th, nullptr, blkread_wait_thread_function, &(iter->second));
        VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_WAIT);
        sch_lock.unlock();
    
        return nextpc;
    }
    else if(select_ret > 0) {
        vector<uint8_t> buf;
        buf.resize(count);
        lseek64(fd->host_fd, fd->usr_seek, SEEK_SET);
        ret = read(fd->host_fd, buf.data(), count);
        fd->usr_seek = lseek64(fd->host_fd, 0, SEEK_CUR);

        if(ret < 0) {
            ret = -errno;
        } else if(ret) {
            if(!_memcpy_to_target(cpu_id, usr_buf, buf.data(), ret)) {
                LOG_SYSCALL_3("read", "%d", usr_fd, "0x%lx", usr_buf, "0x%lx", count, "%s", "EFAULT");
                ECALL_RET(-EFAULT, pc+4);
            }
        }

        LOG_SYSCALL_3("read", "%d", usr_fd, "0x%lx", usr_buf, "0x%lx", count, "0x%lx", ret);
        ECALL_RET(ret, pc+4);
    }
    else {
        ret = -errno;
        LOG_SYSCALL_3("read", "%d", usr_fd, "0x%lx", usr_buf, "0x%lx", count, "%ld", ret);
        ECALL_RET(ret, pc+4);
    }
}

SYSCALL_DEFINE_V2(64, write) {
    int32_t usr_fd = IREG_V(a0);
    VirtAddrT usr_buf = IREG_V(a1);
    uint64_t count = IREG_V(a2);

    FileDescriptor * fd = CURT->fdtable_trans(usr_fd);
    if(fd == nullptr) {
        LOG_SYSCALL_3("write", "%d", usr_fd, "0x%lx", usr_buf, "0x%lx", count, "%s", "EBADF");
        ECALL_RET(-EBADF, pc+4);
    }

    char * buf = new char[count];
    if(!_memcpy_from_target(cpu_id, buf, usr_buf, count)) {
        delete[] buf;
        LOG_SYSCALL_3("write", "%d", usr_fd, "0x%lx", usr_buf, "0x%lx", count, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    lseek64(fd->host_fd, fd->usr_seek, SEEK_SET);
    int64_t ret = write(fd->host_fd, buf, count);
    fd->usr_seek = lseek64(fd->host_fd, 0, SEEK_CUR);

    if(ret < 0) {
        ret = -errno;
    }

    delete[] buf;

    LOG_SYSCALL_3("write", "%d", usr_fd, "0x%lx", usr_buf, "0x%lx", count, "0x%lx", ret);
    ECALL_RET(ret, pc+4);
}

struct _rv_timespec64 {
    uint64_t tv_sec;
    int32_t tv_nsec;
    int32_t pad0;
};

SYSCALL_DEFINE_V2(72, pselect6) {
    uint64_t nfds = IREG_V(a0);
    VirtAddrT readfds = IREG_V(a1);
    VirtAddrT writefds = IREG_V(a2);
    VirtAddrT exceptfds = IREG_V(a3);
    VirtAddrT tmo_p = IREG_V(a4);
    VirtAddrT sigmask_p = IREG_V(a5);

    if(sigmask_p) {
        printf("Warnning : pselect6 with sigmask is un-implemented\n");
    }

    SelectWaitThread wt;
    wt.sys = this;
    wt.cpuid = cpu_id;
    wt.thread = CURT;

    struct timespec tmo;
    if(!tmo_p) {
        wt.timeout = -1;
    }
    else if(!_memcpy_from_target(cpu_id, &tmo, tmo_p, sizeof(tmo))) {
        LOG_SYSCALL_6("pselect6",
            "%ld", nfds, "0x%lx", readfds,
            "0x%lx", writefds, "0x%lx", exceptfds,
            "0x%lx", tmo_p, "0x%lx", sigmask_p,
            "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    } else {
        wt.timeout = tmo.tv_nsec / 1000000L + tmo.tv_sec * 1000L;
    }

    FD_ZERO(&wt.host_readfds);
    FD_ZERO(&wt.host_writefds);
    FD_ZERO(&wt.host_exceptfds);

    wt.readfds = readfds;
    wt.writefds = writefds;
    wt.exceptfds = exceptfds;

    fd_set rbuf, wbuf, ebuf;

    if( (readfds && !_memcpy_from_target(cpu_id, &rbuf, readfds, sizeof(fd_set))) || 
        (writefds && !_memcpy_from_target(cpu_id, &wbuf, writefds, sizeof(fd_set))) || 
        (exceptfds && !_memcpy_from_target(cpu_id, &ebuf, exceptfds, sizeof(fd_set)))
    ) {
        LOG_SYSCALL_6("pselect6",
            "%ld", nfds, "0x%lx", readfds,
            "0x%lx", writefds, "0x%lx", exceptfds,
            "0x%lx", tmo_p, "0x%lx", sigmask_p,
            "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    bool invalid_fd = false;
    for(int32_t i = 0; i < nfds; i++) {
        if(readfds && FD_ISSET(i, &rbuf)) {
            FileDescriptor *hfd = CURT->fdtable_trans(i);
            if(!hfd) {
                invalid_fd = true;
                break;
            }
            wt.host_nfds = std::max<int32_t>(wt.host_nfds, hfd->host_fd + 1);
            wt.hostfd_to_simfd.emplace(hfd->host_fd, i);
            FD_SET(hfd->host_fd, &wt.host_readfds);
        }
        if(writefds && FD_ISSET(i, &wbuf)) {
            FileDescriptor *hfd = CURT->fdtable_trans(i);
            if(!hfd) {
                invalid_fd = true;
                break;
            }
            wt.host_nfds = std::max<int32_t>(wt.host_nfds, hfd->host_fd + 1);
            wt.hostfd_to_simfd.emplace(hfd->host_fd, i);
            FD_SET(hfd->host_fd, &wt.host_writefds);
        }
        if(exceptfds && FD_ISSET(i, &ebuf)) {
            FileDescriptor *hfd = CURT->fdtable_trans(i);
            if(!hfd) {
                invalid_fd = true;
                break;
            }
            wt.host_nfds = std::max<int32_t>(wt.host_nfds, hfd->host_fd + 1);
            wt.hostfd_to_simfd.emplace(hfd->host_fd, i);
            FD_SET(hfd->host_fd, &wt.host_exceptfds);
        }
    }

    if(invalid_fd) {
        LOG_SYSCALL_6("pselect6",
            "%ld", nfds, "0x%lx", readfds,
            "0x%lx", writefds, "0x%lx", exceptfds,
            "0x%lx", tmo_p, "0x%lx", sigmask_p,
            "%s", "EBADF");
        ECALL_RET(-EBADF, pc+4);
    }

    _push_context_stack(cpu_id, pc+4);

    LOG_SYSCALL_6("pselect6",
        "%ld", nfds, "0x%lx", readfds,
        "0x%lx", writefds, "0x%lx", exceptfds,
        "0x%lx", tmo_p, "0x%lx", sigmask_p,
        "%s", "BLOCKED");
    
    sch_lock.lock();
    auto iter = select_wait_threads.emplace(CURT, wt).first;
    pthread_create(&wt.th, nullptr, select_wait_thread_function, &(iter->second));
    VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_WAIT);
    sch_lock.unlock();

    return nextpc;
}

SYSCALL_DEFINE_V2(73, ppoll) {
    VirtAddrT fds = IREG_V(a0);
    uint64_t nfds = IREG_V(a1);
    VirtAddrT tmo_p = IREG_V(a2);
    VirtAddrT sigmask_p = IREG_V(a3);

    if(sigmask_p) {
        printf("Warnning : ppoll with sigmask is un-implemented\n");
    }

    PollWaitThread wt;
    wt.sys = this;
    wt.cpuid = cpu_id;
    wt.thread = CURT;
    wt.tgt_fds = fds;
    wt.host_fds.resize(nfds);

    struct timespec tmo;
    if(!tmo_p) {
        wt.timeout = -1;
    }
    else if(!_memcpy_from_target(cpu_id, &tmo, tmo_p, sizeof(tmo))) {
        LOG_SYSCALL_4("ppoll", "0x%lx", fds, "%ld", nfds, "0x%lx", tmo_p, "0x%lx", sigmask_p, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    } else {
        wt.timeout = tmo.tv_nsec / 1000000L + tmo.tv_sec * 1000L;
    }

    if(!_memcpy_from_target(cpu_id, wt.host_fds.data(), fds, nfds * sizeof(struct pollfd))) {
        LOG_SYSCALL_4("ppoll", "0x%lx", fds, "%ld", nfds, "0x%lx", tmo_p, "0x%lx", sigmask_p, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    for(uint64_t i = 0; i < nfds; i++) {
        FileDescriptor * hfd = CURT->fdtable_trans(wt.host_fds[i].fd);
        if(!hfd) {
            LOG_SYSCALL_4("ppoll", "0x%lx", fds, "%ld", nfds, "0x%lx", tmo_p, "0x%lx", sigmask_p, "%s", "EBADF");
            ECALL_RET(-EBADF, pc+4);
        }
        wt.host_fds[i].fd = hfd->host_fd;
    }

    _push_context_stack(cpu_id, pc+4);

    LOG_SYSCALL_4("ppoll", "0x%lx", fds, "%ld", nfds, "0x%lx", tmo_p, "0x%lx", sigmask_p, "%s", "BLOCKED");
    
    sch_lock.lock();
    auto iter = poll_wait_threads.emplace(CURT, wt).first;
    pthread_create(&wt.th, nullptr, poll_wait_thread_function, &(iter->second));
    VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_WAIT);
    sch_lock.unlock();

    return nextpc;
}

SYSCALL_DEFINE_V2(78, readlinkat) {
    int32_t dirfd = IREG_V(a0);
    VirtAddrT pathname = IREG_V(a1);
    VirtAddrT usr_buf = IREG_V(a2);
    uint64_t bufsiz = IREG_V(a3);

    char pathbuf[PATH_MAX];
    if(!_strcpy_from_target(cpu_id, pathbuf, pathname)) {
        LOG_SYSCALL_4("readlinkat", "%d", dirfd, "0x%lx", pathname, "0x%lx", usr_buf, "%ld", bufsiz, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    FileDescriptor * fd = CURT->fdtable_trans(dirfd);

    vector<char> buf;
    buf.resize(bufsiz);
    int64_t ret = readlinkat((fd?(fd->host_fd):dirfd), pathbuf, buf.data(), bufsiz);
    if(ret < 0) {
        ret = -errno;
    } else if(!_memcpy_to_target(cpu_id, usr_buf, buf.data(), ret)) {
        LOG_SYSCALL_4("readlinkat", "%d", dirfd, "0x%lx", pathname, "0x%lx", usr_buf, "%ld", bufsiz, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    LOG_SYSCALL_4("readlinkat", "%d", dirfd, "%s", pathbuf, "0x%lx", usr_buf, "%ld", bufsiz, "%ld", ret);
    ECALL_RET(ret, pc+4);
}

struct rv_stat {
    uint64_t    st_dev;		/* Device.  */
    uint64_t    st_ino;		/* file serial number.	*/
    uint32_t    st_mode;		/* File mode.  */
    uint32_t    st_nlink;		/* Link count.  */
    uint32_t    st_uid;		/* User ID of the file's owner.  */
    uint32_t    st_gid;		/* Group ID of the file's group.  */
    uint64_t    st_rdev;		/* Device number, if device.  */
    int64_t     st_size;		/* Size of file, in bytes.  */
    int64_t     st_blksize;	/* Optimal block size for I/O.  */
    int64_t     st_blocks;	/* Number 512-byte blocks allocated. */
    struct _rv_timespec64 st_atim;
    struct _rv_timespec64 st_mtim;
    struct _rv_timespec64 st_ctim;
};

SYSCALL_DEFINE_V2(79, newfstatat) {
    int32_t dirfd = IREG_V(a0);
    VirtAddrT pathname = IREG_V(a1);
    VirtAddrT usr_buf = IREG_V(a2);
    uint64_t flags = IREG_V(a3);

    char pathbuf[PATH_MAX];
    if(!_strcpy_from_target(cpu_id, pathbuf, pathname)) {
        LOG_SYSCALL_4("newfstatat", "%d", dirfd, "0x%lx", pathname, "0x%lx", usr_buf, "%ld", flags, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    FileDescriptor * fd = CURT->fdtable_trans(dirfd);

    struct rv_stat buf;
    struct stat host_buf;
    int64_t ret = fstatat((fd?(fd->host_fd):dirfd), pathbuf, &host_buf, flags);
    buf.st_dev = host_buf.st_dev;
    buf.st_ino = host_buf.st_ino;
    buf.st_mode = host_buf.st_mode;
    buf.st_nlink = host_buf.st_nlink;
    buf.st_uid = host_buf.st_uid;
    buf.st_gid = host_buf.st_gid;
    buf.st_rdev = host_buf.st_rdev;
    buf.st_size = host_buf.st_size;
    buf.st_blksize = host_buf.st_blksize;
    buf.st_blocks = host_buf.st_blocks;
    buf.st_atim.tv_sec = host_buf.st_atim.tv_sec;
    buf.st_atim.tv_nsec = host_buf.st_atim.tv_nsec;
    buf.st_mtim.tv_sec = host_buf.st_mtim.tv_sec;
    buf.st_mtim.tv_nsec = host_buf.st_mtim.tv_nsec;
    buf.st_ctim.tv_sec = host_buf.st_ctim.tv_sec;
    buf.st_ctim.tv_nsec = host_buf.st_ctim.tv_nsec;

    if(ret < 0) {
        ret = -errno;
    } else if(!_memcpy_to_target(cpu_id, usr_buf, &host_buf, sizeof(struct rv_stat))) {
        LOG_SYSCALL_4("newfstatat", "%d", dirfd, "0x%lx", pathname, "0x%lx", usr_buf, "%ld", flags, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    LOG_SYSCALL_4("newfstatat", "%d", dirfd, "%s", pathbuf, "0x%lx", usr_buf, "%ld", flags, "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(80, fstat) {
    int32_t dirfd = IREG_V(a0);
    VirtAddrT usr_buf = IREG_V(a1);

    FileDescriptor * fd = CURT->fdtable_trans(dirfd);
    if(!fd) {
        LOG_SYSCALL_2("fstat", "%d", dirfd, "0x%lx", usr_buf, "%s", "EBADF");
        ECALL_RET(-EBADF, pc+4);
    }

    struct rv_stat buf;
    struct stat host_buf;
    int64_t ret = fstat(fd->host_fd, &host_buf);
    buf.st_dev = host_buf.st_dev;
    buf.st_ino = host_buf.st_ino;
    buf.st_mode = host_buf.st_mode;
    buf.st_nlink = host_buf.st_nlink;
    buf.st_uid = host_buf.st_uid;
    buf.st_gid = host_buf.st_gid;
    buf.st_rdev = host_buf.st_rdev;
    buf.st_size = host_buf.st_size;
    buf.st_blksize = host_buf.st_blksize;
    buf.st_blocks = host_buf.st_blocks;
    buf.st_atim.tv_sec = host_buf.st_atim.tv_sec;
    buf.st_atim.tv_nsec = host_buf.st_atim.tv_nsec;
    buf.st_mtim.tv_sec = host_buf.st_mtim.tv_sec;
    buf.st_mtim.tv_nsec = host_buf.st_mtim.tv_nsec;
    buf.st_ctim.tv_sec = host_buf.st_ctim.tv_sec;
    buf.st_ctim.tv_nsec = host_buf.st_ctim.tv_nsec;

    if(ret < 0) {
        ret = -errno;
    } else if(!_memcpy_to_target(cpu_id, usr_buf, &host_buf, sizeof(struct rv_stat))) {
        LOG_SYSCALL_2("fstat", "%d", dirfd, "0x%lx", usr_buf, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    LOG_SYSCALL_2("fstat", "%d", dirfd, "0x%lx", usr_buf, "%ld", ret);
    ECALL_RET(ret, pc+4);
}


SYSCALL_DEFINE_V2(93, exit) {
    int32_t status = IREG_V(a0);

    LOG_SYSCALL_1("exit", "%d", status, "%d", 0);

    thread_exit_codes.emplace(CURT->tid, status);

    sch_lock.lock();

    // Do clear_child_tid
    if(CURT->do_child_cleartid && CURT->clear_child_tid) {
        uint32_t tmp = 0;
        _memcpy_to_target(cpu_id, CURT->clear_child_tid, &tmp, sizeof(tmp));

        PTET pte = CURT->pgtable->pt_get((CURT->clear_child_tid) >> PAGE_ADDR_OFFSET, nullptr);
        if(pte & PTE_V) {
            PhysAddrT paddr = ((pte >> 10) << PAGE_ADDR_OFFSET) + ((CURT->clear_child_tid) & (PAGE_LEN_BYTE - 1));
            auto res = futex_wait_threads.find(paddr);
            if(res != futex_wait_threads.end()) {
                auto &tl = res->second;
                for(auto &fwt : tl) {
                    fwt.thread->context_stack.back()[ireg_index_of("a0")] = 0;
                    insert_ready_thread_and_execute(fwt.thread, fwt.last_cpu_id);
                }
                futex_wait_threads.erase(res);
            }
        }
    }

    thread_objs.erase(CURT->tid);
    thread_exit_codes.emplace(CURT->tid, status);
    wake_up_wait_threads(CURT->tid, CURT->tgid, CURT->parent, status);

    delete CURT;
    CURT = nullptr;

    VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_EXIT);

    sch_lock.unlock();

    return nextpc;
}

SYSCALL_DEFINE_V2(94, exitgroup) {

    int32_t status = IREG_V(a0);

    LOG_SYSCALL_1("exitgroup", "%d", status, "%d", 0);

    thread_exit_codes.emplace(CURT->tid, status);

    sch_lock.lock();
    
    // Terminate all threads in same group
    TgtTGidT tgid = CURT->tgid;
    vector<TgtTidT> allthread = thread_groups[tgid];
    
    for(TgtTidT tid : allthread) {
        auto iter1 = thread_objs.find(tid);
        if(iter1 == thread_objs.end()) continue;
        ThreadV2 *child = iter1->second;

        if(child->do_child_cleartid && child->clear_child_tid) {
            uint32_t tmp = 0;
            PTET pte = child->pgtable->pt_get((child->clear_child_tid) >> PAGE_ADDR_OFFSET, nullptr);
            if(pte & PTE_V) {
                PhysAddrT paddr = ((pte >> 10) << PAGE_ADDR_OFFSET) + ((child->clear_child_tid) & (PAGE_LEN_BYTE - 1));
                RawDataT v = cpus->pxymem_read(cpu_id, paddr & (~7UL));
                if(paddr & 4) {
                    v = ((v << 32) >> 32);
                } else {
                    v = ((v >> 32) << 32);
                }
                cpus->pxymem_write(cpu_id, paddr & (~7UL), v);
                auto res = futex_wait_threads.find(paddr);
                if(res != futex_wait_threads.end()) {
                    auto &tl = res->second;
                    for(auto &fwt : tl) {
                        fwt.thread->context_stack.back()[ireg_index_of("a0")] = 0;
                        if(fwt.thread->tgid != tgid) {
                            insert_ready_thread_and_execute(fwt.thread, fwt.last_cpu_id);
                        }
                    }
                    futex_wait_threads.erase(res);
                }
            }
        }

        cancle_wait_thread_nolock(child);

        for(auto iter = ready_threads.begin(); iter != ready_threads.end(); ) {
            if(*iter == child) iter = ready_threads.erase(iter);
            else iter++;
        }
        // std::remove(ready_threads.begin(), ready_threads.end(), child);

        thread_objs.erase(child->tid);
        thread_exit_codes.emplace(child->tid, status);
        wake_up_wait_threads(child->tid, child->tgid, child->parent, status);

        for(uint32_t i = 0; i < running_threads.size(); i++) {
            if(running_threads[i] == child) {
                running_threads[i] = nullptr;
                cpus->halt(i);
            }
        }
        printf("CPU%d : Cancle Thread %ld\n", cpu_id, child->tid);
        delete child;
    }

    VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_EXIT);

    sch_lock.unlock();

    return nextpc;
}

SYSCALL_DEFINE_V2(96, set_tid_address) {
    uint64_t addr = IREG_V(a0);

    CURT->clear_child_tid = addr;
    LOG_SYSCALL_1("set_tid_address", "0x%lx", addr, "%ld", CURT->tid);
    ECALL_RET(CURT->tid, pc+4);
}

SYSCALL_DEFINE_V2(98, futex) {
    VirtAddrT uaddr = IREG_V(a0);
    int32_t futex_op = IREG_V(a1);
    uint32_t val = IREG_V(a2);
    VirtAddrT timeout = IREG_V(a3);
    uint32_t val2 = IREG_V(a3);
    VirtAddrT uaddr2 = IREG_V(a4);
    uint32_t val3 = IREG_V(a5);

    futex_op &= 127;

    PhysAddrT paddr = 0, paddr2 = 0;
    {
        PTET pte = CURT->pgtable->pt_get(uaddr >> PAGE_ADDR_OFFSET, nullptr);
        if(!(pte & PTE_V)) {
            LOG_SYSCALL_6("futex", "0x%lx", uaddr, "%d", futex_op, "0x%x", val, "0x%lx", uaddr2, "0x%x", val2, "0x%x", val3, "%s", "EFAULT");
            ECALL_RET(-EFAULT, pc+4);
        }
        paddr = ((pte >> 10) << PAGE_ADDR_OFFSET) | (uaddr & (PAGE_LEN_BYTE - 1));
    }
    if(uaddr2) {
        PTET pte = CURT->pgtable->pt_get(uaddr2 >> PAGE_ADDR_OFFSET, nullptr);
        if(pte & PTE_V) {
            paddr2 = ((pte >> 10) << PAGE_ADDR_OFFSET) | (uaddr2 & (PAGE_LEN_BYTE - 1));
        }
    }

    if(futex_op == FUTEX_WAIT || futex_op == FUTEX_WAIT_BITSET) {
        RawDataT _v = cpus->pxymem_read(cpu_id, paddr & (~7UL));
        uint32_t v = ((paddr & 4) ? (_v >> 32) : _v);
        if(v != val) {
            LOG_SYSCALL_6("futex", "0x%lx", uaddr, "%d", futex_op, "0x%x", val, "0x%lx", uaddr2, "0x%x", val2, "0x%x", val3, "%s", "EAGAIN");
            ECALL_RET(-EAGAIN, pc+4);
        }
        LOG_SYSCALL_6("futex", "0x%lx", uaddr, "%d", futex_op, "0x%x", val, "0x%lx", uaddr2, "0x%x", val2, "0x%x", val3, "%s", "WAIT");
        _push_context_stack(cpu_id, pc+4);
        sch_lock.lock();
        futex_wait_thread_insert(paddr, CURT, (futex_op == FUTEX_WAIT_BITSET)?(val3):0, cpu_id);
        VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_WAIT);
        sch_lock.unlock();
        return nextpc;
    }
    else if(futex_op == FUTEX_WAKE || futex_op == FUTEX_WAKE_BITSET) {
        FutexWaitThread buf;
        uint32_t futex_mask = ((futex_op == FUTEX_WAKE_BITSET)?(val3):0);
        uint64_t wake_cnt = 0;
        sch_lock.lock();
        for(wake_cnt = 0; wake_cnt < val; wake_cnt++) {
            if(!futex_wait_thread_pop(paddr, futex_mask, &buf)) {
                break;
            }
            insert_ready_thread_and_execute(buf.thread, buf.last_cpu_id);
        }
        sch_lock.unlock();
        LOG_SYSCALL_6("futex", "0x%lx", uaddr, "%d", futex_op, "0x%x", val, "0x%lx", uaddr2, "0x%x", val2, "0x%x", val3, "%ld", wake_cnt);
        ECALL_RET(wake_cnt, pc+4);
    }

    printf("Unknown futex op %d\n", futex_op);
    simroot_assert(0);
}

SYSCALL_DEFINE_V2(99, set_robust_list) {
    LOG_SYSCALL_2("set_robust_list", "%ld", IREG_V(a0), "%ld", IREG_V(a1), "%d", -1);
    ECALL_RET(-1, pc+4);
}

SYSCALL_DEFINE_V2(113, clock_gettime) {
    uint64_t clockid = IREG_V(a0);
    VirtAddrT usr_tp = IREG_V(a1);

    struct timespec t;
    int64_t ret = clock_gettime(clockid, &t);
    if(ret < 0) {
        ret = -errno;
    } else if(!_memcpy_to_target(cpu_id, usr_tp, &t, sizeof(struct timespec))) {
        LOG_SYSCALL_2("clock_gettime", "%ld", clockid, "0x%lx", usr_tp, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    LOG_SYSCALL_2("clock_gettime", "%ld", clockid, "0x%lx", usr_tp, "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(115, clock_nanosleep) {
    uint64_t clockid = IREG_V(a0);
    uint64_t flags = IREG_V(a1);
    VirtAddrT t = IREG_V(a2);
    VirtAddrT remain = IREG_V(a3);

    SleepWaitThread wt;
    wt.thread = CURT;
    wt.sys = this;
    wt.cpuid = cpu_id;

    if(!_memcpy_from_target(cpu_id, &wt.host_time, t, sizeof(wt.host_time))) {
        LOG_SYSCALL_3("clock_nanosleep", "%ld", clockid, "0x%lx", flags, "0x%lx", t, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    _push_context_stack(cpu_id, pc+4);

    LOG_SYSCALL_3("clock_nanosleep", "%ld", clockid, "0x%lx", flags, "0x%lx", t, "%s", "BLOCKED");
    
    sch_lock.lock();
    auto iter = sleep_wait_threads.emplace(CURT, wt).first;
    pthread_create(&wt.th, nullptr, sleep_wait_thread_function, &(iter->second));
    VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_WAIT);
    sch_lock.unlock();

    return nextpc;
}

SYSCALL_DEFINE_V2(124, sched_yield) {
    //TODO
    LOG_SYSCALL_1("sched_yield", "%ld", IREG_V(a0), "%ld", 0UL);
    ECALL_RET(0, pc+4);
}

SYSCALL_DEFINE_V2(134, sigaction) {
    int32_t signum = IREG_V(a0);
    VirtAddrT act = IREG_V(a1);
    VirtAddrT oldact = IREG_V(a2);
    
    KernelSigaction act_buf, old_buf;

    if(act) {
        if(!_memcpy_from_target(cpu_id, &act_buf, act, sizeof(KernelSigaction))) {
            LOG_SYSCALL_3("sigaction", "%d", signum, "0x%lx", act, "0x%lx", oldact, "%s", "EFAULT");
            ECALL_RET(-EFAULT, pc+4);
        }
    }
    auto iter = CURT->sig_actions->find(signum);
    if(oldact) {
        if(iter == CURT->sig_actions->end()) {
            memset(&old_buf, 0, sizeof(KernelSigaction));
        } else {
            memcpy(&old_buf, &(iter->second), sizeof(KernelSigaction));
        }
        if(!_memcpy_to_target(cpu_id, oldact, &old_buf, sizeof(KernelSigaction))) {
            LOG_SYSCALL_3("sigaction", "%d", signum, "0x%lx", act, "0x%lx", oldact, "%s", "EFAULT");
            ECALL_RET(-EFAULT, pc+4);
        }
    }
    if(act) {
        if(iter == CURT->sig_actions->end()) {
            CURT->sig_actions->emplace(signum, act_buf);
        } else {
            memcpy(&(iter->second), &act_buf, sizeof(KernelSigaction));
        }
    }

    LOG_SYSCALL_3("sigaction", "%d", signum, "0x%lx", act, "0x%lx", oldact, "%d", 0);
    ECALL_RET(0, pc+4);
}

SYSCALL_DEFINE_V2(135, sigprocmask) {
    int32_t how = IREG_V(a0);
    VirtAddrT set = IREG_V(a1);
    VirtAddrT oldset = IREG_V(a2);

    TgtSigsetT buf;
    
    if(set) {
        if(!_memcpy_from_target(cpu_id, &buf, set, sizeof(TgtSigsetT))) {
            LOG_SYSCALL_3("sigprocmask", "%d", how, "0x%lx", set, "0x%lx", oldset, "%s", "EFAULT");
            ECALL_RET(-EFAULT, pc+4);
        }
    }
    if(oldset) {
        if(!_memcpy_to_target(cpu_id, oldset, CURT->sig_proc_mask.get(), sizeof(TgtSigsetT))) {
            LOG_SYSCALL_3("sigprocmask", "%d", how, "0x%lx", set, "0x%lx", oldset, "%s", "EFAULT");
            ECALL_RET(-EFAULT, pc+4);
        }
    }
    if(set) {
        if(how == SIG_BLOCK) {
            for(int sn = 0; sn < 64; sn++) {
                if(sigismember(&buf, sn) > 0) sigaddset(CURT->sig_proc_mask.get(), sn);
            }
        }
        else if(how == SIG_UNBLOCK) {
            for(int sn = 0; sn < 64; sn++) {
                if(sigismember(&buf, sn) > 0) sigdelset(CURT->sig_proc_mask.get(), sn);
            }
        }
        else if(how == SIG_SETMASK) {
            memcpy(CURT->sig_proc_mask.get(), &buf, sizeof(TgtSigsetT));
        }
        else {
            LOG_SYSCALL_3("sigprocmask", "%d", how, "0x%lx", set, "0x%lx", oldset, "%s", "EINVAL");
            ECALL_RET(-EINVAL, pc+4);
        }
    }
    
    LOG_SYSCALL_3("sigprocmask", "%d", how, "0x%lx", set, "0x%lx", oldset, "%d", 0);
    ECALL_RET(0, pc+4);
}

char uts_sysname[65] = "Linux";
char uts_nodename[65] = "fedora-riscv-2-8";
char uts_release[65] = "6.1.31";
char uts_version[65] = "#1 SMP Sun Oct 22 00:58:22 CST 2023";
char uts_machine[65] = "riscv64";
char uts_domainname[65] = "GNU/Linux";

struct target_utsname {
    char sysname[65];    /* Operating system name (e.g., "Linux") */
    char nodename[65];   /* Name within communications network
                            to which the node is attached, if any */
    char release[65];    /* Operating system release
                            (e.g., "2.6.28") */
    char version[65];    /* Operating system version */
    char machine[65];    /* Hardware type identifier */
    char domainname[65]; /* NIS or YP domain name */
};

SYSCALL_DEFINE_V2(160, uname) {
    VirtAddrT usr_buf = IREG_V(a0);
    
    struct target_utsname buf;
    strcpy(buf.sysname, uts_sysname);
    strcpy(buf.nodename, uts_nodename);
    strcpy(buf.release, uts_release);
    strcpy(buf.version, uts_version);
    strcpy(buf.machine, uts_machine);
    strcpy(buf.domainname, uts_domainname);

    if(!_memcpy_to_target(cpu_id, usr_buf, &buf, sizeof(buf))) {
        LOG_SYSCALL_1("uname", "0x%lx", usr_buf, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }
    
    LOG_SYSCALL_1("uname", "0x%lx", usr_buf, "%d", 0);
    ECALL_RET(0, pc+4);
}


SYSCALL_DEFINE_V2(172, getpid) {
    int64_t ret = CURT->tgid;
    LOG_SYSCALL_1("getpid", "%ld", IREG_V(a0), "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(173, getppid) {
    auto iter = thread_objs.find(CURT->parent);
    int64_t ret = ((iter != thread_objs.end())?(iter->second->tgid):1);
    LOG_SYSCALL_1("getppid", "%ld", IREG_V(a0), "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(174, getuid) {
    int64_t ret = getuid();
    LOG_SYSCALL_1("getuid", "%ld", IREG_V(a0), "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(175, geteuid) {
    int64_t ret = geteuid();
    LOG_SYSCALL_1("geteuid", "%ld", IREG_V(a0), "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(176, getgid) {
    int64_t ret = getgid();
    LOG_SYSCALL_1("getgid", "%ld", IREG_V(a0), "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(177, getegid) {
    int64_t ret = getegid();
    LOG_SYSCALL_1("getegid", "%ld", IREG_V(a0), "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(178, gettid) {
    int64_t ret = CURT->tid;
    LOG_SYSCALL_1("gettid", "%ld", IREG_V(a0), "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(206, sendto) {
    int32_t usr_fd = IREG_V(a0);
    VirtAddrT usr_buf = IREG_V(a1);
    uint64_t size = IREG_V(a2);
    uint64_t flags = IREG_V(a3);
    VirtAddrT destaddr = IREG_V(a4);
    uint64_t addrlen = IREG_V(a5);

    SockSendWaitThread wt;
    wt.sys = this;
    wt.cpuid = cpu_id;
    wt.thread = CURT;

    FileDescriptor * fd = CURT->fdtable_trans(usr_fd);
    if(fd == nullptr) {
        LOG_SYSCALL_6("sendto", "%d", usr_fd,
            "0x%lx", usr_buf, "0x%lx", size,
            "0x%lx", flags, "0x%lx", destaddr,
            "%ld", addrlen, "%s", "EBADF"
        );
        ECALL_RET(-EBADF, pc+4);
    }
    
    wt.simfd = usr_fd;
    wt.hostfd = fd->host_fd;
    wt.flags = flags;
    wt.buf = usr_buf;
    wt.size = size;
    wt.addr.assign(addrlen, 0);

    if(!_memcpy_from_target(cpu_id, wt.addr.data(), destaddr, addrlen)) {
        LOG_SYSCALL_6("sendto", "%d", usr_fd,
            "0x%lx", usr_buf, "0x%lx", size,
            "0x%lx", flags, "0x%lx", destaddr,
            "%ld", addrlen, "%s", "EFAULT"
        );
        ECALL_RET(-EFAULT, pc+4);
    }
    
    _push_context_stack(cpu_id, pc+4);

    LOG_SYSCALL_6("sendto", "%d", usr_fd,
        "0x%lx", usr_buf, "0x%lx", size,
        "0x%lx", flags, "0x%lx", destaddr,
        "%ld", addrlen, "%s", "BLOCKED");
    
    sch_lock.lock();
    auto iter = socksend_wait_threads.emplace(CURT, wt).first;
    pthread_create(&wt.th, nullptr, socksend_wait_thread_function, &(iter->second));
    VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_WAIT);
    sch_lock.unlock();

    return nextpc;
}

SYSCALL_DEFINE_V2(212, recvmsg) {
    int32_t usr_fd = IREG_V(a0);
    VirtAddrT usr_msg = IREG_V(a1);
    uint64_t flags = IREG_V(a2);

    SockRecvMsgWaitThread wt;
    wt.sys = this;
    wt.cpuid = cpu_id;
    wt.thread = CURT;

    FileDescriptor * fd = CURT->fdtable_trans(usr_fd);
    if(fd == nullptr) {
        LOG_SYSCALL_3("recvmsg", "%d", usr_fd, "0x%lx", usr_msg, "0x%lx", flags, "%s", "EBADF");
        ECALL_RET(-EBADF, pc+4);
    }
    
    wt.simfd = usr_fd;
    wt.hostfd = fd->host_fd;
    wt.flags = flags;
    wt.tgt_msg_hdr = usr_msg;

    if(!_memcpy_from_target(cpu_id, &(wt.tgt_msg), usr_msg, sizeof(struct msghdr))) {
        LOG_SYSCALL_3("recvmsg", "%d", usr_fd, "0x%lx", usr_msg, "0x%lx", flags, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    if(wt.tgt_msg.msg_controllen) {
        wt.msg_control.assign(wt.tgt_msg.msg_controllen, 0);
        if(!_memcpy_from_target(cpu_id, wt.msg_control.data(), (VirtAddrT)(wt.tgt_msg.msg_control), wt.msg_control.size())) {
            LOG_SYSCALL_3("recvmsg", "%d", usr_fd, "0x%lx", usr_msg, "0x%lx", flags, "%s", "EFAULT");
            ECALL_RET(-EFAULT, pc+4);
        }
    } else if(wt.tgt_msg.msg_control) {
        LOG_SYSCALL_3("recvmsg", "%d", usr_fd, "0x%lx", usr_msg, "0x%lx", flags, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }

    if(wt.tgt_msg.msg_namelen) {
        wt.msg_name.assign(wt.tgt_msg.msg_namelen, 0);
        if(!_memcpy_from_target(cpu_id, wt.msg_name.data(), (VirtAddrT)(wt.tgt_msg.msg_name), wt.msg_name.size())) {
            LOG_SYSCALL_3("recvmsg", "%d", usr_fd, "0x%lx", usr_msg, "0x%lx", flags, "%s", "EFAULT");
            ECALL_RET(-EFAULT, pc+4);
        }
    } else if(wt.tgt_msg.msg_name) {
        LOG_SYSCALL_3("recvmsg", "%d", usr_fd, "0x%lx", usr_msg, "0x%lx", flags, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }

    wt.tgt_iovecs.resize(wt.tgt_msg.msg_iovlen);
    if(wt.tgt_msg.msg_iovlen && !_memcpy_from_target(cpu_id, wt.tgt_iovecs.data(), (VirtAddrT)(wt.tgt_msg.msg_iov), wt.tgt_msg.msg_iovlen * sizeof(struct iovec))) {
        LOG_SYSCALL_3("recvmsg", "%d", usr_fd, "0x%lx", usr_msg, "0x%lx", flags, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }

    _push_context_stack(cpu_id, pc+4);

    LOG_SYSCALL_3("recvmsg", "%d", usr_fd, "0x%lx", usr_msg, "0x%lx", flags, "%s", "BLOCKED");
    
    sch_lock.lock();
    auto iter = sockrecvmsg_wait_threads.emplace(CURT, wt).first;
    pthread_create(&wt.th, nullptr, sockrecvmsg_wait_thread_function, &(iter->second));
    VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_WAIT);
    sch_lock.unlock();

    return nextpc;
}

SYSCALL_DEFINE_V2(214, brk) {
    uint64_t arg0 = IREG_V(a0);

    TgtMemSetList stlist;
    VirtAddrT ret = CURT->pgtable->alloc_brk(arg0, &stlist);
    if(!stlist.empty()) {
        for(auto &st : stlist) _perform_target_memset(cpu_id, st); 
        cpus->flush_tlb_all(cpu_id);
    }
    
    LOG_SYSCALL_1("brk", "0x%lx", arg0, "0x%lx", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(215, munmap) {
    VirtAddrT vaddr = IREG_V(a0);
    uint64_t length = IREG_V(a1);

    if((vaddr % PAGE_LEN_BYTE) || !length) {
        LOG_SYSCALL_2("munmap", "0x%lx", vaddr, "0x%lx", length, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }

    vector<PageIndexT> page_to_read;
    CURT->pgtable->msync_get_ppns(vaddr, length, &page_to_read);
    if(!page_to_read.empty()) {
        unordered_map<PageIndexT, vector<RawDataT>> page_to_writeback;
        for(auto ppn : page_to_read) {
            auto iter = page_to_writeback.emplace(ppn, vector<RawDataT>()).first;
            auto &v = iter->second;
            v.assign(PAGE_LEN_BYTE/8, 0);
            cpus->pxymem_page_read(cpu_id, ppn, v.data());
        }
        CURT->pgtable->msync_writeback(vaddr, length, &page_to_writeback);
    }

    TgtMemSetList stlist;
    CURT->pgtable->free_mmap(vaddr, length, &stlist);
    if(!stlist.empty()) {
        for(auto &st : stlist) _perform_target_memset(cpu_id, st); 
        cpus->flush_tlb_all(cpu_id);
    }

    LOG_SYSCALL_2("munmap", "0x%lx", IREG_V(a0), "0x%lx", IREG_V(a1), "%ld", 0UL);
    ECALL_RET(0, pc+4);
}

SYSCALL_DEFINE_V2(220, clone) {
    uint64_t clone_flags = IREG_V(a0);
    VirtAddrT newsp = IREG_V(a1);
    VirtAddrT parent_tid = IREG_V(a2);
    VirtAddrT tls = IREG_V(a3);
    VirtAddrT child_tid = IREG_V(a4);

    // printf("PGTABLE before CLONE:\n");
    // CURT->pgtable->debug_print_pgtable();

    _push_context_stack(cpu_id, pc+4);

    sch_lock.lock();

    VirtAddrT ret = cur_tid_alloc++;
    AsidT asid = cur_asid_alloc++;

    simroot_assertf(asid < MAX_ASID, "ASID run out");

    TgtMemSetList stlist;
    ThreadV2 *newthread = new ThreadV2(CURT, ret, clone_flags, &stlist);
    newthread->asid = asid;
    thread_objs.emplace(newthread->tid, newthread);

    newthread->clear_child_tid = child_tid;
    if(tls) {
        newthread->context_stack.back()[ireg_index_of("tp")] = tls;
    }
    if(newsp) {
        newthread->context_stack.back()[ireg_index_of("sp")] = newsp;
    }
    newthread->context_stack.back()[ireg_index_of("a0")] = 0;
    if(clone_flags & CLONE_CHILD_SETTID) {
        newthread->set_child_tid = child_tid;
        TgtVMemSet vst;
        vst.vaddr = child_tid;
        vst.data.assign(sizeof(ret), 0);
        memcpy(vst.data.data(), &ret, sizeof(ret));
        newthread->stlist_on_ready.emplace_back(vst);
    }
    if(clone_flags & CLONE_PARENT_SETTID) {
        _memcpy_to_target(cpu_id, parent_tid, &ret, sizeof(ret));
    }
    if(clone_flags & CLONE_CHILD_CLEARTID) {
        newthread->do_child_cleartid = true;
        newthread->clear_child_tid = child_tid;
    }
    if(clone_flags & CLONE_PARENT) {
        newthread->parent = CURT->parent;
    } else {
        newthread->parent = CURT->tid;
    }
    if(newthread->parent) {
        auto iter = thread_objs.find(newthread->parent);
        if(iter != thread_objs.end()) iter->second->childs.insert(newthread->tid);
    }
    if(clone_flags & CLONE_THREAD) {
        newthread->tgid = CURT->tgid;
    } else {
        newthread->tgid = newthread->tid;
    }
    {
        auto iter = thread_groups.find(newthread->tgid);
        if(iter == thread_groups.end()) iter = thread_groups.emplace(newthread->tgid, vector<uint64_t>()).first;
        iter->second.push_back(newthread->tid);
    }

    for(auto & st : stlist) {
        _perform_target_memset(cpu_id, st);
    }
    if(!stlist.empty()) {
        cpus->flush_tlb_all(cpu_id);
    }

    insert_ready_thread_and_execute(newthread, cpu_id);

    sch_lock.unlock();

    // printf("\nPGTABLE 1 after CLONE:\n");
    // CURT->pgtable->debug_print_pgtable();
    // printf("\nPGTABLE 2 after CLONE:\n");
    // newthread->pgtable->debug_print_pgtable();

    CURT->context_stack.pop_back();

    LOG_SYSCALL_5("clone", "0x%lx", clone_flags, "0x%lx", newsp, "0x%lx", parent_tid, "0x%lx", tls, "0x%lx", child_tid, "%ld", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(222, mmap) {
    VirtAddrT vaddr = IREG_V(a0);
    uint64_t length = IREG_V(a1);
    uint64_t prot = IREG_V(a2);
    uint64_t flags = IREG_V(a3);
    int32_t usr_fd = IREG_V(a4);
    uint64_t offset = IREG_V(a5);
    VirtAddrT ret = 0;
    string info = "mmap";

    if((vaddr % PAGE_LEN_BYTE) || (offset % PAGE_LEN_BYTE) || !length) {
        LOG_SYSCALL_6("mmap", "0x%lx", vaddr, "0x%lx", length, "0x%lx", prot, "0x%lx", flags, "%d", usr_fd, "0x%lx", offset, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }

    PageFlagT pgflg = 0;
    if(prot & PROT_EXEC) pgflg |= PGFLAG_X;
    if(prot & PROT_READ) pgflg |= PGFLAG_R;
    if(prot & PROT_WRITE) pgflg |= PGFLAG_W;

    if(flags & MAP_SHARED) pgflg |= PGFLAG_SHARE;
    if(flags & MAP_PRIVATE) pgflg |= PGFLAG_PRIV;

    if((flags & MAP_ANONYMOUS) || (flags & MAP_ANON) || (usr_fd < 0)) pgflg |= PGFLAG_ANON;

    if( ((pgflg & PGFLAG_SHARE) && (pgflg & PGFLAG_PRIV)) ||
        (!(pgflg & PGFLAG_SHARE) && !(pgflg & PGFLAG_PRIV)) ||
        ((pgflg & PGFLAG_ANON) && (usr_fd > 0))
    ) {
        LOG_SYSCALL_6("mmap", "0x%lx", vaddr, "0x%lx", length, "0x%lx", prot, "0x%lx", flags, "%d", usr_fd, "0x%lx", offset, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }

    FileDescriptor * fd = nullptr;
    if(!(pgflg & PGFLAG_ANON)) {
        fd = CURT->fdtable_trans(usr_fd);
        if(fd == nullptr || fd->host_fd < 3) {
            LOG_SYSCALL_6("mmap", "0x%lx", vaddr, "0x%lx", length, "0x%lx", prot, "0x%lx", flags, "%d", usr_fd, "0x%lx", offset, "%s", "EBADF");
            ECALL_RET(-EBADF, pc+4);
        }
        if(offset >= fd->st_size) {
            LOG_SYSCALL_6("mmap", "0x%lx", vaddr, "0x%lx", length, "0x%lx", prot, "0x%lx", flags, "%d", usr_fd, "0x%lx", offset, "%s", "EINVAL");
            ECALL_RET(-EINVAL, pc+4);
        }
        info = fd->path;
    }
    if((pgflg & PGFLAG_ANON) && (pgflg & PGFLAG_SHARE)) {
        fd = new FileDescriptor;
        fd->host_fd = 0;
        fd->path = "shm";
        fd->ref_cnt = 1;
        fd->st_size = length;
        fd->usr_seek = 0;
        info = "shm";
    }

    if(flags & MAP_STACK) {
        pgflg |= PGFLAG_STACK;
        info = "stack";
    }

    TgtMemSetList stlist;
    if(flags & MAP_FIXED) {
        ret = CURT->pgtable->alloc_mmap_fixed(vaddr, length, pgflg, fd, offset, info, &stlist);
    } else {
        ret = CURT->pgtable->alloc_mmap(length, pgflg, fd, offset, info, &stlist);
    }

    if(!stlist.empty()) {
        for(auto &st : stlist) _perform_target_memset(cpu_id, st); 
        cpus->flush_tlb_all(cpu_id);
    }

    LOG_SYSCALL_6("mmap", "0x%lx", vaddr, "0x%lx", length, "0x%lx", prot, "0x%lx", flags, "%d", usr_fd, "0x%lx", offset, "0x%lx", ret);
    ECALL_RET(ret, pc+4);
}

SYSCALL_DEFINE_V2(226, mprotect) {
    VirtAddrT vaddr = IREG_V(a0);
    uint64_t length = IREG_V(a1);
    uint64_t prot = IREG_V(a2);

    if((vaddr % PAGE_LEN_BYTE) || (length % PAGE_LEN_BYTE)) {
        LOG_SYSCALL_3("mprotect", "0x%lx", vaddr, "0x%lx", length, "0x%lx", prot, "%s", "EINVAL");
        ECALL_RET(-EINVAL, pc+4);
    }

    TgtMemSetList stlist;
    
    CURT->pgtable->mprotect(vaddr, length, prot, &stlist);

    if(!stlist.empty()) {
        for(auto &st : stlist) _perform_target_memset(cpu_id, st); 
        cpus->flush_tlb_all(cpu_id);
    }

    LOG_SYSCALL_3("mprotect", "0x%lx", vaddr, "0x%lx", length, "0x%lx", prot, "%ld", 0UL);
    ECALL_RET(0, pc+4);
}

SYSCALL_DEFINE_V2(233, madvise) {
    LOG_SYSCALL_3("madvise", "0x%lx", IREG_V(a0), "%ld", IREG_V(a1), "%ld", IREG_V(a2), "%ld", 0UL);
    ECALL_RET(0, pc+4);
}

SYSCALL_DEFINE_V2(260, wait4) {
    int64_t pid = IREG_V(a0);
    VirtAddrT wstatus = IREG_V(a1);
    uint64_t options = IREG_V(a2);
    VirtAddrT rusage = IREG_V(a3);

    if(options || rusage) {
        printf("Warnning: options 0x%lx and rusage 0x%lx in wait4 is omitted\n", options, rusage);
    }

    sch_lock.lock();

    ExitThreadToBeWaited buf;
    if(try_wait(cpu_id, pid, &buf)) {
        int64_t ret = buf.tid;
        if(!_memcpy_to_target(cpu_id, wstatus, &buf.status, sizeof(buf.status))) {
            LOG_SYSCALL_2("wait4", "%ld", pid, "0x%lx", wstatus, "%s", "EFAULT");
            ret = -EFAULT;
        }
        sch_lock.unlock();
        LOG_SYSCALL_2("wait4", "%ld", pid, "0x%lx", wstatus, "%ld", ret);
        ECALL_RET(ret, pc+4);
    } else {
        LOG_SYSCALL_2("wait4", "%ld", pid, "0x%lx", wstatus, "%s", "BLOCKED");
        wait4_wait_threads.emplace_back();
        auto &wt = wait4_wait_threads.back();
        wt.cpuid = cpu_id;
        wt.thread = CURT;
        wt.pid = pid;
        wt.status = wstatus;
        _push_context_stack(cpu_id, pc+4);
        VirtAddrT nextpc = switch_next_thread_and_execute(cpu_id, SWFLAG_WAIT);
        sch_lock.unlock();
        return nextpc;
    }
}

SYSCALL_DEFINE_V2(261, prlimit) {
    uint64_t pid = IREG_V(a0);
    uint64_t resource = IREG_V(a1);

    LOG_SYSCALL_2("prlimit", "%ld", pid, "%ld", resource, "%ld", 0UL);
    ECALL_RET(0, pc+4);
}

SYSCALL_DEFINE_V2(278, getrandom) {
    VirtAddrT buf = IREG_V(a0);
    uint64_t buflen = IREG_V(a1);
    uint32_t flags = IREG_V(a2);

    vector<uint8_t> hostbuf;
    hostbuf.assign(buflen, 0);

    int64_t ret = getrandom(hostbuf.data(), buflen, flags);
    if(ret < 0) {
        ret = -errno;
    } else if(!_memcpy_to_target(cpu_id, buf, hostbuf.data(), buflen)) {
        LOG_SYSCALL_3("getrandom", "0x%lx", buf, "%ld", buflen, "0x%x", flags, "%s", "EFAULT");
        ECALL_RET(-EFAULT, pc+4);
    }

    LOG_SYSCALL_3("getrandom", "0x%lx", buf, "%ld", buflen, "0x%x", flags, "%ld", ret);
    ECALL_RET(ret, pc+4);
}

VirtAddrT SMPSystemV2::_page_fault_rx(uint32_t cpu_id, VirtAddrT pc, VirtAddrT badaddr, bool isx) {
    VPageIndexT vpn = (badaddr >> PAGE_ADDR_OFFSET);

    PTET pte = CURT->pgtable->pt_get(vpn, nullptr);

    if((pte & PTE_V) && (pte & PTE_COW) && (pte & PTE_NALLOC)) {
        TgtMemSetList stlist;
        vector<TgtPgCpy> cplist;
        CURT->pgtable->apply_cow(vpn << PAGE_ADDR_OFFSET, &stlist, &cplist);
        for(auto &st : stlist) {
            _perform_target_memset(cpu_id, st);
        }
        for(auto &cp : cplist) {
            _perform_target_pagecpy(cpu_id, cp);
        }
        // cpus->flush_tlb_vpgidx(cpu_id, vpn << PAGE_ADDR_OFFSET, CURT->asid);
        cpus->flush_tlb_all(cpu_id);
        
        LOG_SYSCALL_2("page_fault_rx", "0x%lx", pc, "0x%lx", badaddr, "%d", 0);
        return pc;
    }

    printf("%ld: Segment Fault on CPU %d: %s 0x%lx @0x%lx\n", cpus->get_current_tick(), cpu_id, (isx?"Execute":"Read"), badaddr, pc);
    simroot_assert(0);
    return 0;
}

VirtAddrT SMPSystemV2::_page_fault_w(uint32_t cpu_id, VirtAddrT pc, VirtAddrT badaddr) {
    VPageIndexT vpn = (badaddr >> PAGE_ADDR_OFFSET);

    PTET pte = CURT->pgtable->pt_get(vpn, nullptr);

    if((pte & PTE_V) && (pte & PTE_COW)) {
        TgtMemSetList stlist;
        vector<TgtPgCpy> cplist;
        CURT->pgtable->apply_cow(vpn << PAGE_ADDR_OFFSET, &stlist, &cplist);
        for(auto &st : stlist) {
            _perform_target_memset(cpu_id, st);
        }
        for(auto &cp : cplist) {
            _perform_target_pagecpy(cpu_id, cp);
        }
        // cpus->flush_tlb_vpgidx(cpu_id, vpn << PAGE_ADDR_OFFSET, CURT->asid);
        cpus->flush_tlb_all(cpu_id);
        
        LOG_SYSCALL_2("page_fault_w", "0x%lx", pc, "0x%lx", badaddr, "%d", 0);
        return pc;
    }

    printf("%ld: Segment Fault on CPU %d: Write 0x%lx @0x%lx\n", cpus->get_current_tick(), cpu_id, badaddr, pc);
    simroot_assert(0);
    return 0;
}
