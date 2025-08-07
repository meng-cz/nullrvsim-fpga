
#include "cpuinterface2.h"
#include "simroot.h"

void htp_push_halt(HTPFrames &frames, uint32_t cpu_id) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::halt;
    add.cpuid = cpu_id;
}

void htp_push_interrupt(HTPFrames &frames, uint32_t cpu_id) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::itr;
    add.cpuid = cpu_id;
}

void htp_push_set_mmu(HTPFrames &frames, uint32_t cpu_id, PhysAddrT pgtable, AsidT asid) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::mmu;
    add.cpuid = cpu_id;
    add.x1 = asid;
    add.x2 = pgtable;
}

void htp_push_redirect(HTPFrames &frames, uint32_t cpu_id, VirtAddrT addr) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::redir;
    add.cpuid = cpu_id;
    add.x1 = addr;
}

void htp_push_next(HTPFrames &frames) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::next;
}

void htp_push_flush_tlb_all(HTPFrames &frames, uint32_t cpu_id) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::ftlb;
    add.cpuid = cpu_id;
}

void htp_push_flush_tlb_vpgidx(HTPFrames &frames, uint32_t cpu_id, VirtAddrT vaddr, AsidT asid) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::ftlb2;
    add.cpuid = cpu_id;
    add.x1 = asid;
    add.x2 = vaddr;
}

void htp_push_sync_inst_stream(HTPFrames &frames, uint32_t cpu_id) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::synci;
    add.cpuid = cpu_id;
}

void htp_push_regacc_read(HTPFrames &frames, uint32_t cpu_id, RVRegIndexT vreg) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::regrd;
    add.cpuid = cpu_id;
    add.x1 = vreg;
}

void htp_push_regacc_write(HTPFrames &frames, uint32_t cpu_id, RVRegIndexT vreg, RawDataT data) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::regwt;
    add.cpuid = cpu_id;
    add.x1 = vreg;
    add.x2 = data;
}

void htp_push_pxymem_read(HTPFrames &frames, uint32_t cpu_id, PhysAddrT paddr) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::memrd;
    add.cpuid = cpu_id;
    add.x1 = paddr;
}

void htp_push_pxymem_write(HTPFrames &frames, uint32_t cpu_id, PhysAddrT paddr, RawDataT data) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::memwt;
    add.cpuid = cpu_id;
    add.x1 = paddr;
    add.x2 = data;
}

void htp_push_pxymem_page_read(HTPFrames &frames, uint32_t cpu_id, PageIndexT ppn) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::pgrd;
    add.cpuid = cpu_id;
    add.x1 = ppn;
}

void htp_push_pxymem_page_set(HTPFrames &frames, uint32_t cpu_id, PageIndexT ppn, RawDataT value) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::pgst;
    add.cpuid = cpu_id;
    add.x1 = ppn;
    add.x2 = value;
}

void htp_push_pxymem_page_write(HTPFrames &frames, uint32_t cpu_id, PageIndexT ppn, void *data) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::pgwt;
    add.cpuid = cpu_id;
    add.x1 = ppn;
    add.d1.resize(PAGE_LEN_BYTE);
    memcpy(add.d1.data(), data, PAGE_LEN_BYTE);
}

void htp_push_pxymem_page_copy(HTPFrames &frames, uint32_t cpu_id, PageIndexT dst, PageIndexT src) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::pgcp;
    add.cpuid = cpu_id;
    add.x1 = dst;
    add.x2 = src;
}

void htp_push_pxymem_page_zero(HTPFrames &frames, uint32_t cpu_id, PageIndexT ppn) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::pgzero;
    add.cpuid = cpu_id;
    add.x1 = ppn;
}

void htp_push_get_current_tick(HTPFrames &frames) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::clk;
}

void htp_push_get_current_utick(HTPFrames &frames, uint32_t cpu_id) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::uclk;
    add.cpuid = cpu_id;
}

void htp_push_hfutex_setmask(HTPFrames &frames, uint32_t cpu_id, VirtAddrT vaddr) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::hfset;
    add.cpuid = cpu_id;
    add.x1 = vaddr;
}

void htp_push_hfutex_clearmask(HTPFrames &frames, uint32_t cpu_id) {
    frames.emplace_back();
    auto &add = frames.back();
    add.opcode = HTOP::hfclr;
    add.cpuid = cpu_id;
}

inline bool _htp_has_retuen(HTOP opcode) {
    return (
        opcode == HTOP::next ||
        opcode == HTOP::regrd ||
        opcode == HTOP::memrd ||
        opcode == HTOP::pgrd ||
        opcode == HTOP::clk ||
        opcode == HTOP::uclk
    );
}

void htp_pop_next(HTPFrames &frames, uint32_t *itr_cpu, VirtAddrT *itr_pc, uint32_t *itr_cause, RawDataT *itr_arg) {
    simroot_assert(frames.size());
    while(frames.front().opcode != HTOP::next) {
        simroot_assert(!_htp_has_retuen(frames.front().opcode));
        frames.pop_front();
    }
    auto &pop = frames.front();
    *itr_cpu = pop.cpuid;
    *itr_cause = pop.x1;
    *itr_pc = pop.x2;
    *itr_arg = pop.x3;
    frames.pop_front();
}

void htp_pop_regacc_read(HTPFrames &frames, RawDataT *data) {
    simroot_assert(frames.size());
    while(frames.front().opcode != HTOP::regrd) {
        simroot_assert(!_htp_has_retuen(frames.front().opcode));
        frames.pop_front();
    }
    auto &pop = frames.front();
    *data = pop.x2;
    frames.pop_front();
}

void htp_pop_pxymem_read(HTPFrames &frames, RawDataT *data) {
    simroot_assert(frames.size());
    while(frames.front().opcode != HTOP::memrd) {
        simroot_assert(!_htp_has_retuen(frames.front().opcode));
        frames.pop_front();
    }
    auto &pop = frames.front();
    *data = pop.x2;
    frames.pop_front();
}

void htp_pop_page_read(HTPFrames &frames, void *data) {
    simroot_assert(frames.size());
    while(frames.front().opcode != HTOP::pgrd) {
        simroot_assert(!_htp_has_retuen(frames.front().opcode));
        frames.pop_front();
    }
    auto &pop = frames.front();
    simroot_assert(pop.d1.size() >= PAGE_LEN_BYTE);
    memcpy(data, pop.d1.data(), PAGE_LEN_BYTE);
    frames.pop_front();
}

void htp_pop_get_current_tick(HTPFrames &frames, uint64_t *data) {
    simroot_assert(frames.size());
    while(frames.front().opcode != HTOP::clk) {
        simroot_assert(!_htp_has_retuen(frames.front().opcode));
        frames.pop_front();
    }
    auto &pop = frames.front();
    *data = pop.x1;
    frames.pop_front();
}

void htp_pop_get_current_utick(HTPFrames &frames, uint64_t *data) {
    simroot_assert(frames.size());
    while(frames.front().opcode != HTOP::uclk) {
        simroot_assert(!_htp_has_retuen(frames.front().opcode));
        frames.pop_front();
    }
    auto &pop = frames.front();
    *data = pop.x1;
    frames.pop_front();
}

HTOP htp_pop_next_return(HTPFrames &frames) {
    while(frames.size() && !_htp_has_retuen(frames.front().opcode)) {
        frames.pop_front();
    }
    simroot_assert(frames.size());
    return frames.front().opcode;
}

uint64_t htp_pop_regacc_read(HTPFrames &frames) {
    uint64_t ret = 0; 
    htp_pop_regacc_read(frames, &ret);
    return ret;
}
uint64_t htp_pop_pxymem_read(HTPFrames &frames) {
    uint64_t ret = 0; 
    htp_pop_regacc_read(frames, &ret);
    return ret;
}
uint64_t htp_pop_get_current_tick(HTPFrames &frames) {
    uint64_t ret = 0; 
    htp_pop_get_current_tick(frames, &ret);
    return ret;
}
uint64_t htp_pop_get_current_utick(HTPFrames &frames) {
    uint64_t ret = 0; 
    htp_pop_get_current_utick(frames, &ret);
    return ret;
}

