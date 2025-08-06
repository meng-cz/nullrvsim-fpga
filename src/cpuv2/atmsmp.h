#ifndef CPU_ATOMIC_SMP_CORES_V2_H
#define CPU_ATOMIC_SMP_CORES_V2_H


#include "common.h"

#include "cpuv2/cpuinterface2.h"
#include "cpu/isa.h"

#include <fstream>

namespace simcpu {

/**
 * RV64 - SV48 - No Super Page
 */
class AtomicSMPCores : public CPUGroupInterface, public SimObject {

public:

    AtomicSMPCores(uint32_t core_num, PhysAddrT mem_base, uint64_t mem_size);
    ~AtomicSMPCores();

    virtual void process_frames(HTPFrames &frames);

    virtual void halt(uint32_t cpu_id);
    virtual void interrupt(uint32_t cpu_id);
    virtual void set_mmu(uint32_t cpu_id, PhysAddrT pgtable, AsidT asid);
    virtual void redirect(uint32_t cpu_id, VirtAddrT addr);

    virtual bool next(uint32_t *itr_cpu, VirtAddrT *itr_pc, uint32_t *itr_cause, RawDataT *itr_arg);

    virtual void flush_tlb_all(uint32_t cpu_id);
    virtual void flush_tlb_asid(uint32_t cpu_id, AsidT asid);
    virtual void flush_tlb_vpgidx(uint32_t cpu_id, VirtAddrT vaddr, AsidT asid);

    virtual RawDataT regacc_read(uint32_t cpu_id, RVRegIndexT vreg);
    virtual void regacc_write(uint32_t cpu_id, RVRegIndexT vreg, RawDataT data);

    virtual RawDataT pxymem_read(uint32_t cpu_id, PhysAddrT paddr);
    virtual void pxymem_write(uint32_t cpu_id, PhysAddrT paddr, RawDataT data);

    virtual void pxymem_page_read(uint32_t cpu_id, PageIndexT ppn, void * buf);
    virtual void pxymem_page_set(uint32_t cpu_id, PageIndexT ppn, RawDataT value);
    virtual void pxymem_page_write(uint32_t cpu_id, PageIndexT ppn, void * buf);
    virtual void pxymem_page_copy(uint32_t cpu_id, PageIndexT dst, PageIndexT src);
    virtual void pxymem_page_zero(uint32_t cpu_id, PageIndexT ppn) {
        pxymem_page_set(cpu_id, ppn, 0);
    }

    virtual void hfutex_setmask(uint32_t cpu_id, VirtAddrT vaddr);
    virtual void hfutex_clearmask(uint32_t cpu_id);

    virtual uint64_t get_current_tick() {
        return cur_tick;
    };
    virtual uint64_t get_current_utick(uint32_t cpu_id) {
        return uticks[cpu_id];
    }

    virtual void dump_core(std::ofstream &ofile);

    
    uint64_t get_wall_time_tick();


private:

    void _on_cur_simcore(uint32_t id);
    bool _page_trans_and_check(uint32_t id, VirtAddrT vaddr, uint32_t flag, PhysAddrT *paddr);

    int32_t is_sv48 = 0;

typedef struct {
    bool        ishalted;
    bool        interrupt;
    uint32_t    itr_cause;
    uint64_t    itr_arg;
    VirtAddrT   pc;
    PhysAddrT   pgtable;
    AsidT       asid;
    RVRegArray  reg;
    RawDataT    fcsr;
    unordered_map<uint64_t, uint64_t> tlb;
    uint64_t    finished_inst_cnt;
    set<VirtAddrT>  hfutex_mask;
} CoreState;

    vector<CoreState> cores;

    PhysAddrT mem_base;
    vector<uint8_t> main_mem;

    unordered_map<PhysAddrT, set<uint32_t>> lrsc;

    uint64_t cur_tick = 0;
    vector<uint64_t> uticks;

    vector<shared_ptr<std::ofstream>> ofiles;

    bool debug_runtime = false;
};

}

#endif
