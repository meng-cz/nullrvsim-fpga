#ifndef CPU_INTERFACE_V2_H
#define CPU_INTERFACE_V2_H

#include "common.h"
#include "simerror.h"

#include "cpu/isa.h"

using isa::RVRegArray;

namespace simcpu {

class CPUGroupInterface {
public:
    virtual void halt(uint32_t cpu_id) = 0;
    virtual void interrupt(uint32_t cpu_id) = 0;
    virtual void set_mmu(uint32_t cpu_id, PhysAddrT pgtable, AsidT asid) = 0;
    virtual void redirect(uint32_t cpu_id, VirtAddrT addr) = 0;

    virtual bool next(uint32_t *itr_cpu, VirtAddrT *itr_pc, uint32_t *itr_cause, RawDataT *itr_arg) = 0;

    virtual void flush_tlb_all(uint32_t cpu_id) = 0;
    virtual void flush_tlb_asid(uint32_t cpu_id, AsidT asid) = 0;
    virtual void flush_tlb_vpgidx(uint32_t cpu_id, VirtAddrT vaddr, AsidT asid) = 0;

    virtual void sync_inst_stream(uint32_t cpu_id) {};

    virtual RawDataT regacc_read(uint32_t cpu_id, RVRegIndexT vreg) = 0;
    virtual void regacc_write(uint32_t cpu_id, RVRegIndexT vreg, RawDataT data) = 0;

    virtual RawDataT pxymem_read(uint32_t cpu_id, PhysAddrT paddr) = 0;
    virtual void pxymem_write(uint32_t cpu_id, PhysAddrT paddr, RawDataT data) = 0;

    virtual void pxymem_page_read(uint32_t cpu_id, PageIndexT ppn, void * buf) = 0;
    virtual void pxymem_page_set(uint32_t cpu_id, PageIndexT ppn, RawDataT value) = 0;
    virtual void pxymem_page_write(uint32_t cpu_id, PageIndexT ppn, void * buf) = 0;
    virtual void pxymem_page_copy(uint32_t cpu_id, PageIndexT dst, PageIndexT src) = 0;

    virtual uint64_t get_current_tick() = 0;
    virtual uint64_t get_current_utick(uint32_t cpu_id) = 0;
};


}

#endif
