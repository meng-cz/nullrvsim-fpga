#ifndef SERIAL_FPGA_H
#define SERIAL_FPGA_H

#include "common.h"

#include "cpuv2/cpuinterface2.h"

using simcpu::CPUGroupInterface;


typedef vector<uint8_t> BufT;

class SerialFPGAAdapter : public CPUGroupInterface, public TraceObject {

public:

    SerialFPGAAdapter(string devfile, uint32_t bps);
    ~SerialFPGAAdapter();

    virtual void process_frames(HTPFrames &frames);

    virtual void halt(uint32_t cpu_id);
    virtual void interrupt(uint32_t cpu_id);
    virtual void set_mmu(uint32_t cpu_id, PhysAddrT pgtable, AsidT asid);
    virtual void redirect(uint32_t cpu_id, VirtAddrT addr);

    virtual bool next(uint32_t *itr_cpu, VirtAddrT *itr_pc, uint32_t *itr_cause, RawDataT *itr_arg);

    virtual void flush_tlb_all(uint32_t cpu_id);
    virtual void flush_tlb_asid(uint32_t cpu_id, AsidT asid) { flush_tlb_vpgidx(cpu_id, 0, asid); };
    virtual void flush_tlb_vpgidx(uint32_t cpu_id, VirtAddrT vaddr, AsidT asid);

    virtual void sync_inst_stream(uint32_t cpu_id);

    virtual RawDataT regacc_read(uint32_t cpu_id, RVRegIndexT vreg);
    virtual void regacc_write(uint32_t cpu_id, RVRegIndexT vreg, RawDataT data);

    virtual RawDataT pxymem_read(uint32_t cpu_id, PhysAddrT paddr);
    virtual void pxymem_write(uint32_t cpu_id, PhysAddrT paddr, RawDataT data);

    virtual void pxymem_page_read(uint32_t cpu_id, PageIndexT ppn, void * dbuf);
    virtual void pxymem_page_set(uint32_t cpu_id, PageIndexT ppn, RawDataT value);
    virtual void pxymem_page_write(uint32_t cpu_id, PageIndexT ppn, void * dbuf);
    virtual void pxymem_page_copy(uint32_t cpu_id, PageIndexT dst, PageIndexT src);
    virtual void pxymem_page_zero(uint32_t cpu_id, PageIndexT ppn);

    virtual uint64_t get_current_tick();
    virtual uint64_t get_current_utick(uint32_t cpu_id);


    virtual void hfutex_setmask(uint32_t cpu_id, VirtAddrT vaddr);
    virtual void hfutex_clearmask(uint32_t cpu_id);

    virtual void dump_core(std::ofstream &ofile);
    virtual void set_debug(bool on) { debug_op = on; };

private:

    int32_t fd = 0;

    int8_t _perform_op(int8_t op, BufT &data, BufT &retdata);

    void _read_serial(void * buf, uint64_t size);
    void _write_serial(void * buf, uint64_t size);

    void _append_int(BufT &buf, uint64_t data, uint64_t bytes);
    void _append_buf(BufT &buf, void * data, uint64_t bytes);

    uint64_t _pop_int(BufT &buf, uint64_t bytes);

    void _send_frame(HTPFrame &frame);
    void _recv_frame(HTPFrame &frame);
    void _perform_pgrdwt_frame(HTPFrame &frame);

    bool always_flush_all = false;

    int32_t dbg = 0;
    bool debug_op = false;
    bool debug_runtime = false;
};


#endif
