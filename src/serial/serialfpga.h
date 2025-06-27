#ifndef SERIAL_FPGA_H
#define SERIAL_FPGA_H

#include "common.h"

#include "cpuv2/cpuinterface2.h"

using simcpu::CPUGroupInterface;

#define SEROP_NEXT          (0)     //  OP[8]                               -> ACK[8] ID[16] CAUSE[8] PC[48] ARG[48]
#define SEROP_HALT          (1)     //  OP[8]       ID[16]                  -> ACK[8]
#define SEROP_ITR           (2)     //  OP[8]       ID[16]                  -> ACK[8]
#define SEROP_MMU           (3)     //  OP[8]       ID[16] ASID[16] PT[40]  -> ACK[8]
#define SEROP_REDIR         (4)     //  OP[8]       ID[16] PC[48]           -> ACK[8]
#define SEROP_FTLB          (5)     //  OP[8]       ID[16]                  -> ACK[8]
#define SEROP_FTLB2         (6)     //  OP[8]       ID[16] ASID[16] VPN[40] -> ACK[8]
#define SEROP_SYNCI         (7)     //  OP[8]       ID[16]                  -> ACK[8]
#define SEROP_REGRD         (8)     //  OP[8]       ID[16] REG[16]          -> ACK[8] DATA[64]
#define SEROP_REGWT         (9)     //  OP[8]       ID[16] REG[16] DATA[64] -> ACK[8]
#define SEROP_MEMRD         (10)    //  OP[8]       ID[16] PA[48]           -> ACK[8] DATA[64]
#define SEROP_MEMWT         (11)    //  OP[8]       ID[16] PA[48] DATA[64]  -> ACK[8]
#define SEROP_PGRD          (12)    //  OFF[3]OP[5] ID[16] PPN[40]          -> ACK[8] DATA[512B]
#define SEROP_PGST          (13)    //  OP[8]       ID[16] PPN[40] VALUE[64]-> ACK[8]
#define SEROP_PGWT          (14)    //  OFF[3]OP[5] ID[16] PPN[40]          DATA[512B]-> ACK[8]
#define SEROP_PGCP          (15)    //  OP[8]       ID[16] DST[40] SRC[40]  -> ACK[8]
#define SEROP_CLK           (16)    //  OP[8]                               -> ACK[8] CLK[64]
#define SEROP_NUM           (17)

#define SERACK_ALLHALT      (63)

typedef vector<uint8_t> BufT;

class SerialFPGAAdapter : public CPUGroupInterface, public TraceObject {

public:

    SerialFPGAAdapter(string devfile, uint32_t bps);
    ~SerialFPGAAdapter();

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

    virtual uint64_t get_current_tick();

    virtual void dump_core(std::ofstream &ofile);
    virtual void set_debug(bool on) { debug_op = on; };

private:

    int32_t fd = 0;

    int8_t _perform_op(int8_t op, BufT &data, BufT &retdata);

    void _read_serial(void * buf, uint64_t size);
    void _write_serial(void * buf, uint64_t size);

    void _append_int(BufT &buf, int64_t data, uint64_t bytes);
    void _append_buf(BufT &buf, void * data, uint64_t bytes);

    int64_t _pop_int(BufT &buf, uint64_t bytes);

    int32_t dbg = 0;
    bool debug_op = false;
};


#endif
