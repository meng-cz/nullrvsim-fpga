#ifndef CPU_INTERFACE_V2_H
#define CPU_INTERFACE_V2_H

#include "common.h"
#include "simerror.h"

#include "cpu/isa.h"

using isa::RVRegArray;

#define SEROP_NEXT          (0)     //  OP[8]                               -> ACK[8] ID[8] CAUSE[8] PC[48] (ARG[48])
#define SEROP_HALT          (1)     //  OP[8]       ID[8]                  -> ACK[8]
#define SEROP_ITR           (2)     //  OP[8]       ID[8]                  -> ACK[8]
#define SEROP_MMU           (3)     //  OP[8]       ID[8] ASID[16] PT[40]  -> ACK[8]
#define SEROP_REDIR         (4)     //  OP[8]       ID[8] PC[48]           -> ACK[8]
#define SEROP_FTLB          (5)     //  OP[8]       ID[8]                  -> ACK[8]
#define SEROP_FTLB2         (6)     //  OP[8]       ID[8] ASID[16] VPN[40] -> ACK[8]
#define SEROP_SYNCI         (7)     //  OP[8]       ID[8]                  -> ACK[8]
#define SEROP_REGRD         (8)     //  OP[8]       ID[8] REG[16]          -> ACK[8] DATA[64]
#define SEROP_REGWT         (9)     //  OP[8]       ID[8] REG[16] DATA[64] -> ACK[8]
#define SEROP_MEMRD         (10)    //  OP[8]       ID[8] PA[48]           -> ACK[8] DATA[64]
#define SEROP_MEMWT         (11)    //  OP[8]       ID[8] PA[48] DATA[64]  -> ACK[8]
#define SEROP_PGRD          (12)    // (div8) OFF[3]OP[5] ID[8] PPN[40]          -> ACK[8] DATA[512B]
#define SEROP_PGST          (13)    //  OP[8]       ID[8] PPN[40] VALUE[64]-> ACK[8]
#define SEROP_PGWT          (14)    // (div8) OFF[3]OP[5] ID[8] PPN[40]          DATA[512B]-> ACK[8]
#define SEROP_PGCP          (15)    //  OP[8]       ID[8] DST[40] SRC[40]  -> ACK[8]
#define SEROP_CLK           (16)    //  OP[8]                               -> ACK[8] CLK[64]
#define SEROP_UCLK          (17)    //  OP[8]       ID[8]                  -> ACK[8] CLK[64]
#define SEROP_HFSET         (18)    //  OP[8]       ID[8] VA[48]           -> ACK[8]
#define SEROP_HFCLR         (19)    //  OP[8]       ID[8]                  -> ACK[8]
#define SEROP_PGZERO        (20)    //  OP[8]       ID[8] PPN[40]          -> ACK[8]
#define SEROP_NUM           (21)

enum class HTOP {
    next = 0,       //  void                -> ID X1CAUSE X2PC X3ARG
    halt = 1,       //  ID
    itr = 2,        //  ID
    mmu = 3,        //  ID  X1ASID  X2PTADDR
    redir = 4,      //  ID  X1PC
    ftlb = 5,       //  ID 
    ftlb2 = 6,      //  ID  X1ASID  X2VADDR
    synci = 7,      //  ID
    regrd = 8,      //  ID  X1REG           ->    X2DATA
    regwt = 9,      //  ID  X1REG  X2DATA  
    memrd = 10,     //  ID  X1PA            ->    X2DATA
    memwt = 11,     //  ID  X1PA  X2DATA
    pgrd = 12,      //  ID  X1PPN           ->    D1DATA
    pgst = 13,      //  ID  X1PPN  X2VALUE
    pgwt = 14,      //  ID  X1PPN  D1DATA
    pgcp = 15,      //  ID  X1DST  X2SRC
    clk = 16,       //  void                ->    X1CLK
    uclk = 17,      //  ID                  ->    X1CLK
    hfset = 18,     //  ID  X1VA
    hfclr = 19,     //  ID 
    pgzero = 20,    //  ID  X1PPN
};

class HTPFrame {
public:
    HTOP        opcode = HTOP::next;
    uint32_t    cpuid = 0;
    uint64_t    x1 = 0;
    uint64_t    x2 = 0;
    uint64_t    x3 = 0;
    vector<uint8_t> d1;
};

typedef list<HTPFrame> HTPFrames;

void htp_push_halt(HTPFrames &frames, uint32_t cpu_id);
void htp_push_interrupt(HTPFrames &frames, uint32_t cpu_id);
void htp_push_set_mmu(HTPFrames &frames, uint32_t cpu_id, PhysAddrT pgtable, AsidT asid);
void htp_push_redirect(HTPFrames &frames, uint32_t cpu_id, VirtAddrT addr);
void htp_push_next(HTPFrames &frames);
void htp_push_flush_tlb_all(HTPFrames &frames, uint32_t cpu_id);
void htp_push_flush_tlb_vpgidx(HTPFrames &frames, uint32_t cpu_id, VirtAddrT vaddr, AsidT asid);
void htp_push_sync_inst_stream(HTPFrames &frames, uint32_t cpu_id);
void htp_push_regacc_read(HTPFrames &frames, uint32_t cpu_id, RVRegIndexT vreg);
void htp_push_regacc_write(HTPFrames &frames, uint32_t cpu_id, RVRegIndexT vreg, RawDataT data);
void htp_push_pxymem_read(HTPFrames &frames, uint32_t cpu_id, PhysAddrT paddr);
void htp_push_pxymem_write(HTPFrames &frames, uint32_t cpu_id, PhysAddrT paddr, RawDataT data);
void htp_push_pxymem_page_read(HTPFrames &frames, uint32_t cpu_id, PageIndexT ppn);
void htp_push_pxymem_page_set(HTPFrames &frames, uint32_t cpu_id, PageIndexT ppn, RawDataT value);
void htp_push_pxymem_page_write(HTPFrames &frames, uint32_t cpu_id, PageIndexT ppn, void *data);
void htp_push_pxymem_page_copy(HTPFrames &frames, uint32_t cpu_id, PageIndexT dst, PageIndexT src);
void htp_push_pxymem_page_zero(HTPFrames &frames, uint32_t cpu_id, PageIndexT ppn);
void htp_push_get_current_tick(HTPFrames &frames);
void htp_push_get_current_utick(HTPFrames &frames, uint32_t cpu_id);
void htp_push_hfutex_setmask(HTPFrames &frames, uint32_t cpu_id, VirtAddrT vaddr);
void htp_push_hfutex_clearmask(HTPFrames &frames, uint32_t cpu_id);

void htp_pop_next(HTPFrames &frames, uint32_t *itr_cpu, VirtAddrT *itr_pc, uint32_t *itr_cause, RawDataT *itr_arg);
void htp_pop_regacc_read(HTPFrames &frames, RawDataT *data);
void htp_pop_pxymem_read(HTPFrames &frames, RawDataT *data);
void htp_pop_page_read(HTPFrames &frames, void *data);
void htp_pop_get_current_tick(HTPFrames &frames, uint64_t *data);
void htp_pop_get_current_utick(HTPFrames &frames, uint64_t *data);

HTOP htp_pop_next_return(HTPFrames &frames);

namespace simcpu {

class CPUGroupInterface {
public:
    virtual void process_frames(HTPFrames &frames) = 0;

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
    virtual void pxymem_page_zero(uint32_t cpu_id, PageIndexT ppn) = 0;

    virtual uint64_t get_current_tick() = 0;
    virtual uint64_t get_current_utick(uint32_t cpu_id) = 0;

    virtual void hfutex_setmask(uint32_t cpu_id, VirtAddrT vaddr) = 0;
    virtual void hfutex_clearmask(uint32_t cpu_id) = 0;
};


}

#endif
