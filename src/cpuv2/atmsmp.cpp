
#include "atmsmp.h"

#include "cpu/isa.h"
#include "cpu/operation.h"
#include "cpu/csr.h"

#include "simroot.h"
#include "configuration.h"

using isa::RV64OPCode;

namespace simcpu {

AtomicSMPCores::AtomicSMPCores(uint32_t core_num, PhysAddrT mem_base, uint64_t mem_size) {

    cores.resize(core_num);
    for(auto &c : cores) {
        c.ishalted = true;
        c.interrupt = false;
        c.pc = 0;
        c.pgtable = 0;
        c.asid = 0;
        c.reg.fill(0);
        c.fcsr = 0;
        c.finished_inst_cnt = 0;
    }

    this->mem_base = mem_base;
    main_mem.resize(mem_size);

    if(conf::get_int("cpu", "log_inst_to_file", 0)) {
        ofiles.resize(core_num);
        for(uint32_t i = 0; i < core_num; i++) {
            char namebuf[128];
            sprintf(namebuf, "CPU-%d-insts.log", i);
            ofiles[i] = make_shared<std::ofstream>(namebuf, std::ios::out);
        }
    }

    is_sv48 = conf::get_int("root", "vm_is_sv48", 1);

    uticks.assign(core_num, 0);
}

AtomicSMPCores::~AtomicSMPCores() {
    for(auto &f : ofiles) {
        if(f) {
            f->close();
        }
    }
}

void AtomicSMPCores::dump_core(std::ofstream &ofile) {
    for(auto &f : ofiles) {
        if(f) {
            f->close();
        }
    }

    for(uint64_t i = 0; i < cores.size(); i++) {
        char logbuf[256];
        auto &c = cores[i];
        ofile << "Core " << i << ":\n";
        if(c.ishalted) ofile << "Halted\n";
        else if(c.interrupt) {
            sprintf(logbuf, "Interrupt %d, arg 0x%lx\n", c.itr_cause, c.itr_arg);
            ofile << logbuf;
        }
        sprintf(logbuf, "PC: 0x%lx\n", c.pc);
        ofile << logbuf;
        ofile << "PT: " << c.pgtable << "\n";
        ofile << "ASID: " << c.asid << "\n";
        ofile << "Reg:\n";
        for(uint32_t r = 0; r < RV_REG_CNT_INT; r+=2) {
            sprintf(logbuf, "%02d-%s: 0x%16lx, %02d-%s: 0x%16lx\n",
                r, isa::ireg_name(r), c.reg[r],
                r+1, isa::ireg_name(r+1), c.reg[r+1]
            );
            ofile << logbuf;
        }
        ofile << "FReg:\n";
        for(uint32_t r = 0; r < RV_REG_CNT_FP; r+=2) {
            sprintf(logbuf, "%02d-%s: 0x%16lx, %02d-%s: 0x%16lx\n",
                r, isa::ireg_name(r), c.reg[r+RV_REG_CNT_INT],
                r+1, isa::ireg_name(r+1), c.reg[r+1+RV_REG_CNT_INT]
            );
            ofile << logbuf;
        }
        ofile << "TLB:\n";
        for(auto &entry : c.tlb) {
            sprintf(logbuf, "VP 0x%lx, ASID %ld -> PP 0x%lx, Flag 0x%lx\n",
                entry.first >> 16, entry.first & 0xffffUL, 
                entry.second >> 10, entry.second & 0x3ffUL);
            ofile << logbuf;
        }
        ofile << "\n";

    }
}

void AtomicSMPCores::halt(uint32_t cpu_id) {
    cores[cpu_id].ishalted = true;
}

void AtomicSMPCores::interrupt(uint32_t cpu_id){
    if(!cores[cpu_id].interrupt) {
        cores[cpu_id].interrupt = true;
        cores[cpu_id].itr_cause = ITR_USR_EXTERNAL;
    }
}

void AtomicSMPCores::set_mmu(uint32_t cpu_id, PhysAddrT pgtable, AsidT asid){
    cores[cpu_id].pgtable = pgtable;
    cores[cpu_id].asid = asid;
}

void AtomicSMPCores::redirect(uint32_t cpu_id, VirtAddrT addr){
    cores[cpu_id].pc = addr;
    cores[cpu_id].ishalted = false;
}

bool AtomicSMPCores::next(uint32_t *itr_cpu, VirtAddrT *itr_pc, uint32_t *itr_cause, RawDataT *itr_arg){

    while (1) {
        for(uint32_t i = 0; i < cores.size(); i++) {
            if(cores[i].interrupt) {
                *itr_cpu = i;
                *itr_pc = cores[i].pc;
                *itr_cause = cores[i].itr_cause;
                *itr_arg = cores[i].itr_arg;
                cores[i].interrupt = false;
                return true;
            }
        }

        bool all_halt = true;

        for(uint32_t i = 0; i < cores.size(); i++) {
            if(!cores[i].ishalted) all_halt = false;
            _on_cur_simcore(i);
        }

        cur_tick++;

        if(all_halt) return false;
    }
}

void AtomicSMPCores::flush_tlb_all(uint32_t cpu_id) {
    cores[cpu_id].tlb.clear();
}
void AtomicSMPCores::flush_tlb_asid(uint32_t cpu_id, AsidT asid) {
    for(auto iter = cores[cpu_id].tlb.begin(); iter != cores[cpu_id].tlb.end(); ) {
        if((iter->first & 0xffffUL) == asid) {
            iter = cores[cpu_id].tlb.erase(iter);
        }
        else {
            iter++;
        }
    }
}
void AtomicSMPCores::flush_tlb_vpgidx(uint32_t cpu_id, VirtAddrT vaddr, AsidT asid) {
    cores[cpu_id].tlb.erase(((vaddr & (~0xfffUL)) << 4) | (asid & 0xffffUL));
}

RawDataT AtomicSMPCores::regacc_read(uint32_t cpu_id, RVRegIndexT vreg) {
    return (vreg?(cores[cpu_id].reg[vreg]):0);
}
void AtomicSMPCores::regacc_write(uint32_t cpu_id, RVRegIndexT vreg, RawDataT data) {
    if(vreg) cores[cpu_id].reg[vreg] = data;
}

RawDataT AtomicSMPCores::pxymem_read(uint32_t cpu_id, PhysAddrT paddr) {
    return *((RawDataT*)(main_mem.data() + ((paddr - mem_base) & (~7UL))));
}
void AtomicSMPCores::pxymem_write(uint32_t cpu_id, PhysAddrT paddr, RawDataT data) {
    *((RawDataT*)(main_mem.data() + ((paddr - mem_base) & (~7UL)))) = data;
}

void AtomicSMPCores::pxymem_page_read(uint32_t cpu_id, PageIndexT ppn, void * buf) {
    memcpy(buf, main_mem.data() + ((ppn * PAGE_LEN_BYTE) - mem_base), PAGE_LEN_BYTE);
}
void AtomicSMPCores::pxymem_page_set(uint32_t cpu_id, PageIndexT ppn, RawDataT value) {
    vector<RawDataT> buf;
    buf.assign(PAGE_LEN_BYTE/8, value);
    memcpy(main_mem.data() + ((ppn * PAGE_LEN_BYTE) - mem_base), buf.data(), PAGE_LEN_BYTE);
}
void AtomicSMPCores::pxymem_page_write(uint32_t cpu_id, PageIndexT ppn, void * buf) {
    memcpy(main_mem.data() + ((ppn * PAGE_LEN_BYTE) - mem_base), buf, PAGE_LEN_BYTE);
}
void AtomicSMPCores::pxymem_page_copy(uint32_t cpu_id, PageIndexT dst, PageIndexT src) {
    memcpy(main_mem.data() + (dst * PAGE_LEN_BYTE - mem_base), main_mem.data() + (src * PAGE_LEN_BYTE - mem_base), PAGE_LEN_BYTE);
}


bool AtomicSMPCores::_page_trans_and_check(uint32_t id, VirtAddrT vaddr, uint32_t flag, PhysAddrT *paddr) {
    CoreState &core = cores[id];
    auto &tlb = core.tlb;

    uint64_t vidx = (((vaddr & (~0xfffUL)) << 4) | (core.asid & 0xffffUL));
    auto iter = tlb.find(vidx);
    uint64_t pte = 0;
    if(iter != tlb.end()) {
        pte = iter->second;
    } else {
        if(is_sv48) {
            memcpy(&pte, main_mem.data() + ((core.pgtable & (~0xfffUL)) + 8 * ((vaddr >> 39) & 0x1ffUL) - mem_base), 8);
            if(!(pte & PTE_V)) return false;
            memcpy(&pte, main_mem.data() + (((pte & (~0x3ffUL)) << 2) + 8 * ((vaddr >> 30) & 0x1ffUL) - mem_base), 8);
        } else {
            memcpy(&pte, main_mem.data() + ((core.pgtable & (~0xfffUL)) + 8 * ((vaddr >> 30) & 0x1ffUL) - mem_base), 8);
        }
        if(!(pte & PTE_V)) return false;
        memcpy(&pte, main_mem.data() + (((pte & (~0x3ffUL)) << 2) + 8 * ((vaddr >> 21) & 0x1ffUL) - mem_base), 8);
        if(!(pte & PTE_V)) return false;
        memcpy(&pte, main_mem.data() + (((pte & (~0x3ffUL)) << 2) + 8 * ((vaddr >> 12) & 0x1ffUL) - mem_base), 8);
        if(!(pte & PTE_V)) return false;
        tlb[vidx] = pte;
    }

    if((pte & (flag | PTE_V)) == (flag | PTE_V)) {
        *paddr = (((pte & (~0x3ffUL)) << 2) | (vaddr & 0xfffUL));
        return true;
    } else {
        return false;
    }
}

void AtomicSMPCores::_on_cur_simcore(uint32_t id) {
    CoreState &core = cores[id];

    if(core.ishalted || core.interrupt) {
        return;
    }

    uticks[id]++;

    uint32_t raw_inst = 0;
    isa::RV64InstDecoded inst;
    inst.pc = core.pc;

#define RAISE_ITR(cause, arg) do { core.pc = inst.pc; core.interrupt = true; core.itr_cause = cause; core.itr_arg = arg; return; } while(0)

#define CHECK_ADDR_ALIGN(addr, len) do { switch (len) { \
    case 1: break; \
    case 2: if(addr & 1) RAISE_ITR(ITR_ST_MISALIGN, vaddr); break; \
    case 4: if(addr & 3) RAISE_ITR(ITR_ST_MISALIGN, vaddr); break; \
    case 8: if(addr & 7) RAISE_ITR(ITR_ST_MISALIGN, vaddr); break; \
    default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst); }} while(0)

    char logbuf[256];

#define LOG_INST(fmt, ...) do { if(!ofiles.empty()) { \
    int64_t len = sprintf(logbuf, "%ld: ", get_current_tick()); \
    ofiles[id]->write(logbuf, len); \
    len = sprintf(logbuf, fmt, __VA_ARGS__); \
    ofiles[id]->write(logbuf, len); \
    ofiles[id]->write("\n", 1); } } while(0)

    {
        if(core.pc & 1) {
            RAISE_ITR(ITR_INST_MISALIGN, inst.pc);
        }
        PhysAddrT ppc = 0;
        if(!_page_trans_and_check(id, inst.pc, PTE_X, &ppc)) {
            RAISE_ITR(ITR_INST_PGFAULT, inst.pc);
        }
        raw_inst = *((uint16_t*)(main_mem.data() + ppc - mem_base));

        if(!isa::isRVC(raw_inst)) {
            PhysAddrT nppc = 0;
            if(!_page_trans_and_check(id, inst.pc+2, PTE_X, &nppc)) {
                RAISE_ITR(ITR_INST_PGFAULT, inst.pc+2);
            }
            raw_inst |= (*((uint16_t*)(main_mem.data() + nppc - mem_base)) << 16);
        }

        if(!isa::decode_rv64(raw_inst, &inst)) {
            RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        inst.pc = core.pc;
        core.pc += ((inst.is_rvc())?2:4);

        if(!ofiles.empty()) {
            isa::init_rv64_inst_name_str(&inst);
        }
    }

    RVRegIndexT rd1 = inst.rd;
    RVRegIndexT rs1 = inst.rs1;
    RVRegIndexT rs2 = inst.rs2;
    RVRegIndexT rs3 = inst.rs3;

    RVRegArray &reg = core.reg;

    if(inst.opcode == RV64OPCode::auipc) {
        if(rd1) reg[rd1] = inst.pc + RAW_DATA_AS(inst.imm).i64;
        LOG_INST("%s -> 0x%lx", inst.debug_name_str.c_str(), reg[rd1]);
    }
    else if(inst.opcode == RV64OPCode::lui) {
        if(rd1) reg[rd1] = inst.imm;
        LOG_INST("%s -> 0x%lx", inst.debug_name_str.c_str(), reg[rd1]);
    }
    else if(inst.opcode == RV64OPCode::jal) {
        if(rd1) reg[rd1] = inst.pc + ((inst.is_rvc())?2:4);
        core.pc = inst.pc + RAW_DATA_AS(inst.imm).i64;
        LOG_INST("%s > 0x%lx -> 0x%lx", inst.debug_name_str.c_str(), core.pc, reg[rd1]);
    }
    else if(inst.opcode == RV64OPCode::jalr) {
        core.pc = reg[rs1] + RAW_DATA_AS(inst.imm).i64;
        if(rd1) reg[rd1] = inst.pc + ((inst.is_rvc())?2:4);
        LOG_INST("%s > 0x%lx -> 0x%lx", inst.debug_name_str.c_str(), core.pc, reg[rd1]);
    }
    else if(inst.opcode == RV64OPCode::branch) {
        RawDataT br = 0;
        if(SimError::success != isa::perform_branch_op_64(inst.param.branch, &br, reg[rs1], reg[rs2])) {
            RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        if(br) {
            core.pc = inst.pc + RAW_DATA_AS(inst.imm).i64;
        }
        LOG_INST("%s > 0x%lx -> 0x%lx", inst.debug_name_str.c_str(), core.pc, reg[rd1]);
    }
    else if(inst.opcode == RV64OPCode::amo) {
        VirtAddrT vaddr = reg[rs1];
        RawDataT data = reg[rs2];
        uint32_t len = isa::rv64_ls_width_to_length(inst.param.amo.wid);
        PhysAddrT paddr = 0;
        RawDataT ret = 0;
        CHECK_ADDR_ALIGN(vaddr, len);
        if(!_page_trans_and_check(id, vaddr, PTE_R | PTE_W, &paddr)) {
            RAISE_ITR(ITR_ST_PGFAULT, vaddr);
        }
        if(inst.param.amo.op == isa::RV64AMOOP5::SC) {
            auto iter = srlc.find(paddr & (~0x3fUL));
            if(iter == srlc.end() || iter->second != id) {
                ret = 1;
            } else {
                memcpy(main_mem.data() + (paddr - mem_base), &data, len);
            }
            srlc.erase(paddr & (~0x3fUL));
        }
        else if(inst.param.amo.op == isa::RV64AMOOP5::LR) {
            memcpy(&ret, main_mem.data() + (paddr - mem_base), len);
            srlc[paddr & (~0x3fUL)] = id;
            if(len == 4) {
                RAW_DATA_AS(ret).i64 = RAW_DATA_AS(ret).i32;
            }
        }
        else {
            IntDataT previous = 0;
            IntDataT stvalue = 0;
            IntDataT value = data;
            memcpy(&previous, main_mem.data() + (paddr - mem_base), len);
            if(len == 4) {
                RAW_DATA_AS(ret).i64 = RAW_DATA_AS(ret).i32;
            }
            if(SimError::success != isa::perform_amo_op(inst.param.amo, &stvalue, previous, value)) {
                RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
            }
            memcpy(main_mem.data() + (paddr - mem_base), &stvalue, len);
            ret = previous;
            srlc.erase(paddr & (~0x3fUL));
        }
        if(rd1) reg[rd1] = ret;
        LOG_INST("%s -> 0x%lx to @0x%lx (0x%lx)", inst.debug_name_str.c_str(), reg[rd1], vaddr, paddr);
    }
    else if(inst.opcode == RV64OPCode::load || inst.opcode == RV64OPCode::loadfp) {
        VirtAddrT vaddr = reg[rs1] + RAW_DATA_AS(inst.imm).i64;
        uint32_t len = isa::rv64_ls_width_to_length(inst.param.loadstore);
        uint64_t buf = 0;
        RawDataT ret = 0;
        PhysAddrT paddr = 0;
        CHECK_ADDR_ALIGN(vaddr, len);
        if(!_page_trans_and_check(id, vaddr, PTE_R, &paddr)) {
            RAISE_ITR(ITR_LD_PGFAULT, vaddr);
        }
        memcpy(&buf, main_mem.data() + (paddr - mem_base), len);
        switch (inst.param.loadstore)
        {
        case isa::RV64LSWidth::byte : RAW_DATA_AS(ret).i64 = *((int8_t*)(&buf)); break;
        case isa::RV64LSWidth::harf : RAW_DATA_AS(ret).i64 = *((int16_t*)(&buf)); break;
        case isa::RV64LSWidth::word : RAW_DATA_AS(ret).i64 = *((int32_t*)(&buf)); break;
        case isa::RV64LSWidth::dword : RAW_DATA_AS(ret).i64 = *((int64_t*)(&buf)); break;
        case isa::RV64LSWidth::ubyte: ret = *((uint8_t*)(&buf)); break;
        case isa::RV64LSWidth::uharf: ret = *((uint16_t*)(&buf)); break;
        case isa::RV64LSWidth::uword: ret = *((uint32_t*)(&buf)); break;
        default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        if(inst.opcode == RV64OPCode::load) {
            if(rd1) reg[rd1] = ret;
        } else {
            reg[rd1+32] = ret;
        }
        LOG_INST("%s -> 0x%lx from @0x%lx (0x%lx)", inst.debug_name_str.c_str(), ret, vaddr, paddr);
    }
    else if(inst.opcode == RV64OPCode::store || inst.opcode == RV64OPCode::storefp) {
        VirtAddrT vaddr = reg[rs1] + RAW_DATA_AS(inst.imm).i64;
        RawDataT buf = ((inst.opcode == RV64OPCode::store)?(reg[rs2]):(reg[rs2+32]));
        uint32_t len = isa::rv64_ls_width_to_length(inst.param.loadstore);
        PhysAddrT paddr = 0;
        CHECK_ADDR_ALIGN(vaddr, len);
        if(!_page_trans_and_check(id, vaddr, PTE_W, &paddr)) {
            RAISE_ITR(ITR_ST_PGFAULT, vaddr);
        }
        memcpy(main_mem.data() + (paddr - mem_base), &buf, len);
        LOG_INST("%s -> 0x%lx to @0x%lx (0x%lx)", inst.debug_name_str.c_str(), buf, vaddr, paddr);
    }
    else if(inst.opcode == RV64OPCode::opimm) {
        RawDataT ret = 0;
        if(SimError::success != isa::perform_int_op_64(inst.param.intop, &ret, reg[rs1], inst.imm)) {
            RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        if(rd1) reg[rd1] = ret;
        LOG_INST("%s -> 0x%lx", inst.debug_name_str.c_str(), ret);
    }
    else if(inst.opcode == RV64OPCode::opimm32) {
        RawDataT ret = 0;
        if(SimError::success != isa::perform_int_op_32(inst.param.intop, &ret, reg[rs1], inst.imm)) {
            RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        if(rd1) reg[rd1] = ret;
        LOG_INST("%s -> 0x%lx", inst.debug_name_str.c_str(), ret);
    }
    else if(inst.opcode == RV64OPCode::op) {
        RawDataT ret = 0;
        if(SimError::success != isa::perform_int_op_64(inst.param.intop, &ret, reg[rs1], reg[rs2])) {
            RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        if(rd1) reg[rd1] = ret;
        LOG_INST("%s -> 0x%lx", inst.debug_name_str.c_str(), ret);
    }
    else if(inst.opcode == RV64OPCode::op32) {
        RawDataT ret = 0;
        if(SimError::success != isa::perform_int_op_32(inst.param.intop, &ret, reg[rs1], reg[rs2])) {
            RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        if(rd1) reg[rd1] = ret;
        LOG_INST("%s -> 0x%lx",inst.debug_name_str.c_str(), ret);
    }
    else if(inst.opcode == RV64OPCode::madd || inst.opcode == RV64OPCode::msub || inst.opcode == RV64OPCode::nmsub || inst.opcode == RV64OPCode::nmadd) {
        RawDataT ret = 0;
        if(SimError::success != isa::perform_fmadd_op(inst.opcode, inst.param.fp.fwid, &ret, reg[rs1+32], reg[rs2+32], reg[rs3+32], &core.fcsr));
        if(rd1) reg[rd1] = ret;
        LOG_INST("%s -> 0x%lx", inst.debug_name_str.c_str(), ret);
    }
    else if(inst.opcode == RV64OPCode::opfp) {
        RawDataT s1 = 0;
        switch(inst.param.fp.op) {
            case isa::RV64FPOP5::ADD :
            case isa::RV64FPOP5::SUB :
            case isa::RV64FPOP5::MUL :
            case isa::RV64FPOP5::DIV :
            case isa::RV64FPOP5::CMP :
            case isa::RV64FPOP5::MIN :
            case isa::RV64FPOP5::SGNJ:
            case isa::RV64FPOP5::SQRT:
            case isa::RV64FPOP5::MVF2F:
            case isa::RV64FPOP5::CVTF2I:
            case isa::RV64FPOP5::MVF2I:
                s1 = reg[rs1+32];
                break;
            case isa::RV64FPOP5::CVTI2F:
            case isa::RV64FPOP5::MVI2F:
                s1 = reg[rs1];
                break;
            default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        RawDataT ret = 0;
        if(SimError::success != isa::perform_fp_op(inst.param.fp, &ret, s1, reg[rs2+32], &core.fcsr)) {
            RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
        }
        if(isa::rv64_fpop_is_i_rd(inst.param.fp.op)) {
            if(rd1) reg[rd1] = ret;
        } else {
            reg[rd1+32] = ret;
        }
        LOG_INST("%s -> 0x%lx", inst.debug_name_str.c_str(), ret);
    }
    else if(inst.opcode == RV64OPCode::miscmem) {

    }
    else if(inst.opcode == RV64OPCode::system) {
        if(inst.flag & RVINSTFLAG_ECALL) {
            RAISE_ITR(ITR_USR_ECALL, core.pc);
            LOG_INST("@0x%lx: ECALL", inst.pc);
        }
        else if(inst.flag & RVINSTFLAG_EBREAK) {
            RAISE_ITR(ITR_BREAK_POINT, core.pc);
            LOG_INST("@0x%lx: EBREAK", inst.pc);
        }
        else {
            using isa::CSRNumber;
            using isa::RV64CSROP3;
            CSRNumber csr_num = (CSRNumber)(inst.param.csr.index & RV64_CSR_NUM_MASK);
            if(csr_num == CSRNumber::fcsr) {
                switch (inst.param.csr.op)
                {
                case RV64CSROP3::CSRRW: if(rd1) reg[rd1] = isa::rv64_csrrw<0,32>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRS: if(rd1) reg[rd1] = isa::rv64_csrrs<0,32>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRC: if(rd1) reg[rd1] = isa::rv64_csrrc<0,32>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRWI: if(rd1) reg[rd1] = isa::rv64_csrrw<0,32>(&core.fcsr, inst.imm); break;
                case RV64CSROP3::CSRRSI: if(rd1) reg[rd1] = isa::rv64_csrrs<0,32>(&core.fcsr, inst.imm); break;
                case RV64CSROP3::CSRRCI: if(rd1) reg[rd1] = isa::rv64_csrrc<0,32>(&core.fcsr, inst.imm); break;
                default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);;
                }
            }
            else if(csr_num == CSRNumber::frm) {
                switch (inst.param.csr.op)
                {
                case RV64CSROP3::CSRRW: if(rd1) reg[rd1] = isa::rv64_csrrw<5,3>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRS: if(rd1) reg[rd1] = isa::rv64_csrrs<5,3>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRC: if(rd1) reg[rd1] = isa::rv64_csrrc<5,3>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRWI: if(rd1) reg[rd1] = isa::rv64_csrrw<5,3>(&core.fcsr, inst.imm); break;
                case RV64CSROP3::CSRRSI: if(rd1) reg[rd1] = isa::rv64_csrrs<5,3>(&core.fcsr, inst.imm); break;
                case RV64CSROP3::CSRRCI: if(rd1) reg[rd1] = isa::rv64_csrrc<5,3>(&core.fcsr, inst.imm); break;
                default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);;
                }
            }
            else if(csr_num == CSRNumber::fflags) {
                switch (inst.param.csr.op)
                {
                case RV64CSROP3::CSRRW: if(rd1) reg[rd1] = isa::rv64_csrrw<0,5>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRS: if(rd1) reg[rd1] = isa::rv64_csrrs<0,5>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRC: if(rd1) reg[rd1] = isa::rv64_csrrc<0,5>(&core.fcsr, reg[rs1]); break;
                case RV64CSROP3::CSRRWI: if(rd1) reg[rd1] = isa::rv64_csrrw<0,5>(&core.fcsr, inst.imm); break;
                case RV64CSROP3::CSRRSI: if(rd1) reg[rd1] = isa::rv64_csrrs<0,5>(&core.fcsr, inst.imm); break;
                case RV64CSROP3::CSRRCI: if(rd1) reg[rd1] = isa::rv64_csrrc<0,5>(&core.fcsr, inst.imm); break;
                default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);;
                }
            }
            else if(csr_num == CSRNumber::cycle) {
                switch (inst.param.csr.op)
                {
                case RV64CSROP3::CSRRW:
                case RV64CSROP3::CSRRS:
                case RV64CSROP3::CSRRC:
                case RV64CSROP3::CSRRWI:
                case RV64CSROP3::CSRRSI:
                case RV64CSROP3::CSRRCI: if(rd1) reg[rd1] = get_current_tick(); break;
                default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);;
                }
            }
            else if(csr_num == CSRNumber::time) {
                switch (inst.param.csr.op)
                {
                case RV64CSROP3::CSRRW:
                case RV64CSROP3::CSRRS:
                case RV64CSROP3::CSRRC:
                case RV64CSROP3::CSRRWI:
                case RV64CSROP3::CSRRSI:
                case RV64CSROP3::CSRRCI: if(rd1) reg[rd1] = get_wall_time_tick(); break;
                default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);;
                }
            }
            else if(csr_num == CSRNumber::instret) {
                switch (inst.param.csr.op)
                {
                case RV64CSROP3::CSRRW:
                case RV64CSROP3::CSRRS:
                case RV64CSROP3::CSRRC:
                case RV64CSROP3::CSRRWI:
                case RV64CSROP3::CSRRSI:
                case RV64CSROP3::CSRRCI: if(rd1) reg[rd1] = core.finished_inst_cnt; break;
                default: RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);;
                }
            }
            else {
                RAISE_ITR(ITR_ILLEGAL_INST, raw_inst);
            }
            LOG_INST("%s -> 0x%lx", inst.debug_name_str.c_str(), reg[rd1]);
        }
    }
    
#undef RAISE_ITR
#undef CHECK_ADDR_ALIGN
}

uint64_t AtomicSMPCores::get_wall_time_tick() {
    uint64_t tick = get_current_tick();
    uint64_t wall_time_freq_mhz = conf::get_int("root", "wall_time_freq_mhz", 1);
    uint64_t global_freq_mhz = conf::get_int("root", "global_freq_mhz", 1000);
    double tick_to_walltime = (double)(wall_time_freq_mhz) / (double)(global_freq_mhz);
    return (uint64_t)((double)tick * tick_to_walltime);
};



}
