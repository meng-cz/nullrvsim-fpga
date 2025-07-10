
#include "serialfpga.h"
#include "simroot.h"
#include "configuration.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>

#include <sys/stat.h>
#include <sys/types.h>

int32_t get_baudrate_const(int baudrate) {
    switch(baudrate) {
        case 0:      return B0;
        case 50:     return B50;
        case 75:     return B75;
        case 110:    return B110;
        case 134:    return B134;
        case 150:    return B150;
        case 200:    return B200;
        case 300:    return B300;
        case 600:    return B600;
        case 1200:   return B1200;
        case 1800:   return B1800;
        case 2400:   return B2400;
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default:     return (speed_t)-1;
    }
}

SerialFPGAAdapter::SerialFPGAAdapter(string devfile, uint32_t baudrate) {

    dbg = conf::get_int("serial", "debug_enable", 0);

    fd = open(devfile.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    simroot_assertf(fd > 0, "Open Serial Device %s Failed", devfile.c_str());

    struct termios tio;
    bzero(&tio, sizeof(tio) );
    simroot_assertf(tcgetattr(fd,&tio) == 0, "Setup Serail Device %s Failed", devfile.c_str());

    int rate = get_baudrate_const(baudrate);
    simroot_assertf(rate > 0, "Unknown Serial Speed %d", baudrate);
    simroot_assertf(cfsetospeed(&tio, rate) == 0, "Setup Baudrate %d for Device %s Failed", baudrate, devfile.c_str());
    simroot_assertf(cfsetispeed(&tio, rate) == 0, "Setup Baudrate %d for Device %s Failed", baudrate, devfile.c_str());
    
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;

    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_oflag &= ~OPOST;

    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;

    simroot_assertf(tcsetattr(fd,TCSANOW,&tio) == 0, "Setup Serail Device %s Failed", devfile.c_str());

    tcflush(fd,TCIFLUSH);
}

SerialFPGAAdapter::~SerialFPGAAdapter() {
    if(fd) close(fd);
}

void SerialFPGAAdapter::_read_serial(void * buf, uint64_t size) {
    int64_t sz = 0;
    while(sz < size) {
        int64_t ret = read(fd, (uint8_t*)(buf) + sz, size - sz);
        simroot_assertf(ret >= 0, "Read Serial Failed: %ld", ret);
        if(dbg) {
            printf("Read Serial [%ld]:", ret);
            for(uint64_t i = 0; i < ret; i++) printf(" %02x", ((uint8_t*)buf)[sz + i] & 0xff);
            printf("\n");
        }
        sz += ret;
    }
}

void SerialFPGAAdapter::_write_serial(void * buf, uint64_t size) {
    if(dbg) {
        printf("Write Serial [%ld]:", size);
        for(uint64_t i = 0; i < size; i++) printf(" %02x", ((uint8_t*)buf)[i] & 0xff);
        printf("\n");
    }
    int64_t ret = write(fd, buf, size);
    simroot_assertf(ret == size, "Write Serial Failed: %ld", ret);
}

void SerialFPGAAdapter::_append_int(BufT &buf, uint64_t data, uint64_t bytes) {
    simroot_assert(bytes > 0 && bytes <= 8);
    uint64_t sz = buf.size();
    buf.resize(sz + bytes);
    memcpy(buf.data() + sz, &data, bytes);
}
void SerialFPGAAdapter::_append_buf(BufT &buf, void * data, uint64_t bytes) {
    uint64_t sz = buf.size();
    buf.resize(sz + bytes);
    memcpy(buf.data() + sz, data, bytes);
}
uint64_t SerialFPGAAdapter::_pop_int(BufT &buf, uint64_t bytes) {
    uint64_t ret = 0;
    simroot_assert(bytes > 0 && bytes <= 8);
    memcpy(&ret, buf.data(), bytes);
    if(buf.size() > bytes) memmove(buf.data(), buf.data() + bytes, buf.size() - bytes);
    buf.resize(buf.size() - bytes);
    return ret;
}

const uint64_t SEROP_RET_BITS[SEROP_NUM] = {
    16+8+48+48,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    64,
    0,
    64,
    0,
    512*8,
    0,
    0,
    0,
    64,
    64
};

int8_t SerialFPGAAdapter::_perform_op(int8_t op, BufT &data, BufT &retdata) {
    int8_t raw_op = (op & 0x1f);
    simroot_assert(raw_op < SEROP_NUM && raw_op >= 0);
    _write_serial(&op, 1);
    _write_serial(data.data(), data.size());
    retdata.resize(SEROP_RET_BITS[raw_op] / 8);
    int8_t ret = 0;
    _read_serial(&ret, 1);
    _read_serial(retdata.data(), retdata.size());
    return ret;
}

#define DEBUGOP(fmt, ...) do { if(debug_op) printf("SERIAL FPGA CPU %d: " fmt "\n", cpu_id, ##__VA_ARGS__); } while(0)

void SerialFPGAAdapter::halt(uint32_t cpu_id) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    int8_t value = _perform_op(SEROP_HALT, buf, ret);
    simroot_assertf(SEROP_HALT == value, "Operation Halt on Core %d Failed: %d", cpu_id, value);
    DEBUGOP("Halt");
}

void SerialFPGAAdapter::interrupt(uint32_t cpu_id) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    int8_t value = _perform_op(SEROP_ITR, buf, ret);
    simroot_assertf(SEROP_ITR == value, "Operation Interrupt on Core %d Failed: %d", cpu_id, value);
    DEBUGOP("Interrupt");
}

void SerialFPGAAdapter::set_mmu(uint32_t cpu_id, PhysAddrT pgtable, AsidT asid) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, asid, 2);
    _append_int(buf, pgtable >> PAGE_ADDR_OFFSET, 5);
    int8_t value = _perform_op(SEROP_MMU, buf, ret);
    simroot_assertf(SEROP_MMU == value, "Operation SetMMU on Core %d (0x%lx, 0x%d) Failed: %d", cpu_id, pgtable >> PAGE_ADDR_OFFSET, asid, value);
    DEBUGOP("SetMMU (0x%lx, 0x%d)", pgtable >> PAGE_ADDR_OFFSET, asid);
}

void SerialFPGAAdapter::redirect(uint32_t cpu_id, VirtAddrT addr) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, addr, 6);
    int8_t value = _perform_op(SEROP_REDIR, buf, ret);
    simroot_assertf(SEROP_REDIR == value, "Operation Redirect on Core %d (0x%lx) Failed: %d", cpu_id, addr, value);
    DEBUGOP("Redirect (0x%lx)", addr);
}

bool SerialFPGAAdapter::next(uint32_t *itr_cpu, VirtAddrT *itr_pc, uint32_t *itr_cause, RawDataT *itr_arg) {
    vector<uint8_t> buf, ret;
    int8_t value = _perform_op(SEROP_NEXT, buf, ret);
    simroot_assertf(SEROP_NEXT == value, "Operation Next Failed: %d", value);
    *itr_cpu = _pop_int(ret, 2);
    *itr_cause = _pop_int(ret, 1);
    *itr_pc = _pop_int(ret, 6);
    *itr_arg = _pop_int(ret, 6);
    uint32_t cpu_id = *itr_cpu;
    DEBUGOP("Next -> (0x%lx, %d, 0x%lx)", *itr_pc, *itr_cause, *itr_arg);
    return (*itr_cpu != 0xffff);
}

void SerialFPGAAdapter::flush_tlb_all(uint32_t cpu_id) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    int8_t value = _perform_op(SEROP_FTLB, buf, ret);
    simroot_assertf(SEROP_FTLB == value, "Operation FlushTLB on Core %d Failed: %d", cpu_id, value);
    DEBUGOP("FlushTLB");
}

void SerialFPGAAdapter::flush_tlb_vpgidx(uint32_t cpu_id, VirtAddrT vaddr, AsidT asid) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, asid, 2);
    _append_int(buf, vaddr >> PAGE_ADDR_OFFSET, 5);
    int8_t value = _perform_op(SEROP_FTLB2, buf, ret);
    simroot_assertf(SEROP_FTLB2 == value, "Operation FlushTLB2 on Core %d (%d, 0x%lx) Failed: %d", cpu_id, asid, vaddr >> PAGE_ADDR_OFFSET, value);
    DEBUGOP("FlushTLB2 (%d, 0x%lx)", asid, vaddr >> PAGE_ADDR_OFFSET);
}


void SerialFPGAAdapter::sync_inst_stream(uint32_t cpu_id) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    int8_t value = _perform_op(SEROP_SYNCI, buf, ret);
    simroot_assertf(SEROP_SYNCI == value, "Operation SyncI on Core %d Failed: %d", cpu_id, value);
    DEBUGOP("SyncI");
}

RawDataT SerialFPGAAdapter::regacc_read(uint32_t cpu_id, RVRegIndexT vreg) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, vreg, 2);
    int8_t value = _perform_op(SEROP_REGRD, buf, ret);
    simroot_assertf(SEROP_REGRD == value, "Operation RegRead on Core %d (%d) Failed: %d", cpu_id, vreg, value);
    uint64_t rvalue = _pop_int(ret, 8);
    DEBUGOP("RegRead (%d) -> 0x%lx", vreg, rvalue);
    return rvalue;
}

void SerialFPGAAdapter::regacc_write(uint32_t cpu_id, RVRegIndexT vreg, RawDataT data) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, vreg, 2);
    _append_int(buf, data, 8);
    int8_t value = _perform_op(SEROP_REGWT, buf, ret);
    simroot_assertf(SEROP_REGWT == value, "Operation RegWrite on Core %d (%d, 0x%lx) Failed: %d", cpu_id, vreg, data, value);
    DEBUGOP("RegWrite (%d, 0x%lx)", vreg, data);
}

RawDataT SerialFPGAAdapter::pxymem_read(uint32_t cpu_id, PhysAddrT paddr) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, paddr, 6);
    int8_t value = _perform_op(SEROP_MEMRD, buf, ret);
    simroot_assertf(SEROP_MEMRD == value, "Operation MemRead on Core %d (0x%lx) Failed: %d", cpu_id, paddr, value);
    uint64_t rvalue = _pop_int(ret, 8);
    DEBUGOP("MemRead (0x%lx) -> 0x%lx", paddr, rvalue);
    return rvalue;
}

void SerialFPGAAdapter::pxymem_write(uint32_t cpu_id, PhysAddrT paddr, RawDataT data) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, paddr, 6);
    _append_int(buf, data, 8);
    int8_t value = _perform_op(SEROP_MEMWT, buf, ret);
    simroot_assertf(SEROP_MEMWT == value, "Operation MemWrite on Core %d (0x%lx, 0x%lx) Failed: %d", cpu_id, paddr, data, value);
    DEBUGOP("MemWrite (0x%lx, 0x%lx)", paddr, data);
}

void SerialFPGAAdapter::pxymem_page_read(uint32_t cpu_id, PageIndexT ppn, void * dbuf) {
    for(int32_t i = 0; i < 8; i++) {
        vector<uint8_t> buf, ret;
        _append_int(buf, cpu_id, 2);
        _append_int(buf, ppn, 5);
        int8_t op = (SEROP_PGRD | (i << 5));
        int8_t value = _perform_op(op, buf, ret);
        simroot_assertf(op == value, "Operation PageRead on Core %d (0x%lx) Failed: %d", cpu_id, ppn, value);
        simroot_assert(ret.size() == PAGE_LEN_BYTE/8);
        memcpy((uint8_t*)dbuf + (i*PAGE_LEN_BYTE/8), ret.data(), PAGE_LEN_BYTE/8);
    }
    DEBUGOP("PageRead (0x%lx)", ppn);
}

void SerialFPGAAdapter::pxymem_page_set(uint32_t cpu_id, PageIndexT ppn, RawDataT value) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, ppn, 5);
    _append_int(buf, value, 8);
    int8_t rvalue = _perform_op(SEROP_PGST, buf, ret);
    simroot_assertf(SEROP_PGST == rvalue, "Operation PageSet on Core %d (0x%lx, 0x%lx) Failed: %d", cpu_id, ppn, value, rvalue);
    DEBUGOP("PageSet (0x%lx, 0x%lx)", ppn, value);
}

void SerialFPGAAdapter::pxymem_page_write(uint32_t cpu_id, PageIndexT ppn, void * dbuf) {
    for(int32_t i = 0; i < 8; i++) {
        vector<uint8_t> buf, ret;
        _append_int(buf, cpu_id, 2);
        _append_int(buf, ppn, 5);
        _append_buf(buf, (uint8_t*)dbuf + (i*PAGE_LEN_BYTE/8), PAGE_LEN_BYTE/8);
        int8_t op = (SEROP_PGWT | (i << 5));
        int8_t value = _perform_op(op, buf, ret);
        simroot_assertf(op == value, "Operation PageWrite on Core %d (0x%lx) Failed: %d", cpu_id, ppn, value);
    }
    DEBUGOP("PageWrite (0x%lx)", ppn);
}

void SerialFPGAAdapter::pxymem_page_copy(uint32_t cpu_id, PageIndexT dst, PageIndexT src) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    _append_int(buf, dst, 5);
    _append_int(buf, src, 5);
    int8_t value = _perform_op(SEROP_PGCP, buf, ret);
    simroot_assertf(SEROP_PGCP == value, "Operation PageCopy on Core %d (0x%lx, 0x%lx) Failed: %d", cpu_id, dst, src, value);
    DEBUGOP("PageCopy (0x%lx -> 0x%lx)", src, dst);
}

uint64_t SerialFPGAAdapter::get_current_tick() {
    vector<uint8_t> buf, ret;
    int8_t value = _perform_op(SEROP_CLK, buf, ret);
    simroot_assertf(SEROP_CLK == value, "Operation Clock Failed: %d", value);
    uint64_t rvalue = _pop_int(ret, 8);
    uint32_t cpu_id = 0;
    DEBUGOP("Clock -> %ld", rvalue);
    return rvalue;
}

uint64_t SerialFPGAAdapter::get_current_utick(uint32_t cpu_id) {
    vector<uint8_t> buf, ret;
    _append_int(buf, cpu_id, 2);
    int8_t value = _perform_op(SEROP_UCLK, buf, ret);
    simroot_assertf(SEROP_UCLK == value, "Operation UClock on Core %d Failed: %d", cpu_id, value);
    uint64_t rvalue = _pop_int(ret, 8);
    DEBUGOP("UClock (0x%d) -> %ld", cpu_id, rvalue);
    return rvalue;
}


void SerialFPGAAdapter::dump_core(std::ofstream &ofile) {
    char logbuf[256];
    uint32_t cpu_num = conf::get_int("root", "core_num", 1);
    for(uint32_t i = 0; i < cpu_num; i++) {
        sprintf(logbuf, "CPU %d Regs:\n", i);
        ofile << logbuf;
        for(uint32_t r = 0; r < RV_REG_CNT_INT; r+=2) {
            sprintf(logbuf, "%02d-%s: 0x%16lx, %02d-%s: 0x%16lx\n",
                r, isa::ireg_name(r), (r?regacc_read(i, r):0),
                r+1, isa::ireg_name(r+1), regacc_read(i, r+1)
            );
            ofile << logbuf;
        }
    }
}
