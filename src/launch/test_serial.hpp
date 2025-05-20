
#include "serial/serialfpga.h"

#include "configuration.h"
#include "simroot.h"

bool test_serial_1(string dev_path) {

    uint32_t baudrate = conf::get_int("serial", "baudrate", 115200);

    uint64_t mem_base = conf::get_int("root", "memory_base_addr", 0);
    simroot_assert((mem_base % PAGE_LEN_BYTE) == 0);

    SerialFPGAAdapter * dev = new SerialFPGAAdapter(dev_path, baudrate);

    printf("Test 1 Start\n");

    printf("Save Reg X5 as 0x1122334455667788\n");
    dev->regacc_write(0, 5, 0x1122334455667788UL);
    
    printf("Read Reg X5\n");
    assert(0x1122334455667788UL == dev->regacc_read(0, 5));

    printf("Write 0x8001_0000 as 0x0102030405060708\n");
    dev->pxymem_write(0, 0x80010000UL, 0x0102030405060708UL);

    printf("Read 0x8001_0000\n");
    assert(0x0102030405060708UL == dev->pxymem_read(0, 0x80010000UL));

    printf("Read Reg X5\n");
    assert(0x1122334455667788UL == dev->regacc_read(0, 5));

    printf("Test 1 PASSED\n");

    return true;
}

bool test_serial_2(string dev_path) {

    uint32_t baudrate = conf::get_int("serial", "baudrate", 115200);

    uint64_t mem_base = conf::get_int("root", "memory_base_addr", 0);
    simroot_assert((mem_base % PAGE_LEN_BYTE) == 0);

    SerialFPGAAdapter * dev = new SerialFPGAAdapter(dev_path, baudrate);

    printf("Test 2 Start\n");

    printf("Set Page 0x8001_0 as 0x8877665544332211\n");
    dev->pxymem_page_set(0, 0x80010UL, 0x8877665544332211UL);

    printf("Write 0x8001_0080 as 0x0505050505050505\n");
    dev->pxymem_write(0, 0x80010080UL, 0x0505050505050505UL);

    printf("Read 0x8001_0000\n");
    assert(0x8877665544332211UL == dev->pxymem_read(0, 0x80010000UL));

    printf("Read 0x8001_0040\n");
    assert(0x8877665544332211UL == dev->pxymem_read(0, 0x80010040UL));

    printf("Read 0x8001_0080\n");
    assert(0x0505050505050505UL == dev->pxymem_read(0, 0x80010080UL));

    printf("Copy Page 0x8001_0 to 0x8004_0\n");
    dev->pxymem_page_copy(0, 0x80040UL, 0x80010UL);
    
    printf("Read 0x8004_0000\n");
    assert(0x8877665544332211UL == dev->pxymem_read(0, 0x80040000UL));

    printf("Read 0x8004_0040\n");
    assert(0x8877665544332211UL == dev->pxymem_read(0, 0x80040040UL));

    printf("Read 0x8004_0080\n");
    assert(0x0505050505050505UL == dev->pxymem_read(0, 0x80040080UL));
    
    printf("Test 2 PASSED\n");

    return true;
}


bool test_serial_3(string dev_path) {

    uint32_t baudrate = conf::get_int("serial", "baudrate", 115200);

    uint64_t mem_base = conf::get_int("root", "memory_base_addr", 0);
    simroot_assert((mem_base % PAGE_LEN_BYTE) == 0);

    SerialFPGAAdapter * dev = new SerialFPGAAdapter(dev_path, baudrate);

    printf("Test 3 Start\n");

    vector<uint64_t> pg;
    pg.assign(512, 0);
    for(uint64_t i = 0; i < 512; i++) pg[i] = i*8 + 0x4000UL;

    printf("Write Page 0x8001_0\n");
    dev->pxymem_page_write(0, 0x80010UL, pg.data());

    printf("Read 0x8001_0000\n");
    assert(0x4000UL == dev->pxymem_read(0, 0x80010000UL));

    printf("Read 0x8001_0100\n");
    assert(0x4100UL == dev->pxymem_read(0, 0x80010100UL));

    printf("Read 0x8001_0400\n");
    assert(0x4400UL == dev->pxymem_read(0, 0x80010400UL));

    printf("Copy Page 0x8001_0 to 0x8004_0\n");
    dev->pxymem_page_copy(0, 0x80040UL, 0x80010UL);
    
    printf("Read 0x8004_0000\n");
    assert(0x4000UL == dev->pxymem_read(0, 0x80040000UL));

    printf("Read 0x8004_0140\n");
    assert(0x4140UL == dev->pxymem_read(0, 0x80040140UL));

    printf("Read 0x8004_0880\n");
    assert(0x4880UL == dev->pxymem_read(0, 0x80040880UL));

    vector<uint64_t> pgrd;
    pgrd.assign(512, 0);
    printf("Read Page 0x8004_0\n");
    dev->pxymem_page_read(0, 0x80040UL, pgrd.data());
    for(uint64_t i = 0; i < 512; i++) assert(pgrd[i] == pg[i]);
    
    printf("Test 3 PASSED\n");

    return true;
}

bool test_serial_4(string dev_path) {

    uint32_t baudrate = conf::get_int("serial", "baudrate", 115200);

    uint64_t mem_base = conf::get_int("root", "memory_base_addr", 0);
    simroot_assert((mem_base % PAGE_LEN_BYTE) == 0);

    SerialFPGAAdapter * dev = new SerialFPGAAdapter(dev_path, baudrate);

    printf("Test 4 Start\n");

    vector<uint32_t> bufs;
    bufs.resize(1024, 0x13);

    printf("Clear Page 0x80000 - 0x80003\n");
    dev->pxymem_page_set(0, 0x80000UL, 0);
    dev->pxymem_page_set(0, 0x80001UL, 0);
    dev->pxymem_page_set(0, 0x80002UL, 0);
    dev->pxymem_page_set(0, 0x80003UL, 0);

    printf("Init PageTable of VPage 0x10000 -> 0x80003\n");
    
    dev->pxymem_write(0, 0x80000000UL + 8 * ((0x10000UL >> 18) & 0x1ff), (0x80001UL << 10) + 1);
    dev->pxymem_write(0, 0x80001000UL + 8 * ((0x10000UL >> 9) & 0x1ff), (0x80002UL << 10) + 1);
    dev->pxymem_write(0, 0x80002000UL + 8 * ((0x10000UL) & 0x1ff), (0x80003UL << 10) + 0xf);
    dev->pxymem_write(0, 0x80003000UL, 0x06400893UL | (0x00000073UL << 32));

    printf("Setup MMU\n");
    dev->set_mmu(0, 0x80000000UL, 0);

    printf("Start ILA Trigger, and Type \"1\" to Continue...\n");
    do {
        if(getchar() == '1') break;
    } while(true);

    printf("\nRedirect to VAddr 0x10000000\n");
    dev->redirect(0, 0x10000000UL);

    uint32_t cpuid = 0;
    VirtAddrT pc = 0;
    uint32_t cause = 0;
    uint64_t arg = 0;
    assert(dev->next(&cpuid, &pc, &cause, &arg));

    printf("Got Event on CPU %d, @0x%lx, Cause %d, Arg 0x%lx\n", cpuid, pc, cause, arg);

    printf("Test 4 PASSED\n");

    return true;
}

