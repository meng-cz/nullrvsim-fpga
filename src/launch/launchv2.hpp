
#include "cpuv2/atmsmp.h"

#include "sysv2/sysv2.h"

#include "serial/serialfpga.h"

#include "simroot.h"
#include "configuration.h"

using simcpu::AtomicSMPCores;

bool mpv2(std::vector<string> &argv) {
    SimWorkload workload;
    workload.argv.assign(argv.begin(), argv.end());
    workload.file_path = argv[0];
    workload.stack_size = (uint64_t)(conf::get_int("workload", "stack_size_mb", 8)) * 1024UL * 1024UL;
    workload.envs.emplace_back(string("LD_LIBRARY_PATH=") + conf::get_str("workload", "ld_path", ""));
    string ldpath = conf::get_str("workload", "ld_path", "");
    {
        std::stringstream ss(ldpath);
        string item = "";
        while (std::getline(ss, item, ';')) {
            if (!item.empty()) {
                workload.ldpaths.push_back(item);
            }
            else {
                break;
            }
        }
    }

    uint32_t cpu_num = conf::get_int("root", "core_num", 1);
    simroot_assert(cpu_num > 0 && cpu_num <= 256);
    uint64_t mem_size = conf::get_int("root", "memory_size_gb", 1);
    simroot_assert(mem_size > 0 && mem_size <= 256);
    mem_size = (mem_size << 30);
    uint64_t mem_base = conf::get_int("root", "memory_base_addr", 0);
    simroot_assert(mem_base > 0 && mem_base + mem_size <= 256);
    simroot_assert((mem_base % PAGE_LEN_BYTE) == 0);

    shared_ptr<AtomicSMPCores> hardwares = make_shared<AtomicSMPCores>(cpu_num, mem_size);
    simroot::add_sim_object(hardwares.get(), "Hardware");
    
    shared_ptr<SMPSystemV2> sys = make_shared<SMPSystemV2>(workload, hardwares.get(), cpu_num, mem_base, mem_size);

    sys->run_sim();

    printf("Simulation Finished with %ld Ticks\n", hardwares->get_current_tick());
    
    simroot::print_statistic();

    return true;
}

bool mpser(string devpath, vector<string> &argv) {
    SimWorkload workload;
    workload.argv.assign(argv.begin(), argv.end());
    workload.file_path = argv[0];
    workload.stack_size = (uint64_t)(conf::get_int("workload", "stack_size_mb", 8)) * 1024UL * 1024UL;
    workload.envs.emplace_back(string("LD_LIBRARY_PATH=") + conf::get_str("workload", "ld_path", ""));
    string ldpath = conf::get_str("workload", "ld_path", "");
    {
        std::stringstream ss(ldpath);
        string item = "";
        while (std::getline(ss, item, ';')) {
            if (!item.empty()) {
                workload.ldpaths.push_back(item);
            }
            else {
                break;
            }
        }
    }

    uint32_t cpu_num = conf::get_int("root", "core_num", 1);
    simroot_assert(cpu_num > 0 && cpu_num <= 256);
    uint64_t mem_size = conf::get_int("root", "memory_size_gb", 1);
    simroot_assert(mem_size > 0 && mem_size <= 256);
    mem_size = (mem_size << 30);
    uint64_t mem_base = conf::get_int("root", "memory_base_addr", 0);
    simroot_assert(mem_base > 0 && mem_base + mem_size <= 256);
    simroot_assert((mem_base % PAGE_LEN_BYTE) == 0);
    uint32_t baudrate = conf::get_int("serial", "baudrate", 115200);

    shared_ptr<SerialFPGAAdapter> hardwares = make_shared<SerialFPGAAdapter>(devpath, baudrate);
    
    shared_ptr<SMPSystemV2> sys = make_shared<SMPSystemV2>(workload, hardwares.get(), cpu_num, mem_base, mem_size);

    sys->run_sim();

    printf("Simulation Finished with %ld Ticks\n", hardwares->get_current_tick());
    
    simroot::print_statistic();

    return true;
}

