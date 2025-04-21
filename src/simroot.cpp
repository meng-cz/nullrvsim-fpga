#include "simroot.h"
#include "configuration.h"
#include "spinlocks.h"

#include <filesystem>

namespace simroot {

typedef struct {
    std::ofstream *ofile = nullptr;
    std::list<string> buf;
    uint64_t lineoffset = 0;
    int32_t linecnt = 0;
} LogFile;

inline void log_file_close(LogFile *p) {
    if(p->linecnt > 0) {
        for(auto &str : p->buf) {
            *(p->ofile) << str << "\n";
        }
    }
    p->ofile->close();
    delete p;
}

vector<std::pair<string, SimObject*>> sim_objects;
std::set<LogFile*> logfiles;

void int_signal_handler(int signum) {
    printf("Recieve SIGINT\n");
    dump_core();
    exit(-1);
}

bool simroot_isinit = false;

void init_simroot() {
    if(!simroot_isinit) {
        std::string log_dir = conf::get_str("root", "out_dir", "out");
        std::filesystem::create_directories(log_dir);
        signal(SIGINT, int_signal_handler);
    }
    simroot_isinit = true;
}

void add_sim_object(SimObject *p_obj, std::string name) {
    init_simroot();
    sim_objects.emplace_back(name, p_obj);
}

void clear_sim_object() {
    init_simroot();
    sim_objects.clear();
}

void print_statistic() {
    if(simroot_isinit) {
        std::string log_dir = conf::get_str("root", "out_dir", "out");
        std::ofstream statistic_log_file(log_dir + "/statistic.txt", std::ios::out);
        for(auto &entry : sim_objects) {
            statistic_log_file << entry.first << std::string("\n");
            entry.second->print_statistic(statistic_log_file);
            statistic_log_file << std::endl;
        }
        statistic_log_file.close();
    }
}

void clear_statistic() {
    if(simroot_isinit) {
        for(auto &entry : sim_objects) {
            entry.second->clear_statistic();
        }
    }
}

void dump_core() {
    init_simroot();
    string path = conf::get_str("root", "core_path", "core.txt");
    std::ofstream ofile(path);

    for(auto &entry: sim_objects) {
        ofile << entry.first << ":\n";
        entry.second->dump_core(ofile);
        ofile << std::endl;
    }

    for(auto &f : logfiles) {
        log_file_close(f);
    }
    logfiles.clear();

    printf("Core file at %s\n", path.c_str());
}

LogFileT create_log_file(string path, int32_t linecnt) {
    init_simroot();
    LogFile *ret = new LogFile;
    ret->ofile = new std::ofstream(path);
    ret->linecnt = linecnt;
    logfiles.insert(ret);
    return (LogFileT)ret;
}

void log_line(LogFileT fd, string &str) {
    LogFile *p = (LogFile *)fd;
    if(p->linecnt > 0) {
        p->buf.push_back(str);
        if(p->buf.size() > p->linecnt) {
            p->buf.pop_front();
            p->lineoffset++;
        }
    }
    else if(p->linecnt < 0) {
        *(p->ofile) << str << std::endl;
    }
}
void log_line(LogFileT fd, const char* cstr) {
    LogFile *p = (LogFile *)fd;
    if(p->linecnt > 0) {
        p->buf.push_back(cstr);
        if(p->buf.size() > p->linecnt) {
            p->buf.pop_front();
            p->lineoffset++;
        }
    }
    else if(p->linecnt < 0) {
        *(p->ofile) << cstr << std::endl;
    }
}

void destroy_log_file(LogFileT fd) {
    LogFile *p = (LogFile *)fd;
    logfiles.erase(p);
    log_file_close(p);
}

}


namespace test{


bool test_simroot() {
    
    return false;
}

}
