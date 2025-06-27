#ifndef RVSIM_SIMROOT_H
#define RVSIM_SIMROOT_H

#include "common.h"

#define simroot_assert(expr) {if(!(static_cast <bool> (expr))) [[unlikely]] { simroot::dump_core(); printf("Failed %s, at file %s, line %d\n", #expr, __FILE__, __LINE__); fflush(stdout); exit(-1);}}

#define simroot_assertf(expr, fmt, ...) {if(!(static_cast <bool> (expr))) [[unlikely]] { simroot::dump_core(); printf(fmt "\n", ##__VA_ARGS__); printf("Failed %s, at file %s, line %d\n", #expr, __FILE__, __LINE__); fflush(stdout); exit(-1);}}

namespace simroot {

void add_trace_object(TraceObject * obj, std::string name);

// --------- 模拟对象管理 ------------

void add_sim_object(SimObject *p_obj, std::string name);

void clear_sim_object();

// --------- 日志管理 ------------

void print_statistic();
void clear_statistic();

void dump_core();

typedef void* LogFileT;

/// @brief 创建一个log文件
/// @param path 文件路径
/// @param linecnt >0:保留最后n行。=0:无效。<0:直接输出每一行到文件
/// @return 
LogFileT create_log_file(string path, int32_t linecnt);

void log_line(LogFileT fd, string &str);
void log_line(LogFileT fd, const char* cstr);

void destroy_log_file(LogFileT fd);

}


namespace test {

bool test_simroot();

}

#endif
