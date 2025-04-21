# nullrvsim-fpga

(Working in Progress)

在FPGA上运行用户模式模拟

- 基于RV64GC指令集
- 仅在FPGA上运行核心与内存等部分硬件，无任何外设
- ecall与exception代理到主机执行

## 目前的内容

软件部分：

- 软硬件接口（CPU接口）
- 中断处理支持：ECALL，页错误，非法指令，非法对齐
- Linux ECALL支持：文件IO，虚拟内存，mmap，多线程多进程，futex锁，socket
- 测试用例（RV64GC用户态ELF，可动态链接）（待整理补充）

通讯部分：

- 串口
- PCIE-XDMA（暂未实现）

硬件部分：

- 接口转接模块

## 构建

### 依赖：

- 我用的Linux内核版本：**5.15.\*** *（不同的内核版本可能导致主机系统调用接口不同，详细的版本支持信息还需测试）*
- 编译工具与内核头文件: **gcc, g++, cmake, linux-headers**
- RV交叉编译工具：**riscv64-linux-gnu-gcc**

### 编译：

```bash
mkdir build
cd build
cmake ..
make -j16
```

### 编译后的内容：

- **build/nullrvsim**: 模拟器的可执行文件
- **build/conf/default.ini**: 默认配置文件，可通过命令行参数-c重新指定
- **build/example/\***: 用于测试的RV64GC可执行文件与相关数据文件


## 运行

TODO

## 感谢

使用/包含的项目：
- Easylogging++: [abumq/easyloggingpp](https://github.com/abumq/easyloggingpp)
- ELFIO: [serge1/ELFIO](https://github.com/serge1/ELFIO)




