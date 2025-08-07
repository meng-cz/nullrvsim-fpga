
package nulctrl

import chisel3._
import chisel3.util._

class NulCPUBundle extends Bundle {
    val inited          = Input (Bool())
    val priv            = Input (UInt(2.W))     // 核心实时的特权级状态（U:0, S:1, M:3）
    val ext_itr         = Output(Bool())        // 拉高时：触发核心外部中断
    val stop_fetch      = Output(Bool())        // 拉高时：核心停止向后端发送取到的指令，后端仅能接受由inst64接口注入的指令

    val regacc_rd       = Output(Bool())        // 拉高时：请求一次寄存器读，与regacc_wt互斥
    val regacc_wt       = Output(Bool())        // 拉高时：请求一次寄存器写，与regacc_rd互斥
    val regacc_idx      = Output(UInt(5.W))     // 逻辑寄存器号
    val regacc_wdata    = Output(UInt(64.W))    // 寄存器写数据
    val regacc_rdata    = Input (UInt(64.W))    // 寄存器读数据
    val regacc_busy     = Input (Bool())        // 拉高时：寄存器操作无法在当前周期完成，需要等待。输入状态会被保持至该信号拉低

    val inst64          = Output(Bool())        // 拉高时：向后端注入一条RV64指令（M模式），仅包含OP, OPIMM, LD, SD, CSR, MRET
    val inst64_raw      = Output(UInt(32.W))    // 指令二进制
    val inst64_nowait   = Output(Bool())
    val inst64_ready    = Input (Bool())        // 拉高时：后端可以接受一条指令
    val inst64_flush    = Output(Bool())        // 拉高时：等待所有指令执行完成
    val inst64_busy     = Input (Bool())        // 拉高时：后端存在至少一条正在执行的指令
}

class UartIO extends DecoupledIO(UInt(8.W)) {}

class Tx(frequency: Int, baudRate: Int) extends Module {
    val io = IO(new Bundle {
        val txd = Output(UInt(1.W))
        val channel = Flipped(new UartIO())
    })

    val BIT_CNT = ((frequency + baudRate / 2) / baudRate - 1).asUInt
    // val BIT_CNT = (frequency/baudRate - 1).asUInt

    val shiftReg = RegInit(0x7ff.U)
    val cntReg = RegInit(0.U(20.W))
    val bitsReg = RegInit(0.U(4.W))

    io.channel.ready := (cntReg === 0.U) && (bitsReg === 0.U)
    io.txd := shiftReg(0)

    when(cntReg === 0.U) {

        cntReg := BIT_CNT
        when(bitsReg =/= 0.U) {
            val shift = shiftReg >> 1
            shiftReg := 1.U ## shift(9, 0)
            bitsReg := bitsReg - 1.U
        } .otherwise {
            when(io.channel.valid) {
                // two stop bits, data, one start bit
                shiftReg := 3.U ## io.channel.bits ## 0.U
                bitsReg := 11.U
            } .otherwise {
                shiftReg := 0x7ff.U
            }
        }

    } .otherwise {
        cntReg := cntReg - 1.U
    }
}

class Rx(frequency: Int, baudRate: Int) extends Module {
    val io = IO(new Bundle {
        val rxd = Input(UInt(1.W))
        val channel = new UartIO()
    })

    val BIT_CNT = ((frequency + baudRate / 2) / baudRate - 1).U
    val START_CNT = ((3 * frequency / 2 + baudRate / 2) / baudRate - 1).U
    // val BIT_CNT = (frequency/baudRate - 1).asUInt
    // val START_CNT = ((3 * frequency) / (2 * baudRate) - 1).U

    // Sync in the asynchronous RX data, reset to 1 to not start reading after a reset
    val rxReg = RegNext(RegNext(io.rxd, 1.U), 1.U)

    val shiftReg = RegInit(0.U(8.W))
    val cntReg = RegInit(0.U(20.W))
    val bitsReg = RegInit(0.U(4.W))
    val validReg = RegInit(false.B)

    when(cntReg =/= 0.U) {
        cntReg := cntReg - 1.U
    } .elsewhen(bitsReg =/= 0.U) {
        cntReg := BIT_CNT
        shiftReg := rxReg ## (shiftReg >> 1)
        bitsReg := bitsReg - 1.U
        // the last bit shifted in
        when(bitsReg === 1.U) {
        validReg := true.B
        }
    } .elsewhen(rxReg === 0.U) {
        // wait 1.5 bits after falling edge of start
        cntReg := START_CNT
        bitsReg := 8.U
    }

    when(validReg && io.channel.ready) {
        validReg := false.B
    }

    io.channel.bits := shiftReg
    io.channel.valid := validReg
}
