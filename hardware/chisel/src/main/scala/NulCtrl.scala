
package nulctrl

import chisel3._
import chisel3.util._

class NulCPUBundle extends Bundle {
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
    val inst64_ready    = Input (Bool())        // 拉高时：后端可以接受一条指令
    val inst64_flush    = Output(Bool())        // 拉高时：等待所有指令执行完成
    val inst64_busy     = Input (Bool())        // 拉高时：后端存在至少一条正在执行的指令
}

class UartIO extends DecoupledIO(UInt(8.W)) {}

class NulCPUCtrl() extends Module {

    val io = IO(new Bundle {
        val cpu     = new NulCPUBundle()
        val tx      = new UartIO()
        val rx      = Flipped(new UartIO())
        val dgb_sta = Output(UInt(8.W))
    })

    io.rx.ready := true.B
    io.tx.valid := false.B 
    io.tx.bits := 0.U 

    val CPU_HALT = 0.U 
    val CPU_ITR  = 1.U 
    val CPU_USER = 2.U 
    val cpu_state = RegInit(0.U(2.W))

    io.cpu.ext_itr := false.B
    io.cpu.regacc_rd := false.B 
    io.cpu.regacc_wt := false.B 
    io.cpu.regacc_idx := 0.U 
    io.cpu.regacc_wdata := 0.U 
    io.cpu.inst64 := false.B 
    io.cpu.inst64_raw := 0.U 
    io.cpu.inst64_flush := false.B 

    val global_clk = RegInit(0.U(64.W))
    val user_clk = RegInit(0.U(64.W))

    global_clk := global_clk + 1.U
    when(io.cpu.priv === 0.U) {
        user_clk := user_clk + 1.U
    }

    val eq_valid = RegInit(false.B)
    
    val last_priv = RegNext(io.cpu.priv, 3.U(2.W))
    when(last_priv === 0.U && io.cpu.priv =/= 0.U && cpu_state === CPU_USER) {
        eq_valid := true.B 
        cpu_state := CPU_ITR
    }
    io.cpu.stop_fetch := (cpu_state =/= CPU_USER) || (last_priv === 0.U && io.cpu.priv =/= 0.U)
    
    val is_sv48 = false

    val SEROP_NEXT  = 0.U
    val SEROP_HALT  = 1.U
    val SEROP_ITR   = 2.U
    val SEROP_MMU   = 3.U
    val SEROP_REDIR = 4.U
    val SEROP_FTLB  = 5.U
    val SEROP_FTLB2 = 6.U
    val SEROP_SYNCI = 7.U
    val SEROP_REGRD = 8.U
    val SEROP_REGWT = 9.U
    val SEROP_MEMRD = 10.U
    val SEROP_MEMWT = 11.U
    val SEROP_PGRD  = 12.U
    val SEROP_PGST  = 13.U
    val SEROP_PGWT  = 14.U
    val SEROP_PGCP  = 15.U
    val SEROP_CLK   = 16.U

    val STATE_INIT_WAIT     = 0.U 
    val STATE_DO_INIT       = 1.U

    val STATE_RECV_HEAD     = 2.U
    val STATE_RECV_ARG      = 3.U
    val STATE_SEND_HEAD     = 4.U 
    val STATE_SEND_ARG      = 5.U 

    val STATE_WAIT_NEXT     = 8.U 
    val STATE_NEXT          = 9.U 
    val STATE_MMU           = 10.U 
    val STATE_REDIR         = 11.U 
    val STATE_FLUSH         = 12.U 
    val STATE_REGRD         = 13.U 
    val STATE_REGWT         = 14.U 
    val STATE_MEMRD         = 15.U 
    val STATE_MEMWT         = 16.U 
    val STATE_PGRD          = 17.U 
    val STATE_PGST          = 18.U 
    val STATE_PGWT          = 19.U 
    val STATE_PGCP          = 20.U 
    val STATE_SYNCI         = 21.U 

    val state = RegInit(0.U(5.W))
    val trans_bytes = RegInit(0.U(10.W))
    val trans_pos = RegInit(0.U(10.W))

    io.dgb_sta := state 

    val opcode = RegInit(0.U(5.W))
    val opoff = RegInit(0.U(3.W))
    val oparg = RegInit(VecInit(Seq.fill(16)(0.U(8.W))))
    val retarg = RegInit(VecInit(Seq.fill(16)(0.U(8.W))))

    when(state === STATE_INIT_WAIT && io.cpu.priv === 3.U) {
        state := STATE_DO_INIT
    }

    when(state === STATE_RECV_HEAD && io.rx.valid) {
        val rxop = io.rx.bits(4, 0)
        val rxoff = io.rx.bits(7, 5)
        opcode := rxop
        opoff := rxoff

        when(rxop === SEROP_HALT || rxop === SEROP_ITR || rxop === SEROP_FTLB || rxop === SEROP_SYNCI) {
            trans_bytes := 2.U 
        }.elsewhen(rxop === SEROP_REGRD) {
            trans_bytes := 4.U 
        }.elsewhen(rxop === SEROP_PGRD || rxop === SEROP_PGWT) {
            trans_bytes := 7.U 
        }.elsewhen(rxop === SEROP_REDIR || rxop === SEROP_MEMRD) {
            trans_bytes := 8.U 
        }.elsewhen(rxop === SEROP_MMU || rxop === SEROP_FTLB2) {
            trans_bytes := 9.U 
        }.elsewhen(rxop === SEROP_REGWT || rxop === SEROP_PGCP) {
            trans_bytes := 12.U 
        }.elsewhen(rxop === SEROP_PGST) {
            trans_bytes := 15.U 
        }.elsewhen(rxop === SEROP_MEMWT) {
            trans_bytes := 16.U 
        }.otherwise {
            trans_bytes := 0.U 
        }

        trans_pos := 0.U
        state := STATE_RECV_ARG
    }

    when(state === STATE_RECV_ARG && trans_bytes === trans_pos) {
        trans_bytes := 0.U
        trans_pos := 0.U
        switch(opcode) {
            is(SEROP_NEXT) { state := STATE_WAIT_NEXT }
            is(SEROP_HALT) {
                io.cpu.ext_itr := true.B
                cpu_state := CPU_HALT
                state := STATE_SEND_HEAD
            }
            is(SEROP_ITR) {
                io.cpu.ext_itr := true.B
                state := STATE_SEND_HEAD
            }
            is(SEROP_MMU) { state := STATE_MMU }
            is(SEROP_REDIR) { state := STATE_REDIR }
            is(SEROP_FTLB) { state := STATE_FLUSH }
            is(SEROP_FTLB2) { state := STATE_FLUSH }
            is(SEROP_SYNCI) { state := STATE_SYNCI }
            is(SEROP_REGRD) { state := STATE_REGRD }
            is(SEROP_REGWT) { state := STATE_REGWT }
            is(SEROP_MEMRD) { state := STATE_MEMRD }
            is(SEROP_MEMWT) { state := STATE_MEMWT }
            is(SEROP_PGRD) { state := STATE_PGRD }
            is(SEROP_PGST) { state := STATE_PGST }
            is(SEROP_PGWT) { state := STATE_PGWT }
            is(SEROP_PGCP) { state := STATE_PGCP }
            is(SEROP_CLK) {
                state := STATE_SEND_HEAD
                for(i <- 0 to 7) {
                    retarg(i) := global_clk(i*8+7, i*8)
                }
            }
        }
    }.elsewhen(state === STATE_RECV_ARG && io.rx.valid) {
        oparg(trans_pos) := io.rx.bits
        trans_pos := trans_pos + 1.U
    }

    when(state === STATE_SEND_HEAD) {
        io.tx.valid := true.B 
        io.tx.bits := Cat(opoff, opcode)
        trans_pos := 0.U
        when(io.tx.ready) {
            when(opcode === SEROP_NEXT) {
                trans_bytes := 15.U
                state := STATE_SEND_ARG
            }.elsewhen(opcode === SEROP_REGRD) {
                trans_bytes := 8.U
                state := STATE_SEND_ARG
            }.elsewhen(opcode === SEROP_MEMRD) {
                trans_bytes := 8.U
                state := STATE_SEND_ARG
            }.elsewhen(opcode === SEROP_CLK) {
                trans_bytes := 8.U
                state := STATE_SEND_ARG
            }.otherwise {
                trans_bytes := 0.U
                state := STATE_RECV_HEAD
            }
        }
    }

    when(state === STATE_SEND_ARG) {
        io.tx.valid := true.B
        io.tx.bits := retarg(trans_pos)
        when(io.tx.ready) {
            when(trans_pos + 1.U === trans_bytes) {
                trans_pos := 0.U
                trans_bytes := 0.U
                state := STATE_RECV_HEAD
            }.otherwise {
                trans_pos := trans_pos + 1.U
            }
        }
    }

    when(state === STATE_WAIT_NEXT && eq_valid) {
        eq_valid := false.B
        state := STATE_NEXT
    }

    val satp_back = RegInit(0.U(64.W))

    when(state === STATE_MMU) {
        val satp_mode = if(is_sv48) 9.U(4.W) else 8.U(4.W)
        satp_back := Cat(
            satp_mode,
            oparg(3),
            oparg(2),
            (0.U(4.W)),
            oparg(8),
            oparg(7),
            oparg(6),
            oparg(5),
            oparg(4)
        )
        state := STATE_SEND_HEAD
    }

    val cnt = RegInit(1.U(128.W))
    val regback = RegInit(VecInit(Seq.fill(10)(0.U(64.W))))

    val reg_idxs = Array(5.U, 6.U, 7.U, 8.U, 9.U, 18.U, 19.U, 20.U, 21.U, 22.U)

    def read_reg(idx: Int, dst: UInt) {
        io.cpu.regacc_rd := true.B
        io.cpu.regacc_idx := reg_idxs(idx)
        when(!io.cpu.regacc_busy) {
            cnt := (cnt << 1)
            dst := io.cpu.regacc_rdata
        }
    }
    def read_reg_to_retarg(idx: Int, pos: Int, bytes: Int) {
        io.cpu.regacc_rd := true.B
        io.cpu.regacc_idx := reg_idxs(idx)
        when(!io.cpu.regacc_busy) {
            cnt := (cnt << 1)
            for(i <- 0 to (bytes-1)) {
                retarg(pos + i) := (io.cpu.regacc_rdata >> i*8)(7, 0)
            }
        }
    }
    def write_reg(idx: Int, src: UInt) {
        io.cpu.regacc_wt := true.B
        io.cpu.regacc_idx := reg_idxs(idx)
        io.cpu.regacc_wdata := src
        when(!io.cpu.regacc_busy) {
            cnt := (cnt << 1)
        }
    }
    def write_reg_from_oparg(idx: Int, pos: Int, bytes: Int) {
        io.cpu.regacc_wt := true.B
        io.cpu.regacc_idx := reg_idxs(idx)
        val _regacc_wdata = Wire(Vec(8, UInt(8.W)))
        for(i <- 0 to (bytes-1)) {
            _regacc_wdata(i) := oparg(pos + i)
        }
        if(bytes < 8) {
            for(i <- bytes to 7) {
                _regacc_wdata(i) := 0.U(8.W)
            }
        }
        io.cpu.regacc_wdata := _regacc_wdata.asUInt()
        when(!io.cpu.regacc_busy) {
            cnt := (cnt << 1)
        }
    }
    def invoke_inst(inst: UInt) {
        io.cpu.inst64 := true.B
        io.cpu.inst64_raw := inst
        when(io.cpu.inst64_ready) {
            cnt := (cnt << 1)
        }
    }
    def wait_inst() {
        io.cpu.inst64_flush := true.B 
        when(!io.cpu.inst64_busy) {
            cnt := (cnt << 1)
        }
    }
    def backup_regs(idx: Int, nums: Int) {
        for(i <- 0 until nums) {
            when(cnt(i + idx)) { read_reg(i, regback(i)) }
        }
    }
    def recover_regs(idx: Int, nums: Int) {
        for(i <- 0 until nums) {
            when(cnt(i + idx)) { write_reg(i, regback(i)) }
        }
    }

    val def_pmp_cfg = ("h1f".U(64.W))
    val def_pmp_addr = if(is_sv48) ("hffffffffffff".U(64.W)) else ("h7fffffffff".U(64.W))

    when(state === STATE_DO_INIT) {
        when(cnt(0)) { write_reg(2, def_pmp_cfg) }
        when(cnt(1)) { write_reg(3, def_pmp_addr) }
        when(cnt(2)) { invoke_inst("h3a039073".U) } // csrrw x0, pmpcfg0, x7
        when(cnt(3)) { invoke_inst("h3b041073".U) } // csrrw x0, pmpaddr0, x8
        when(cnt(4)) { wait_inst() }
        when(cnt(5)) {
            cnt := 1.U 
            state := STATE_RECV_HEAD
        }
    }

    when(state === STATE_FLUSH) {
        when(cnt(0)) { invoke_inst("b00010010000000000000000001110011".U) } // sfence.vma x0, x0
        when(cnt(1)) { wait_inst() }
        when(cnt(2)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_SYNCI) {
        when(cnt(0)) { invoke_inst("h0000100f".U) } // fence.i
        when(cnt(1)) { wait_inst() }
        when(cnt(2)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_REGRD && !oparg(2)(5)) {
        io.cpu.regacc_rd := true.B
        io.cpu.regacc_idx := oparg(2)(4, 0)
        when(!io.cpu.regacc_busy) {
            state := STATE_SEND_HEAD
            for(i <- 0 to 7) {
                retarg(i) := io.cpu.regacc_rdata(i*8+7, i*8)
            }
        }
    }

    when(state === STATE_REGRD && oparg(2)(5)) {
        backup_regs(0, 1)
        when(cnt(1)) { invoke_inst(("he20002d3".U) | (oparg(2)(4, 0) << 15)) } // fmv.x.d x5, fn
        when(cnt(2)) { wait_inst() }
        when(cnt(3)) { read_reg_to_retarg(0, 2, 8) }
        recover_regs(4, 1)
        when(cnt(5)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_REGWT && !oparg(2)(5)) {
        io.cpu.regacc_wt := true.B
        io.cpu.regacc_idx := oparg(2)(4, 0)
        val _regacc_wdata = Wire(Vec(8, UInt(8.W)))
        for(i <- 0 to 7) {
            _regacc_wdata(i) := oparg(4+i)
        }
        io.cpu.regacc_wdata := _regacc_wdata.asUInt()
        when(!io.cpu.regacc_busy) {
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_REGWT && oparg(2)(5)) {
        backup_regs(0, 1)
        when(cnt(1)) { write_reg_from_oparg(0, 4, 8) }
        when(cnt(2)) { invoke_inst(("hf2028053".U) | (oparg(2)(4, 0) << 7)) } // fmv.d.x fn, x5
        when(cnt(3)) { wait_inst() }
        recover_regs(4, 1)
        when(cnt(5)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_NEXT) {
        retarg(0) := 0.U 
        retarg(1) := 0.U 
        backup_regs(0, 3)
        when(cnt(3)) { invoke_inst("h342022f3".U) } // csrrs x5, mcause, x0
        when(cnt(4)) { invoke_inst("h34102373".U) } // csrrs x6, mepc, x0
        when(cnt(5)) { invoke_inst("h343023f3".U) } // csrrs x7, mtval, x0
        when(cnt(6)) { wait_inst() }
        when(cnt(7)) { read_reg_to_retarg(0, 2, 1) }
        when(cnt(8)) { read_reg_to_retarg(1, 3, 6) }
        when(cnt(9)) { read_reg_to_retarg(2, 9, 6) }
        when(cnt(10)) { invoke_inst("h180012f3".U) } // csrrw x5, satp, x0
        when(cnt(11)) { wait_inst() }
        when(cnt(12)) { read_reg(0, satp_back) }
        recover_regs(13, 3)
        when(cnt(16)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_REDIR) {
        backup_regs(0, 2)
        when(cnt(2)) { write_reg(0, satp_back) }
        when(cnt(3)) { write_reg_from_oparg(1, 2, 6) }
        when(cnt(4)) { invoke_inst("h18029073".U) } // csrrw x0, satp, x5
        when(cnt(5)) { invoke_inst("h34131073".U) } // csrrw x0, mepc, x6
        // Clear MPP Bits (mstatus[12:11]) to return U mode
        when(cnt(6)) { invoke_inst("h00300293".U) } // addi x5, x0, 3
        when(cnt(7)) { invoke_inst("h00b29293".U) } // slli x5, x5, 11
        when(cnt(8)) { invoke_inst("h3002b073".U) } // csrrc x0, mstatus, x5
        when(cnt(9)) { invoke_inst("h0330000f".U) } // fence rw, rw
        when(cnt(10)) { wait_inst() }
        recover_regs(11, 2)
        when(cnt(13)) { invoke_inst("h30200073".U) } // mret
        when(cnt(14)) {
            io.cpu.inst64_flush := true.B 
            cnt := (cnt << 1)
            cpu_state := CPU_USER
        }
        when(cnt(15)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_MEMRD) {
        backup_regs(0, 2)
        when(cnt(2)) { write_reg_from_oparg(0, 2, 6) }
        when(cnt(3)) { invoke_inst("h0002b303".U) } // ld x6, 0(x5)
        when(cnt(4)) { wait_inst() }
        when(cnt(5)) { read_reg_to_retarg(1, 0, 8) }
        recover_regs(6, 2)
        when(cnt(8)) {
            cnt := 1.U
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_MEMWT) {
        backup_regs(0, 2)
        when(cnt(2)) { write_reg_from_oparg(0, 2, 6) }
        when(cnt(3)) { write_reg_from_oparg(1, 8, 8) }
        when(cnt(4)) { invoke_inst("h0062b023".U) } // sd x6, 0(x5)
        when(cnt(5)) { wait_inst() }
        recover_regs(6, 2)
        when(cnt(8)) {
            cnt := 1.U
            state := STATE_SEND_HEAD
        }
    }

    val _pg_base_addr = Wire(Vec(5, UInt(8.W)))
    for(i <- 0 to 4) {
        _pg_base_addr(i) := oparg(2+i)
    }
    val pg_base_addr = Cat((0.U(12.W)), _pg_base_addr.asUInt(), (0.U(12.W)))
    val pg_loop_cnt = RegInit(0.U(7.W))

    when(state === STATE_PGST) {
        backup_regs(0, 2)
        when(cnt(2)) { write_reg(0, pg_base_addr) }
        when(cnt(3)) { write_reg_from_oparg(1, 7, 8) }
        when(cnt(4)) { invoke_inst("h0062b023".U) } // sd x6, 0(x5)
        when(cnt(5)) { invoke_inst("h0062b423".U) } // sd x6, 8(x5)
        when(cnt(6)) { invoke_inst("h0062b823".U) } // sd x6, 16(x5)
        when(cnt(7)) { invoke_inst("h0062bc23".U) } // sd x6, 24(x5)
        when(cnt(8)) { invoke_inst("h0262b023".U) } // sd x6, 32(x5)
        when(cnt(9)) { invoke_inst("h0262b423".U) } // sd x6, 40(x5)
        when(cnt(10)) { invoke_inst("h0262b823".U) } // sd x6, 48(x5)
        when(cnt(11)) { invoke_inst("h0262bc23".U) } // sd x6, 56(x5)
        when(cnt(12)) { invoke_inst("h04028293".U) } // addi x5, x5, 64
        when(cnt(13)) { wait_inst() }
        when(cnt(14)) {
            when(pg_loop_cnt < 63.U) {
                cnt := (1.U << 4)
                pg_loop_cnt := pg_loop_cnt + 1.U 
            }.otherwise {
                cnt := (cnt << 1)
                pg_loop_cnt := 0.U
            }
        }
        recover_regs(15, 2)
        when(cnt(17)) {
            cnt := 1.U
            state := STATE_SEND_HEAD
        }
    }

    val _srcpg_base_addr = Wire(Vec(5, UInt(8.W)))
    for(i <- 0 to 4) {
        _srcpg_base_addr(i) := oparg(7+i)
    }
    val srcpg_base_addr = Cat((0.U(12.W)), _srcpg_base_addr.asUInt(), (0.U(12.W)))

    when(state === STATE_PGCP) {
        backup_regs(0, 10)
        when(cnt(10)) { write_reg(0, srcpg_base_addr) }
        when(cnt(11)) { write_reg(9, pg_base_addr) }
        when(cnt(12)) { invoke_inst("h0002b303".U) } // ld x6, 0(x5)
        when(cnt(13)) { invoke_inst("h0082b383".U) } // ld x7, 8(x5)
        when(cnt(14)) { invoke_inst("h0102b403".U) } // ld x8, 16(x5)
        when(cnt(15)) { invoke_inst("h0182b483".U) } // ld x9, 24(x5)
        when(cnt(16)) { invoke_inst("h0202b903".U) } // ld x18, 32(x5)
        when(cnt(17)) { invoke_inst("h0282b983".U) } // ld x19, 40(x5)
        when(cnt(18)) { invoke_inst("h0302ba03".U) } // ld x20, 48(x5)
        when(cnt(19)) { invoke_inst("h0382ba83".U) } // ld x21, 56(x5)
        when(cnt(20)) { invoke_inst("h006b3023".U) } // sd x6, 0(x22)
        when(cnt(21)) { invoke_inst("h007b3423".U) } // sd x7, 8(x22)
        when(cnt(22)) { invoke_inst("h008b3823".U) } // sd x8, 16(x22)
        when(cnt(23)) { invoke_inst("h009b3c23".U) } // sd x9, 24(x22)
        when(cnt(24)) { invoke_inst("h032b3023".U) } // sd x18, 32(x22)
        when(cnt(25)) { invoke_inst("h033b3423".U) } // sd x19, 40(x22)
        when(cnt(26)) { invoke_inst("h034b3823".U) } // sd x20, 48(x22)
        when(cnt(27)) { invoke_inst("h035b3c23".U) } // sd x21, 56(x22)
        when(cnt(28)) { invoke_inst("h04028293".U) } // addi x5, x5, 64
        when(cnt(29)) { invoke_inst("h040b0b13".U) } // addi x22, x22, 64
        when(cnt(30)) { wait_inst() }
        when(cnt(31)) {
            when(pg_loop_cnt < 63.U) {
                cnt := (1.U << 12)
                pg_loop_cnt := pg_loop_cnt + 1.U 
            }.otherwise {
                cnt := (cnt << 1)
                pg_loop_cnt := 0.U
            }
        }
        recover_regs(32, 10)
        when(cnt(42)) {
            cnt := 1.U
            state := STATE_SEND_HEAD
        }
    }

    val pgbuf_div8 = RegInit(VecInit(Seq.fill(64)(0.U(64.W))))
    val pgbuf_push_pos = RegInit(0.U(12.W))
    val pgbuf_pop_pos = RegInit(0.U(12.W))

    val _pgdiv8_base_addr = Wire(Vec(5, UInt(8.W)))
    for(i <- 0 to 4) {
        _pgdiv8_base_addr(i) := oparg(2+i)
    }
    val pgdiv8_base_addr = Cat((0.U(12.W)), _pgdiv8_base_addr.asUInt(), opoff, (0.U(9.W)))

    when(state === STATE_PGRD) {
        backup_regs(0, 9)
        when(cnt(9)) {
            io.tx.valid := true.B
            io.tx.bits := Cat(opoff, opcode)
            when(io.tx.ready) {
                cnt := (cnt << 1)
            }
        }
        when(cnt(10)) { write_reg(0, pgdiv8_base_addr) }
        when(cnt(11)) { invoke_inst("h0002b303".U) } // ld x6, 0(x5)
        when(cnt(12)) { invoke_inst("h0082b383".U) } // ld x7, 8(x5)
        when(cnt(13)) { invoke_inst("h0102b403".U) } // ld x8, 16(x5)
        when(cnt(14)) { invoke_inst("h0182b483".U) } // ld x9, 24(x5)
        when(cnt(15)) { invoke_inst("h0202b903".U) } // ld x18, 32(x5)
        when(cnt(16)) { invoke_inst("h0282b983".U) } // ld x19, 40(x5)
        when(cnt(17)) { invoke_inst("h0302ba03".U) } // ld x20, 48(x5)
        when(cnt(18)) { invoke_inst("h0382ba83".U) } // ld x21, 56(x5)
        when(cnt(19)) { wait_inst() }
        for(i <- 0 to 7) {
            when(cnt(20+i)) { read_reg(1+i, pgbuf_div8((pgbuf_push_pos >> 3) + i.U)) }
        }
        when(cnt(28)) {
            when(pgbuf_push_pos === 448.U) {
                cnt := (1.U << 31)
            }.otherwise {
                cnt := (cnt << 1)
            }
            pgbuf_push_pos := pgbuf_push_pos + 64.U
        }
        when(cnt(29)) { invoke_inst("h04028293".U) } // addi x5, x5, 64
        when(cnt(30)) { cnt := (1.U << 11) }
        recover_regs(31, 9)
        when(cnt(40)) {
            when(pgbuf_push_pos === pgbuf_pop_pos) {
                cnt := 1.U 
                state := STATE_RECV_HEAD
                pgbuf_pop_pos := 0.U 
                pgbuf_push_pos := 0.U 
            }
        }

        when(cnt(9, 0) === 0.U && pgbuf_push_pos > pgbuf_pop_pos) {
            io.tx.valid := true.B 
            io.tx.bits := (pgbuf_div8(pgbuf_pop_pos >> 3) >> (pgbuf_pop_pos(2,0) << 3))(7, 0)
            when(io.tx.ready) {
                pgbuf_pop_pos := pgbuf_pop_pos + 1.U
            }
        }
    }

    when(state === STATE_PGWT) {
        when(io.rx.valid && pgbuf_push_pos < 512.U) {
            val shift_bits = (pgbuf_push_pos(2,0) << 3)
            val masks = (255.U(8.W) << shift_bits)
            pgbuf_div8(pgbuf_push_pos >> 3) := (pgbuf_div8(pgbuf_push_pos >> 3) & (~masks)) | (io.rx.bits << shift_bits)
            pgbuf_push_pos := pgbuf_push_pos + 1.U
        }

        backup_regs(0, 9)
        when(cnt(9)) { write_reg(0, pgdiv8_base_addr) }
        when(cnt(10)) {
            when(pgbuf_push_pos >= pgbuf_pop_pos + 64.U) {
                cnt := (cnt << 1)
            }
        }
        for(i <- 0 to 7) {
            when(cnt(11+i)) { write_reg(1+i, pgbuf_div8((pgbuf_pop_pos >> 3) + i.U)) }
        }
        when(cnt(19)) { invoke_inst("h0062b023".U) } // sd x6, 0(x5)
        when(cnt(20)) { invoke_inst("h0072b423".U) } // sd x7, 8(x5)
        when(cnt(21)) { invoke_inst("h0082b823".U) } // sd x8, 16(x5)
        when(cnt(22)) { invoke_inst("h0092bc23".U) } // sd x9, 24(x5)
        when(cnt(23)) { invoke_inst("h0322b023".U) } // sd x18, 32(x5)
        when(cnt(24)) { invoke_inst("h0332b423".U) } // sd x19, 40(x5)
        when(cnt(25)) { invoke_inst("h0342b823".U) } // sd x20, 48(x5)
        when(cnt(26)) { invoke_inst("h0352bc23".U) } // sd x21, 56(x5)
        when(cnt(27)) { invoke_inst("h04028293".U) } // addi x5, x5, 64
        when(cnt(28)) { wait_inst() }
        when(cnt(29)) {
            when(pgbuf_pop_pos === 448.U) {
                cnt := (cnt << 1)
            }.otherwise {
                cnt := (1.U << 10)
            }
            pgbuf_pop_pos := pgbuf_pop_pos + 64.U
        }
        recover_regs(30, 9)
        when(cnt(39)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
            pgbuf_pop_pos := 0.U 
            pgbuf_push_pos := 0.U 
        }
    }

}

object NulCPUCtrlMain extends App {
    println("Generating the NulCPUCtrl hardware")
    emitVerilog(new NulCPUCtrl(), Array("--target-dir", "generated"))
}

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

class NulCPUCtrlWithUart(frequency: Int, baudRate: Int) extends Module {
    val io = IO(new Bundle {
        val cpu     = new NulCPUBundle()
        val txd     = Output(UInt(1.W))
        val rxd     = Input(UInt(1.W))
        val dgb_sta = Output(UInt(5.W))
    })

    val ctrl = Module(new NulCPUCtrl())
    val tx = Module(new Tx(frequency, baudRate))
    val rx = Module(new Rx(frequency, baudRate))

    io.dgb_sta := ctrl.io.dgb_sta

    io.cpu <> ctrl.io.cpu 
    io.txd := tx.io.txd 
    rx.io.rxd := io.rxd 
    ctrl.io.tx <> tx.io.channel
    ctrl.io.rx <> rx.io.channel
}

object NulCPUCtrlUartMain extends App {
    println("Generating the NulCPUCtrlUart hardware")
    emitVerilog(new NulCPUCtrlWithUart(125000000, 115200), Array("--target-dir", "generated"))
}
