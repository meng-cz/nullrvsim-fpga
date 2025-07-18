
package nulctrl

import chisel3._
import chisel3.util._


class NulCPUCtrlMP(cpunum: Int) extends Module {

    val is_sv48 = false
    val is_hard_fp = true

    assert(cpunum > 1)
    val cpunum_bitwid = log2Ceil(cpunum)

    val io = IO(new Bundle {
        val cpu     = Vec(cpunum, new NulCPUBundle())
        val tx      = new UartIO()
        val rx      = Flipped(new UartIO())
        val dbg_sta = Output(UInt(8.W))
    })

    io.rx.ready := true.B
    io.tx.valid := false.B 
    io.tx.bits := 0.U 

    val CPU_INIT = 0.U 
    val CPU_HALT = 1.U 
    val CPU_ITR  = 2.U 
    val CPU_USER = 3.U 

    val cpu_state = RegInit(VecInit(Seq.fill(cpunum)(0.U(2.W))))

    for(i <- 0 until cpunum) {
        io.cpu(i).ext_itr := false.B
        io.cpu(i).regacc_rd := false.B
        io.cpu(i).regacc_wt := false.B
        io.cpu(i).regacc_idx := 0.U
        io.cpu(i).regacc_wdata := 0.U 
        io.cpu(i).inst64 := false.B 
        io.cpu(i).inst64_raw := 0.U
        io.cpu(i).inst64_nowait := false.B 
        io.cpu(i).inst64_flush := false.B
    }

    val global_clk = RegInit(0.U(64.W))
    val user_clk = RegInit(VecInit(Seq.fill(cpunum)(0.U(64.W))))

    global_clk := global_clk + 1.U
    for(i <- 0 until cpunum) {
        when(io.cpu(i).priv === 0.U) {
            user_clk(i) := user_clk(i) + 1.U
        }
    }

    val cpu_raised_itr = RegInit(VecInit(Seq.fill(cpunum)(false.B)))
    val last_priv = RegInit(VecInit(Seq.fill(cpunum)(3.U(2.W))))
    for(i <- 0 until cpunum) {
        last_priv(i) := io.cpu(i).priv
        when((last_priv(i) === 0.U) && (io.cpu(i).priv =/= 0.U) && (cpu_state(i) === CPU_USER)) {
            cpu_raised_itr(i) := true.B 
            cpu_state(i) := CPU_ITR
        }
        io.cpu(i).stop_fetch := (cpu_state(i) === CPU_ITR) || (cpu_state(i) === CPU_HALT) || (last_priv(i) === 0.U && io.cpu(i).priv =/= 0.U)
    }
    
    val event_queue = Module(new Queue(UInt(cpunum_bitwid.W), cpunum + 1))
    val has_itr = cpu_raised_itr.reduce(_||_)
    val event_idx = PriorityEncoder(cpu_raised_itr.asUInt)
    event_queue.io.enq.valid := has_itr
    event_queue.io.enq.bits := event_idx
    when(has_itr && event_queue.io.enq.ready) {
        cpu_raised_itr(event_idx) := false.B
    }
    
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
    val SEROP_UCLK  = 17.U
    val SEROP_INST  = 20.U

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
    val STATE_FLUSH2        = 13.U 
    val STATE_REGRD         = 14.U 
    val STATE_REGWT         = 15.U 
    val STATE_MEMRD         = 16.U 
    val STATE_MEMWT         = 17.U 
    val STATE_PGRD          = 18.U 
    val STATE_PGST          = 19.U 
    val STATE_PGWT          = 20.U 
    val STATE_PGCP          = 21.U 
    val STATE_SYNCI         = 22.U 
    val STATE_INST          = 23.U 

    val state = RegInit(0.U(5.W))
    val trans_bytes = RegInit(0.U(10.W))
    val trans_pos = RegInit(0.U(10.W))

    io.dbg_sta := state.pad(8) 

    val opcode = RegInit(0.U(5.W))
    val opoff = RegInit(0.U(3.W))
    val oparg = RegInit(VecInit(Seq.fill(16)(0.U(8.W))))
    val retarg = RegInit(VecInit(Seq.fill(16)(0.U(8.W))))
    val opidx = Cat(oparg(1), oparg(0))(cpunum_bitwid-1, 0)
    val sel_cpu = io.cpu(opidx)

    val all_inited = io.cpu.map(_.inited).reduce(_&&_)
    when(state === STATE_INIT_WAIT && all_inited) {
        state := STATE_DO_INIT
        for(i <- 0 until cpunum) { cpu_state(i) := CPU_HALT }
    }

    when(state === STATE_RECV_HEAD && io.rx.valid) {
        val rxop = io.rx.bits(4, 0)
        val rxoff = io.rx.bits(7, 5)
        opcode := rxop
        opoff := rxoff

        when(rxop === SEROP_HALT || rxop === SEROP_ITR || rxop === SEROP_FTLB || rxop === SEROP_SYNCI || rxop === SEROP_UCLK) {
            trans_bytes := 2.U 
        }.elsewhen(rxop === SEROP_REGRD) {
            trans_bytes := 4.U 
        }.elsewhen(rxop === SEROP_INST) {
            trans_bytes := 6.U 
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
        state := STATE_SEND_HEAD
        switch(opcode) {
            is(SEROP_NEXT) { state := STATE_WAIT_NEXT }
            is(SEROP_HALT) {
                sel_cpu.ext_itr := true.B
                cpu_state(opidx) := CPU_HALT
            }
            is(SEROP_ITR) {
                sel_cpu.ext_itr := true.B
            }
            is(SEROP_MMU) { state := STATE_MMU }
            is(SEROP_REDIR) { state := STATE_REDIR }
            is(SEROP_FTLB) { state := STATE_FLUSH }
            is(SEROP_FTLB2) { state := STATE_FLUSH2 }
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
                for(i <- 0 to 7) {
                    retarg(i) := global_clk(i*8+7, i*8)
                }
            }
            is(SEROP_UCLK) {
                for(i <- 0 to 7) {
                    retarg(i) := user_clk(opidx)(i*8+7, i*8)
                }
            }
            is(SEROP_INST) { state := STATE_INST }
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
            }.elsewhen(opcode === SEROP_REGRD || opcode === SEROP_MEMRD || opcode === SEROP_CLK || opcode === SEROP_UCLK) {
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

    val all_halted = cpu_state.forall(_ === CPU_HALT)

    event_queue.io.deq.ready := (state === STATE_WAIT_NEXT)
    when(state === STATE_WAIT_NEXT && event_queue.io.deq.valid) {
        val pending_next_idx = event_queue.io.deq.bits.pad(16)
        oparg(0) := pending_next_idx(7,0) // to update sel_cpu port
        oparg(1) := pending_next_idx(15,8)
        retarg(0) := pending_next_idx(7,0)
        retarg(1) := pending_next_idx(15,8)
        state := STATE_NEXT
    }.elsewhen(state === STATE_WAIT_NEXT && all_halted) {
        retarg(0) := "hff".U
        retarg(1) := "hff".U
        state := STATE_SEND_HEAD
    }

    val cnt = RegInit(1.U(128.W))
    val regback = RegInit(VecInit(Seq.fill(10)(0.U(64.W))))

    val reg_idxs = Array(5.U, 6.U, 7.U, 8.U, 9.U, 10.U, 11.U, 12.U, 13.U, 14.U)

    def read_reg(idx: Int, dst: UInt) {
        sel_cpu.regacc_rd := true.B
        sel_cpu.regacc_idx := reg_idxs(idx)
        when(!sel_cpu.regacc_busy) {
            cnt := (cnt << 1)
            dst := sel_cpu.regacc_rdata
        }
    }
    def read_reg_to_retarg(idx: Int, pos: Int, bytes: Int) {
        sel_cpu.regacc_rd := true.B
        sel_cpu.regacc_idx := reg_idxs(idx)
        when(!sel_cpu.regacc_busy) {
            cnt := (cnt << 1)
            for(i <- 0 to (bytes-1)) {
                retarg(pos + i) := (sel_cpu.regacc_rdata >> i*8)(7, 0)
            }
        }
    }
    def write_reg(idx: Int, src: UInt) {
        sel_cpu.regacc_wt := true.B
        sel_cpu.regacc_idx := reg_idxs(idx)
        sel_cpu.regacc_wdata := src
        when(!sel_cpu.regacc_busy) {
            cnt := (cnt << 1)
        }
    }
    def write_reg_from_oparg(idx: Int, pos: Int, bytes: Int) {
        sel_cpu.regacc_wt := true.B
        sel_cpu.regacc_idx := reg_idxs(idx)
        val _regacc_wdata = Wire(Vec(8, UInt(8.W)))
        for(i <- 0 to (bytes-1)) {
            _regacc_wdata(i) := oparg(pos + i)
        }
        if(bytes < 8) {
            for(i <- bytes to 7) {
                _regacc_wdata(i) := 0.U(8.W)
            }
        }
        sel_cpu.regacc_wdata := _regacc_wdata.asUInt()
        when(!sel_cpu.regacc_busy) {
            cnt := (cnt << 1)
        }
    }
    def invoke_inst(inst: UInt) {
        sel_cpu.inst64 := true.B
        sel_cpu.inst64_raw := inst
        when(sel_cpu.inst64_ready) {
            cnt := (cnt << 1)
        }
    }
    def wait_inst() {
        sel_cpu.inst64_flush := true.B 
        when(!sel_cpu.inst64_busy) {
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

    val init_cnt = RegInit(0.U(cpunum_bitwid.W))
    when(state === STATE_DO_INIT) {
        when(cnt(0)) { cnt := (cnt << 1.U) }
        when(cnt(1)) { write_reg(2, def_pmp_cfg) }
        when(cnt(2)) { write_reg(3, def_pmp_addr) }
        when(cnt(3)) { invoke_inst("h3a039073".U) } // csrrw x0, pmpcfg0, x7
        when(cnt(4)) { invoke_inst("h3b041073".U) } // csrrw x0, pmpaddr0, x8
        when(cnt(5)) {
            if(is_hard_fp) {
                invoke_inst("h000022b7".U) // lui	x5,0x2
            } else {
                cnt := (cnt << 1)
            }
        }
        when(cnt(6)) {
            if(is_hard_fp) {
                invoke_inst("h3002a073".U) // csrs	mstatus,x5
            } else {
                cnt := (cnt << 1)
            }
        }
        when(cnt(7)) { wait_inst() }
        when(cnt(8)) {
            cnt := 1.U 
            when(init_cnt === (cpunum - 1).U) {
                state := STATE_RECV_HEAD
            }.otherwise {
                val nextidx = (init_cnt + 1.U).pad(16)
                oparg(0) := nextidx(7,0)
                oparg(1) := nextidx(15,8)
                init_cnt := init_cnt + 1.U
            }
        }
    }

    when(state === STATE_INST) {
        when(cnt(0)) { invoke_inst(Cat(oparg(5), oparg(4), oparg(3), oparg(2))) }
        when(cnt(1)) { wait_inst() }
        when(cnt(2)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    val satp_mode = if(is_sv48) 9.U(4.W) else 8.U(4.W)
    val satp_value = Cat(
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

    when(state === STATE_MMU) {
        backup_regs(0, 1)
        when(cnt(1)) { write_reg(0, satp_value) }
        when(cnt(2)) { invoke_inst("h18029073".U) } // csrrw x0, satp, x5
        when(cnt(3)) { wait_inst() }
        recover_regs(4, 1)
        when(cnt(5)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
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

    val flush_tlb_address = Cat((0.U(12.W)), oparg(8), oparg(7), oparg(6), oparg(5), oparg(4), (0.U(12.W)))
    when(state === STATE_FLUSH2) {
        backup_regs(0, 1)
        when(cnt(1)) { write_reg(1, flush_tlb_address) }
        when(cnt(2)) { invoke_inst("b00010010000000000000000001110011".U | (5.U << 15)) } // sfence.vma x5, x0
        when(cnt(3)) { wait_inst() }
        recover_regs(4, 1)
        when(cnt(5)) {
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
        sel_cpu.regacc_rd := true.B
        sel_cpu.regacc_idx := oparg(2)(4, 0)
        when(!sel_cpu.regacc_busy) {
            state := STATE_SEND_HEAD
            for(i <- 0 to 7) {
                retarg(i) := sel_cpu.regacc_rdata(i*8+7, i*8)
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
        sel_cpu.regacc_wt := true.B
        sel_cpu.regacc_idx := oparg(2)(4, 0)
        val _regacc_wdata = Wire(Vec(8, UInt(8.W)))
        for(i <- 0 to 7) {
            _regacc_wdata(i) := oparg(4+i)
        }
        sel_cpu.regacc_wdata := _regacc_wdata.asUInt()
        when(!sel_cpu.regacc_busy) {
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
        backup_regs(0, 3)
        when(cnt(3)) { invoke_inst("h342022f3".U) } // csrrs x5, mcause, x0
        when(cnt(4)) { invoke_inst("h34102373".U) } // csrrs x6, mepc, x0
        when(cnt(5)) { invoke_inst("h343023f3".U) } // csrrs x7, mtval, x0
        when(cnt(6)) { wait_inst() }
        when(cnt(7)) { read_reg_to_retarg(0, 2, 1) }
        when(cnt(8)) { read_reg_to_retarg(1, 3, 6) }
        when(cnt(9)) { read_reg_to_retarg(2, 9, 6) }
        recover_regs(10, 3)
        when(cnt(13)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_REDIR) {
        backup_regs(0, 2)
        when(cnt(2)) { write_reg_from_oparg(1, 2, 6) }
        when(cnt(3)) { invoke_inst("h34131073".U) } // csrrw x0, mepc, x6
        // Clear MPP Bits (mstatus[12:11]) to return U mode
        when(cnt(4)) { invoke_inst("h00300293".U) } // addi x5, x0, 3
        when(cnt(5)) { invoke_inst("h00b29293".U) } // slli x5, x5, 11
        when(cnt(6)) { invoke_inst("h3002b073".U) } // csrrc x0, mstatus, x5
        when(cnt(7)) {
            if(is_hard_fp) {
                invoke_inst("h000022b7".U) // lui	x5,0x2
            } else {
                cnt := (cnt << 1)
            }
        }
        when(cnt(8)) {
            if(is_hard_fp) {
                invoke_inst("h3002a073".U) // csrs	mstatus,x5
            } else {
                cnt := (cnt << 1)
            }
        }
        when(cnt(9)) {
            if(is_hard_fp) {
                invoke_inst("h00301073".U) // csrrw x0, fcsr, x0
            } else {
                cnt := (cnt << 1)
            }
        }
        when(cnt(10)) { invoke_inst("h0330000f".U) } // fence rw, rw
        when(cnt(11)) { wait_inst() }
        recover_regs(12, 2)
        when(cnt(14)) {
            sel_cpu.inst64_nowait := true.B
            invoke_inst("h30200073".U)
            when(sel_cpu.inst64_ready) {
                cpu_state(opidx) := CPU_USER
            }
        } // mret
        when(cnt(15)) {
            sel_cpu.inst64_flush := true.B 
            cnt := (cnt << 1)
        }
        when(cnt(16)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_MEMRD) {
        backup_regs(0, 2)
        when(cnt(2)) { write_reg_from_oparg(0, 2, 6) }
        when(cnt(3)) { invoke_inst("h0002b303".U) } // ld x6, 0(x5)
        when(cnt(4)) { invoke_inst("h0330000f".U) } // fence rw, rw
        when(cnt(5)) { wait_inst() }
        when(cnt(6)) { read_reg_to_retarg(1, 0, 8) }
        recover_regs(7, 2)
        when(cnt(9)) {
            cnt := 1.U
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_MEMWT) {
        backup_regs(0, 2)
        when(cnt(2)) { write_reg_from_oparg(0, 2, 6) }
        when(cnt(3)) { write_reg_from_oparg(1, 8, 8) }
        when(cnt(4)) { invoke_inst("h0062b023".U) } // sd x6, 0(x5)
        when(cnt(5)) { invoke_inst("h0330000f".U) } // fence rw, rw
        when(cnt(6)) { wait_inst() }
        recover_regs(7, 2)
        when(cnt(9)) {
            cnt := 1.U
            state := STATE_SEND_HEAD
        }
    }

    val _ppn_from_arg2 = Wire(Vec(5, UInt(8.W)))
    for(i <- 0 to 4) {
        _ppn_from_arg2(i) := oparg(2+i)
    }
    val pg_base_addr2 = Cat((0.U(12.W)), oparg(6), oparg(5), oparg(4), oparg(3), oparg(2), (0.U(12.W)))

    val _ppn_from_arg7 = Wire(Vec(5, UInt(8.W)))
    for(i <- 0 to 4) {
        _ppn_from_arg7(i) := oparg(7+i)
    }
    val pg_base_addr7 = Cat((0.U(12.W)), oparg(11), oparg(10), oparg(9), oparg(8), oparg(7), (0.U(12.W)))

    val pg_loop_cnt = RegInit(0.U(8.W))


    when(state === STATE_PGST) {
        backup_regs(0, 2)
        when(cnt(2)) { write_reg(0, pg_base_addr2) }
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
            when(pg_loop_cnt =/= 63.U) {
                cnt := (1.U << 4)
                pg_loop_cnt := pg_loop_cnt + 1.U 
            }.otherwise {
                cnt := (cnt << 1)
                pg_loop_cnt := 0.U
            }
        }
        when(cnt(15)) { invoke_inst("h0330000f".U) } // fence rw, rw
        when(cnt(16)) { wait_inst() }
        recover_regs(17, 2)
        when(cnt(19)) {
            cnt := 1.U
            state := STATE_SEND_HEAD
        }
    }

    when(state === STATE_PGCP) {
        backup_regs(0, 10)
        when(cnt(10)) { write_reg(0, pg_base_addr7) }
        when(cnt(11)) { write_reg(9, pg_base_addr2) }
        when(cnt(12)) { invoke_inst("h0002b303".U) } // ld x6, 0(x5)
        when(cnt(13)) { invoke_inst("h0082b383".U) } // ld x7, 8(x5)
        when(cnt(14)) { invoke_inst("h0102b403".U) } // ld x8, 16(x5)
        when(cnt(15)) { invoke_inst("h0182b483".U) } // ld x9, 24(x5)
        when(cnt(16)) { invoke_inst("h0202b503".U) } // ld x10, 32(x5)
        when(cnt(17)) { invoke_inst("h0282b583".U) } // ld x11, 40(x5)
        when(cnt(18)) { invoke_inst("h0302b603".U) } // ld x12, 48(x5)
        when(cnt(19)) { invoke_inst("h0382b683".U) } // ld x13, 56(x5)
        when(cnt(20)) { invoke_inst("h00673023".U) } // sd x6, 0(x14)
        when(cnt(21)) { invoke_inst("h00773423".U) } // sd x7, 8(x14)
        when(cnt(22)) { invoke_inst("h00873823".U) } // sd x8, 16(x14)
        when(cnt(23)) { invoke_inst("h00973c23".U) } // sd x9, 24(x14)
        when(cnt(24)) { invoke_inst("h02a73023".U) } // sd x10, 32(x14)
        when(cnt(25)) { invoke_inst("h02b73423".U) } // sd x11, 40(x14)
        when(cnt(26)) { invoke_inst("h02c73823".U) } // sd x12, 48(x14)
        when(cnt(27)) { invoke_inst("h02d73c23".U) } // sd x13, 56(x14)
        when(cnt(28)) { invoke_inst("h04028293".U) } // addi x5, x5, 64
        when(cnt(29)) { invoke_inst("h04070713".U) } // addi x14, x14, 64
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
        when(cnt(32)) { invoke_inst("h0330000f".U) } // fence rw, rw
        when(cnt(33)) { wait_inst() }
        recover_regs(34, 10)
        when(cnt(44)) {
            cnt := 1.U
            state := STATE_SEND_HEAD
        }
    }

    val pgbuf_div8 = RegInit(VecInit(Seq.fill(64)(0.U(64.W))))
    val pgbuf_uart_pos = RegInit(0.U(12.W))
    val pgbuf_cpu_pos = RegInit(0.U(12.W))

    when(state === STATE_PGRD) {
        backup_regs(0, 9)
        when(cnt(9)) {
            io.tx.valid := true.B
            io.tx.bits := Cat(opoff, opcode)
            when(io.tx.ready) {
                cnt := (cnt << 1)
            }
        }
        when(cnt(10)) { write_reg(0, pg_base_addr2 | (opoff << 9)) }
        when(cnt(11)) { invoke_inst("h0002b303".U) } // ld x6, 0(x5)
        when(cnt(12)) { invoke_inst("h0082b383".U) } // ld x7, 8(x5)
        when(cnt(13)) { invoke_inst("h0102b403".U) } // ld x8, 16(x5)
        when(cnt(14)) { invoke_inst("h0182b483".U) } // ld x9, 24(x5)
        when(cnt(15)) { invoke_inst("h0202b503".U) } // ld x10, 32(x5)
        when(cnt(16)) { invoke_inst("h0282b583".U) } // ld x11, 40(x5)
        when(cnt(17)) { invoke_inst("h0302b603".U) } // ld x12, 48(x5)
        when(cnt(18)) { invoke_inst("h0382b683".U) } // ld x13, 56(x5)
        when(cnt(19)) { invoke_inst("h04028293".U) } // addi x5, x5, 64
        when(cnt(20)) { wait_inst() }
        for(i <- 0 to 7) {
            when(cnt(21+i)) { read_reg(1+i, pgbuf_div8((pgbuf_cpu_pos >> 3) + i.U)) }
        }
        when(cnt(29)) {
            when(pgbuf_cpu_pos === 448.U) {
                cnt := (cnt << 1)
            }.otherwise {
                cnt := (1.U << 11)
            }
            pgbuf_cpu_pos := pgbuf_cpu_pos + 64.U
        }
        when(cnt(30)) { invoke_inst("h0330000f".U) } // fence rw, rw
        when(cnt(31)) { wait_inst() }
        recover_regs(32, 9)
        when(cnt(41)) {
            when(pgbuf_cpu_pos === pgbuf_uart_pos) {
                cnt := 1.U 
                state := STATE_RECV_HEAD
                pgbuf_cpu_pos := 0.U 
                pgbuf_uart_pos := 0.U 
            }
        }

        when(cnt(9, 0) === 0.U && pgbuf_cpu_pos > pgbuf_uart_pos) {
            io.tx.valid := true.B 
            val word = pgbuf_div8(pgbuf_uart_pos >> 3)
            val shift = (pgbuf_uart_pos(2,0) * 8.U)
            io.tx.bits := (word >> shift)(7, 0)
            when(io.tx.ready) {
                pgbuf_uart_pos := pgbuf_uart_pos + 1.U
            }
        }
    }

    val wt_byte_cnt = RegInit(0.U(4.W))
    val wt_byte_buf = RegInit(VecInit(Seq.fill(8)(0.U(8.W))))

    when(state === STATE_PGWT) {
        when(io.rx.valid && pgbuf_uart_pos =/= 512.U) {
            wt_byte_buf(wt_byte_cnt) := io.rx.bits
            when(wt_byte_cnt === 7.U) {
                wt_byte_cnt := 0.U
                pgbuf_div8(pgbuf_uart_pos >> 3) := Cat(io.rx.bits, wt_byte_buf(6), wt_byte_buf(5), wt_byte_buf(4), wt_byte_buf(3), wt_byte_buf(2), wt_byte_buf(1), wt_byte_buf(0))
                pgbuf_uart_pos := pgbuf_uart_pos + 8.U
            }.otherwise {
                wt_byte_cnt := wt_byte_cnt + 1.U 
            }
        }

        backup_regs(0, 9)
        when(cnt(9)) { write_reg(0, pg_base_addr2 | (opoff << 9)) }
        when(cnt(10)) {
            when(pgbuf_uart_pos >= pgbuf_cpu_pos + 64.U) {
                cnt := (cnt << 1)
            }
        }
        for(i <- 0 to 7) {
            when(cnt(11+i)) { write_reg(1+i, pgbuf_div8((pgbuf_cpu_pos >> 3) + i.U)) }
        }
        when(cnt(19)) { invoke_inst("h0062b023".U) } // sd x6, 0(x5)
        when(cnt(20)) { invoke_inst("h0072b423".U) } // sd x7, 8(x5)
        when(cnt(21)) { invoke_inst("h0082b823".U) } // sd x8, 16(x5)
        when(cnt(22)) { invoke_inst("h0092bc23".U) } // sd x9, 24(x5)
        when(cnt(23)) { invoke_inst("h02a2b023".U) } // sd x10, 32(x5)
        when(cnt(24)) { invoke_inst("h02b2b423".U) } // sd x11, 40(x5)
        when(cnt(25)) { invoke_inst("h02c2b823".U) } // sd x12, 48(x5)
        when(cnt(26)) { invoke_inst("h02d2bc23".U) } // sd x13, 56(x5)
        when(cnt(27)) { invoke_inst("h04028293".U) } // addi x5, x5, 64
        when(cnt(28)) { wait_inst() }
        when(cnt(29)) {
            when(pgbuf_cpu_pos === 448.U) {
                cnt := (cnt << 1)
            }.otherwise {
                cnt := (1.U << 10)
            }
            pgbuf_cpu_pos := pgbuf_cpu_pos + 64.U
        }
        when(cnt(30)) { invoke_inst("h0330000f".U) } // fence rw, rw
        when(cnt(31)) { wait_inst() }
        recover_regs(32, 9)
        when(cnt(41)) {
            cnt := 1.U 
            state := STATE_SEND_HEAD
            pgbuf_cpu_pos := 0.U 
            pgbuf_uart_pos := 0.U
            wt_byte_cnt := 0.U 
        }
    }

}

class NulCPUCtrlMPWithUart(cpunum: Int, frequency: Int, baudRate: Int) extends Module {
    val io = IO(new Bundle {
        val cpu     = Vec(cpunum, new NulCPUBundle())
        val txd     = Output(UInt(1.W))
        val rxd     = Input(UInt(1.W))
        val dbg_sta = Output(UInt(5.W))
    })

    val ctrl = Module(new NulCPUCtrlMP(cpunum))
    val tx = Module(new Tx(frequency, baudRate))
    val rx = Module(new Rx(frequency, baudRate))

    io.dbg_sta := ctrl.io.dbg_sta

    io.cpu <> ctrl.io.cpu 
    io.txd := tx.io.txd 
    rx.io.rxd := io.rxd 
    ctrl.io.tx <> tx.io.channel
    ctrl.io.rx <> rx.io.channel
}

object NulCPUCtrlUartMPMain extends App {
    println("Generating the NulCPUCtrlMPUart hardware")
    emitVerilog(new NulCPUCtrlMPWithUart(4, 100000000, 460800), Array("--target-dir", "generated"))
}
