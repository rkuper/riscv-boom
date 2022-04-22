//******************************************************************************
// See LICENSE.Berkeley for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.lsu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

import boom.common._
import boom.exu.BrResolutionInfo
import boom.util.{IsKilledByBranch, GetNewBrMask, BranchKillableQueue, IsOlder, UpdateBrMask}



abstract class DataPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends BoomModule()(p)
{
  val io = IO(new Bundle {
    val mshr_avail = Input(Bool())
    val req_val    = Input(Bool())
    val req_addr   = Input(UInt(coreMaxAddrBits.W))
    val req_coh    = Input(new ClientMetadata)

    val prefetch   = Decoupled(new BoomDCacheReq)
  })
}

/**
  * Does not prefetch
  */
class NullPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{
  io.prefetch.valid := false.B
  io.prefetch.bits  := DontCare
}

/**
  * Next line prefetcher. Grabs the next line on a cache miss
  */
class NLPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{

  val req_valid = RegInit(false.B)
  val req_addr  = Reg(UInt(coreMaxAddrBits.W))
  val req_cmd   = Reg(UInt(M_SZ.W))

  val mshr_req_addr = io.req_addr + cacheBlockBytes.U
  val cacheable = edge.manager.supportsAcquireBSafe(mshr_req_addr, lgCacheBlockBytes.U)
  when (io.req_val && cacheable) {
    req_valid := true.B
    req_addr  := mshr_req_addr
    req_cmd   := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
  } .elsewhen (io.prefetch.fire()) {
    req_valid := false.B
  }

  io.prefetch.valid            := req_valid && io.mshr_avail
  io.prefetch.bits.addr        := req_addr
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
}


/**
  * Offset prefetcher. Grabs the next and an offseted line on a cache miss
  */
class OffsetPrefetcher(implicit edge: TLEdgeOut, p: Parameters, val historySize: Int = 10) extends DataPrefetcher
{

  val req_valid      = RegInit(false.B)
  val req_addr       = Reg(UInt(coreMaxAddrBits.W))
  val req_cmd        = Reg(UInt(M_SZ.W))

  val past_access    = Reg(UInt(coreMaxAddrBits.W))

  val offset_history = Reg(Vec(historySize, SInt(coreMaxAddrBits.W)))

  val offset         = (io.req_addr.asSInt) - (past_access.asSInt)
  val offset_found   = (offset === offset_history(0))
                        (offset =/= 0.S)
  val mshr_req_addr  = Mux(offset_found,
                           (io.req_addr.asSInt + offset).asUInt,
                           io.req_addr + cacheBlockBytes.U)
  val cacheable = edge.manager.supportsAcquireBSafe(mshr_req_addr, lgCacheBlockBytes.U)
  when (io.req_val && cacheable) {
    req_valid         := true.B
    req_addr          := mshr_req_addr
    req_cmd           := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)

    past_access       := io.req_addr
    offset_history(0) := offset
    for (i <- 1 until historySize) { offset_history(i) := offset_history(i-1) }
  } .elsewhen (io.prefetch.fire()) {
    req_valid  := false.B
  }

  io.prefetch.valid            := req_valid && io.mshr_avail
  io.prefetch.bits.addr        := req_addr
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
}


/**
  * Stride prefetcher. Grabs the next and an offseted line on a cache miss
  */
class StridePrefetcher(implicit edge: TLEdgeOut, p: Parameters, val historySize: Int = 10) extends DataPrefetcher
{

  val req_valid        = RegInit(false.B)
  val req_addr         = Reg(UInt(coreMaxAddrBits.W))
  val req_cmd          = Reg(UInt(M_SZ.W))

  val stride           = Reg(SInt(coreMaxAddrBits.W))
  val stride_accum     = Reg(SInt(coreMaxAddrBits.W))
  val past_access      = Reg(UInt(coreMaxAddrBits.W))
  val offset_history   = Reg(Vec(historySize, SInt(coreMaxAddrBits.W)))
  val prefetch_counter = Reg(UInt(historySize.W))
  val stride_found     = RegInit(false.B)
  val prefetch_fired   = RegInit(false.B)
  val past_req_cmd     = RegInit(false.B)
  val active_nextline  = RegInit(false.B)

  // val mshr_req_addr    = (io.req_addr.asSInt + stride_accum).asUInt
  val stride_mshr_req_addr = (past_access.asSInt + stride_accum).asUInt
  val stride_cacheable     = edge.manager.supportsAcquireBSafe(stride_mshr_req_addr, lgCacheBlockBytes.U)
  val nl_mshr_req_addr     = io.req_addr + cacheBlockBytes.U
  val nl_cacheable         = edge.manager.supportsAcquireBSafe(nl_mshr_req_addr, lgCacheBlockBytes.U)
  val offset               = (io.req_addr.asSInt) - (past_access.asSInt)
  val pattern_found        = stride_cacheable &&
                             ((offset === offset_history(0)) ||
                              (offset === offset_history(1)) ||
                              (offset === offset_history(2)) ||
                              (offset === offset_history(3)) ||
                              (offset === offset_history(4))) &&
                              // (offset === offset_history(5))) &&
                             (offset =/= 0.S)

  // Begin prefetching when stride has been found
  when (!active_nextline && stride_found) {

    // Grab the next 7 new cache lines
    when (prefetch_counter < 4.U && stride_cacheable) {

      // set flag saying it has NOT been fired yet
      when (!prefetch_fired) {
        req_valid         := true.B
        req_addr          := stride_mshr_req_addr
        req_cmd           := Mux(past_req_cmd, M_PFW, M_PFR)
        stride_accum      := stride_accum + stride
        past_access       := ((past_access.asSInt) + stride).asUInt
        past_req_cmd      := ClientStates.hasWritePermission(io.req_coh.state)
        prefetch_fired    := true.B

      // Once fired, then set flag to allow new stride to prefetch and prepare new addr
      } .elsewhen (io.prefetch.fire()) {
        prefetch_fired   := false.B
        req_valid        := false.B
        prefetch_counter := prefetch_counter + 1.U
      }

    // Once prefetching ends (or out of bounds), reset and find stride again
    } .otherwise {
      prefetch_counter := 0.U
      prefetch_fired   := false.B
      stride_found     := false.B
      req_valid        := false.B
    }

  // Search for stride by comparing last three addresses accessed
  } .elsewhen (io.req_val && (stride_cacheable || nl_cacheable) && !active_nextline) {
    past_access       := io.req_addr
    past_req_cmd      := ClientStates.hasWritePermission(io.req_coh.state)
    stride            := offset
    stride_accum      := offset
    stride_found      := pattern_found
    offset_history(0) := offset
    for (i <- 1 until historySize) { offset_history(i) := offset_history(i-1) }

    // If stride not found yet, get next line for now
    when (!pattern_found) {
      req_valid       := true.B
      req_addr        := nl_mshr_req_addr
      req_cmd         := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
      active_nextline := true.B
    }
  } .elsewhen (io.prefetch.fire()) {
    req_valid       := false.B
    active_nextline := false.B
  }


  io.prefetch.valid            := req_valid && (io.mshr_avail || (stride_found && stride_cacheable))
  io.prefetch.bits.addr        := req_addr
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
}


// TODO: This is purely for testing/comparison, NOT the main design for this assignment
/**
  * Offset prefetcher. Grabs only an offseted line on a cache miss
  */
class OnlyOffsetPrefetcher(implicit edge: TLEdgeOut, p: Parameters, val historySize: Int = 10) extends DataPrefetcher
{

  val req_valid      = RegInit(false.B)
  val req_addr       = Reg(UInt(coreMaxAddrBits.W))
  val req_cmd        = Reg(UInt(M_SZ.W))

  val past_access    = Reg(UInt(coreMaxAddrBits.W))

  val offset_history = Reg(Vec(historySize, SInt(coreMaxAddrBits.W)))

  val offset         = (io.req_addr.asSInt) - (past_access.asSInt)
  val offset_found   = ((offset === offset_history(0)) ||
                        (offset === offset_history(1)) ||
                        (offset === offset_history(2)) ||
                        (offset === offset_history(3)) ||
                        (offset === offset_history(4)) ||
                        (offset === offset_history(5))) &&
                        (offset =/= 0.S)
  val mshr_req_addr  = Mux(offset_found,
                           (io.req_addr.asSInt + offset).asUInt,
                           (io.req_addr.asSInt + offset_history(0)).asUInt)
                           // io.req_addr + cacheBlockBytes.U)
  val cacheable = edge.manager.supportsAcquireBSafe(mshr_req_addr, lgCacheBlockBytes.U)
  when (io.req_val && cacheable) {
    req_valid     := true.B
    req_addr      := mshr_req_addr
    req_cmd       := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)

    past_access   := io.req_addr
    offset_history(0) := offset
    for (i <- 1 until historySize) { offset_history(i) := offset_history(i-1) }
  } .elsewhen (io.prefetch.fire()) {
    req_valid  := false.B
  }

  io.prefetch.valid            := req_valid && io.mshr_avail
  io.prefetch.bits.addr        := req_addr
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
}
