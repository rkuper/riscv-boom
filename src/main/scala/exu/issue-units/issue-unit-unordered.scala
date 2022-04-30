//******************************************************************************
// Copyright (c) 2015 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// RISCV Processor Issue Logic
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.exu

import chisel3._
import chisel3.util._

import freechips.rocketchip.config.Parameters
import freechips.rocketchip.util.Str

import FUConstants._
import boom.common._

/**
 * Specific type of issue unit
 *
 * @param params issue queue params
 * @param numWakeupPorts number of wakeup ports for the issue queue
 */
class IssueUnitStatic(
  params: IssueParams,
  numWakeupPorts: Int)
  (implicit p: Parameters)
  extends IssueUnit(params.numEntries, params.issueWidth, numWakeupPorts, params.iqType, params.dispatchWidth)
{
  //-------------------------------------------------------------
  // Set up the entry vectors

  val entry_wen_oh = Wire(Vec(numIssueSlots, UInt(dispatchWidth.W)))
  val entry_wen_oh_array = Array.fill(numIssueSlots,dispatchWidth){false.B}


  //-------------------------------------------------------------
  // Select spots to allocate the new dispatched uops

  val will_be_valid_issue    = (0 until numIssueSlots).map(i => issue_slots(i).will_be_valid)
  val will_be_valid_dispatch = (0 until dispatchWidth).map(i => io.dis_uops(i).valid &&
                                                        !dis_uops(i).exception &&
                                                        !dis_uops(i).is_fence &&
                                                        !dis_uops(i).is_fencei)
  var allocated = WireInit(VecInit(Seq.fill(dispatchWidth){false.B}))
  for (i <- 0 until numIssueSlots) {
		var next_allocated = Wire(Vec(dispatchWidth, Bool()))
    var can_allocate = !(issue_slots(i).valid)
    for (w <- 0 until dispatchWidth) {
			entry_wen_oh_array(i)(w) = can_allocate && !(allocated(w))
      next_allocated(w) := can_allocate | allocated(w)
      can_allocate = can_allocate && allocated(w)
    }
    allocated = next_allocated
  }


  //-------------------------------------------------------------
  // Adjust the newly selected slots and then assign them to the issue_slots array

  // if we can find an issue slot, do we actually need it?
  // also, translate from Scala data structures to Chisel Vecs
  for (i <- 0 until numIssueSlots) {
    val temp_uop_val = Wire(Vec(dispatchWidth, Bool()))
    for (w <- 0 until dispatchWidth) {
      temp_uop_val(w) := will_be_valid_dispatch(w) && entry_wen_oh_array(i)(w)
    }
    entry_wen_oh(i) := temp_uop_val.asUInt
  }

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).in_uop.valid := entry_wen_oh(i).orR
    issue_slots(i).in_uop.bits  := Mux1H(entry_wen_oh(i), dis_uops)
    issue_slots(i).clear        := false.B
  }


  //-------------------------------------------------------------
  // Dispatch/Entry Logic
  // did we find a spot to slide the new dispatched uops into?

  val will_be_available = (0 until numIssueSlots).map(i =>
                            (!issue_slots(i).will_be_valid || issue_slots(i).clear) && !(issue_slots(i).in_uop.valid))
  val num_available = PopCount(will_be_available)
  for (w <- 0 until dispatchWidth) {
    io.dis_uops(w).ready := allocated(w)
  }


  //-------------------------------------------------------------
  // Issue Select Logic

  // set default
  for (w <- 0 until issueWidth) {
    io.iss_valids(w) := false.B
    io.iss_uops(w)   := NullMicroOp
    // unsure if this is overkill
    io.iss_uops(w).prs1 := 0.U
    io.iss_uops(w).prs2 := 0.U
    io.iss_uops(w).prs3 := 0.U
    io.iss_uops(w).lrs1_rtype := RT_X
    io.iss_uops(w).lrs2_rtype := RT_X
  }

  val requests = issue_slots.map(s => s.request)
  val port_issued = Array.fill(issueWidth){Bool()}
  for (w <- 0 until issueWidth) {
    port_issued(w) = false.B
  }

  for (i <- 0 until numIssueSlots) {
    issue_slots(i).grant := false.B
    var uop_issued = false.B

    for (w <- 0 until issueWidth) {
      val can_allocate = (issue_slots(i).uop.fu_code & io.fu_types(w)) =/= 0.U

      when (requests(i) && !uop_issued && can_allocate && !port_issued(w)) {
        issue_slots(i).grant := true.B
        io.iss_valids(w) := true.B
        io.iss_uops(w) := issue_slots(i).uop
      }
      val was_port_issued_yet = port_issued(w)
      port_issued(w) = (requests(i) && !uop_issued && can_allocate) | port_issued(w)
      uop_issued = (requests(i) && can_allocate && !was_port_issued_yet) | uop_issued
    }
  }
}
