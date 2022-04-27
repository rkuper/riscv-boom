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
import chisel3.util.{log2Ceil, PopCount, MuxCase}

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
class IssueUnitIndexCollapsing(
  params: IssueParams,
  numWakeupPorts: Int)
  (implicit p: Parameters)
  extends IssueUnit(params.numEntries, params.issueWidth, numWakeupPorts, params.iqType, params.dispatchWidth)
{
  //-------------------------------------------------------------
  // Set up the indexes for each record
  val issue_indexes = RegInit(VecInit(Seq.fill(numIssueSlots) {
    val issue_index = Wire(new IndexedRecord(log2Ceil(numIssueSlots)))
    issue_index.index          := 0.U
    issue_index.valid          := false.B
    issue_index.will_be_valid  := false.B
    issue_index
  }))

  for (i <- 0 until numIssueSlots) {
    issue_indexes(i).will_be_valid := issue_slots(issue_indexes(i).index).will_be_valid
  }

  val dispatch_indexes = WireInit(VecInit(Seq.fill(numIssueSlots) {
    val dispatch_index = Wire(new IndexedRecord(log2Ceil(numIssueSlots)))
    dispatch_index.index          := 0.U
    dispatch_index.valid          := false.B
    dispatch_index.will_be_valid  := false.B
    dispatch_index
  }))


  //-------------------------------------------------------------
  // FIXME BROKEN: Figure out how much to shift entries by

  // val maxShift = dispatchWidth
  // val vacants = issue_indexes.map(s => !(s.valid))
  // val shamts_oh = Array.fill(numIssueSlots) {Wire(UInt(width=maxShift.W))}

  // // Determine how much to shift index slots by and saturate the shift number
  // // by the max shift amount (number of incoming uops from dispatch)
  // def SaturatingCounterOH(count_oh:UInt, inc: Bool, max: Int): UInt = {
  //    val next = Wire(UInt(width=max.W))
  //    next := count_oh
  //    when ((count_oh < max.U) && inc) {
  //      next := count_oh + 1.U
  //    }
  //    next
  // }
  // shamts_oh(0) := 0.U
  // for (i <- 1 until numIssueSlots) {
  //   shamts_oh(i) := SaturatingCounterOH(shamts_oh(i-1), vacants(i-1), maxShift)
  // }

  // for (i <- (numIssueSlots-1) to 0 by -1) {
  //   val move_to_index = i.U - shamts_oh(i)
  //   issue_indexes(move_to_index) := issue_indexes(i)
  // }


  //-------------------------------------------------------------
  // Figure out how much to shift entries by
  val maxShift = dispatchWidth
  val vacants = issue_indexes.map(s => !(s.valid)) ++ io.dis_uops.map(_.valid).map(!_.asBool)
  val shamts_oh = Array.fill(numIssueSlots+dispatchWidth) {Wire(UInt(width=maxShift.W))}
  // track how many to shift up this entry by by counting previous vacant spots
  def SaturatingCounterOH(count_oh:UInt, inc: Bool, max: Int): UInt = {
     val next = Wire(UInt(width=max.W))
     next := count_oh
     when (count_oh === 0.U && inc) {
       next := 1.U
     } .elsewhen (!count_oh(max-1) && inc) {
       next := (count_oh << 1.U)
     }
     next
  }
  shamts_oh(0) := 0.U
  for (i <- 1 until numIssueSlots + dispatchWidth) {
    shamts_oh(i) := SaturatingCounterOH(shamts_oh(i-1), vacants(i-1), maxShift)
  }


  //-------------------------------------------------------------
  // Shift/Collapse the issue_indexes
  val will_be_valid = (0 until numIssueSlots).map(i => issue_indexes(i).will_be_valid) ++
                      (0 until dispatchWidth).map(i => io.dis_uops(i).valid &&
                                                        !dis_uops(i).exception &&
                                                        !dis_uops(i).is_fence &&
                                                        !dis_uops(i).is_fencei)

  val all_indexes = issue_indexes.map(s=>s.index) ++ dispatch_indexes.map(s=>s.index)
  for (i <- 0 until numIssueSlots) {
    for (j <- 1 to maxShift by 1) {
      when (shamts_oh(i+j) === (1 << (j-1)).U) {
        issue_indexes(i).valid         := will_be_valid(i+j)
        issue_indexes(i).will_be_valid := will_be_valid(i+j)
        issue_indexes(i).index         := all_indexes(i+j)
      }
    }
    issue_indexes(i).valid         := shamts_oh(i) =/= 0.U
    issue_indexes(i).will_be_valid := shamts_oh(i) =/= 0.U
  }


  //-------------------------------------------------------------
  // Insert the dispatched uops to anywhere they can fit and update the issue_indexes

  // Number of actively used issue slots to determine next open slot in issue_indexes
  val num_used  = PopCount((0 until numIssueSlots).map(i => issue_indexes(i).valid))
  // Temporary wire for determining if the dispatch uop has been allocated yet in this section only
  var allocated = WireInit(VecInit(Seq.fill(dispatchWidth){false.B}))

  for (i <- 0 until numIssueSlots) {
    var next_allocated = Wire(Vec(dispatchWidth, Bool()))
    var can_allocate   = !(issue_slots(i).valid)
    var slot_allocated = false.B
    for (w <- 0 until dispatchWidth) {
      // TODO: What is this, why this order
      can_allocate       = can_allocate && !allocated(w)
      next_allocated(w) := can_allocate | allocated(w)

      // Since this slot is open, assign the dispatched uop to it
      when (!slot_allocated && can_allocate) {
        issue_slots(i).in_uop.valid         :=  true.B
        issue_slots(i).in_uop.bits          :=  dis_uops(w)
        issue_indexes(num_used + w.U).valid :=  true.B
        issue_indexes(num_used + w.U).index :=  i.U
        slot_allocated                       =  true.B
      }
    }
    allocated = next_allocated
  }


  //-------------------------------------------------------------
  // Dispatch/Entry Logic
  // did we find a spot to slide the new dispatched uops into?
  val will_be_available = (0 until numIssueSlots).map(i =>
                            (!issue_slots(i).will_be_valid || issue_slots(i).clear) && !(issue_slots(i).in_uop.valid))
  val num_available     = PopCount(will_be_available)
  for (w <- 0 until dispatchWidth) {
    io.dis_uops(w).ready      := RegNext(num_available > w.U)
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

  val requests = (0 until numIssueSlots).map(i => issue_slots(issue_indexes(i).index).request)
  val port_issued = Array.fill(issueWidth){Bool()}
  for (w <- 0 until issueWidth) {
    port_issued(w) = false.B
  }

  for (i <- 0 until numIssueSlots) {
    val issue_index = issue_indexes(i).index
    issue_slots(issue_index).grant := false.B
    var uop_issued = false.B

    for (w <- 0 until issueWidth) {
      val can_allocate = (issue_slots(issue_index).uop.fu_code & io.fu_types(w)) =/= 0.U
      when (requests(i) && !uop_issued && can_allocate && !port_issued(w)) {
        issue_slots(issue_index).grant := true.B
        io.iss_valids(w) := true.B
        io.iss_uops(w) := issue_slots(issue_index).uop
      }
      val was_port_issued_yet = port_issued(w)
      port_issued(w) = (requests(i) && !uop_issued && can_allocate) | port_issued(w)
      uop_issued = (requests(i) && can_allocate && !was_port_issued_yet) | uop_issued
    }
  }
}



class IndexedRecord(val IQWidth: Int) extends Bundle {
  val valid = Bool()
  val index = UInt(IQWidth.W)
  val will_be_valid = Bool()
}
