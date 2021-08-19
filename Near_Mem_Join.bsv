package Near_Mem_Join;

import FIFOF        :: *;
import Vector :: *;
import SpecialFIFOs :: *;
import SourceSink :: *;

// Import Near_Mem_IFC from another Near_Mem implementation,
// presumably WB_L1_L2 or whichever we are replacing with this
import MMU_Cache_Common :: *;
import Near_Mem_IFC :: *;
import Near_Mem_Caches :: *;
import Near_Mem_Join_Types :: *;
import Near_Mem_TCM_IFC :: *;
import Near_Mem_TCM_Merged :: *;

import SoC_Map :: *;
import Routable :: *;
import ISA_Decls :: *;

typedef enum {
   TCM,
   CACHES
} Mem_Dest deriving (Bits, Eq, FShow);

// implement L1_WT cache interface
// TODO this can't handle receiving a request when there is already an
// outstanding request. This should never happen because the CPU should not
// allow it, but if it does happen then it will cause issues here.
module mkNear_Mem_Join (Near_Mem_IFC);
   Reg #(Bit #(4)) rg_verbosity <- mkReg (0);

   FIFOF #(IMem_Req) f_imem_req <- mkBypassFIFOF;
   let ug_imem_req_sink <- toUnguardedSink (f_imem_req);
   let ug_imem_req_source <- toUnguardedSource (f_imem_req, ?);
   Wire #(Bool) w_imem_req_active <- mkDWire (False);
   Reg #(Mem_Dest) rg_imem_dest <- mkRegU;

   FIFOF #(DMem_Req) f_dmem_req <- mkBypassFIFOF;
   Wire #(Bool) w_dmem_req_active <- mkDWire (False);
   Reg #(Mem_Dest) rg_dmem_dest <- mkRegU;

   Reg #(Mem_Dest) rg_tohost_src <- mkReg (TCM);

   SoC_Map_IFC soc_map <- mkSoC_Map;

   Near_Mem_IFC near_mem_caches <- mkNear_Mem;
   Near_Mem_TCM_IFC near_mem_tcm <- mkNear_Mem_TCM;
   Vector #(TExp #(SizeOf #(Mem_Dest)), IMem_IFC) v_imems = newVector;
   v_imems[pack (TCM)] = near_mem_tcm.imem;
   v_imems[pack (CACHES)] = near_mem_caches.imem;

   Vector #(TExp #(SizeOf #(Mem_Dest)), DMem_IFC) v_dmems = newVector;
   v_dmems[pack (TCM)] = near_mem_tcm.dmem;
   v_dmems[pack (CACHES)] = near_mem_caches.dmem;

   // select between TCM or cache imem interface
   function IMem_IFC fn_sel_cache_or_tcm_imem (Mem_Dest dest);
      return v_imems[pack (dest)];
   endfunction

   // select between TCM or cache dmem interface
   function DMem_IFC fn_sel_cache_or_tcm_dmem (Mem_Dest dest);
      return v_dmems[pack (dest)];
   endfunction

   // forward imem requests to the correct destination
   rule rl_forward_imem (w_imem_req_active);
      let req = ug_imem_req_source.peek;
      ug_imem_req_source.drop;
      let dest = inRange (soc_map.m_tcm_addr_range, req.addr) ? TCM
                                                            : CACHES;
      rg_imem_dest <= dest;
      (fn_sel_cache_or_tcm_imem (dest)).req ( req.f3
                                            , req.addr
`ifdef ISA_PRIV_S
                                            , req.priv
                                            , req.sstatus_SUM
                                            , req.mstatus_MXR
                                            , req.satp
`endif
`ifdef RVFI_DII
                                            , req.seq_req
`endif
                                            );
      if (rg_verbosity > 1) begin
         $display ("rl_forward_imem: forwarding request to memory");
         $display ("    req: ", fshow (req));
         $display ("    dest: ", fshow (dest));
      end
   endrule

   rule rl_forward_dmem;
      let req = f_dmem_req.first;
      f_dmem_req.deq;
      let dest = inRange (soc_map.m_tcm_addr_range, req.addr) ? TCM
                                                              : CACHES;
      rg_dmem_dest <= dest;
      fn_sel_cache_or_tcm_dmem (dest).req ( req.op
                                          , req.f3
                                          , req.is_unsigned
`ifdef ISA_A
                                          , req.amo_funct5
`endif
                                          , req.addr
                                          , req.store_value
`ifdef ISA_PRIV_S
                                          , req.priv
                                          , req.sstatus_SUM
                                          , req.mstatus_MXR
                                          , req.satp
`endif
                                          );
      if (rg_verbosity > 1) begin
         $display ("rl_forward_dmem: forwarding request to memory");
         $display ("    req: ", fshow (req));
         $display ("    dest: ", fshow (dest));
      end
   endrule

   rule rl_debug (fn_sel_cache_or_tcm_imem(rg_imem_dest).valid);
      if (rg_verbosity > 1) begin
         $display ("mem_join debug, dest: ", fshow (rg_imem_dest));
         $display ("    valid: ", fshow (fn_sel_cache_or_tcm_imem(rg_imem_dest).valid));
         $display ("    instr: ", fshow (fn_sel_cache_or_tcm_imem(rg_imem_dest).instr));
      end
   endrule




   interface server_reset = near_mem_caches.server_reset;

   interface IMem_IFC imem;
      method Action req ( Bit #(3)  f3
                        , WordXL    addr
`ifdef ISA_PRIV_S
                        , Priv_Mode priv
                        , Bit #(1)  sstatus_SUM
                        , Bit #(1)  mstatus_MXR
                        , WordXL    satp
`endif
`ifdef RVFI_DII
                        , Dii_Id seq_req
`endif
                        );
         ug_imem_req_sink.put (Near_Mem_Join_Types::IMem_Req { f3: f3
                                  , addr: addr
`ifdef ISA_PRIV_S
                                  , priv: priv
                                  , sstatus_SUM: sstatus_SUM
                                  , mstatus_MXR: mstatus_MXR
                                  , satp: satp
`endif
`ifdef RVFI_DII
                                  , seq_req: seq_req
`endif
                                  });
         w_imem_req_active <= True;
         $display ("joint mem request");
      endmethod

      method Action commit;
         if (rg_imem_dest == TCM) begin
            near_mem_tcm.imem.commit;
         end else begin
            near_mem_caches.imem.commit;
         end
      endmethod

      method Bool valid          = fn_sel_cache_or_tcm_imem (rg_imem_dest).valid;
      method Bool is_i32_not_i16 = fn_sel_cache_or_tcm_imem (rg_imem_dest).is_i32_not_i16;
      method WordXL pc           = fn_sel_cache_or_tcm_imem (rg_imem_dest).pc;
      method
`ifdef RVFI_DII
                                 Tuple2 #(Instr, Dii_Id) instr
`else
                                 Instr    instr
`endif
                                                    = fn_sel_cache_or_tcm_imem (rg_imem_dest).instr;
      method Bool exc            = fn_sel_cache_or_tcm_imem (rg_imem_dest).exc;
      method Exc_Code exc_code   = fn_sel_cache_or_tcm_imem (rg_imem_dest).exc_code;
      method WordXL tval         = fn_sel_cache_or_tcm_imem (rg_imem_dest).tval;

`ifdef PERFORMANCE_MONITORING
      method EventsCache events;
         EventsCache evts = EventsCache { evt_LD           :  near_mem_caches.imem.events.evt_LD
                                                           || near_mem_tcm.imem.events.evt_LD
                                        , evt_LD_MISS      :  near_mem_caches.imem.events.evt_LD_MISS
                                                           || near_mem_tcm.imem.events.evt_LD_MISS
                                        , evt_LD_MISS_LAT  :  near_mem_caches.imem.events.evt_LD_MISS_LAT
                                                           || near_mem_tcm.imem.events.evt_LD_MISS_LAT
                                        , evt_ST           :  near_mem_caches.imem.events.evt_ST
                                                           || near_mem_tcm.imem.events.evt_ST
                                        , evt_ST_MISS      :  near_mem_caches.imem.events.evt_ST_MISS
                                                           || near_mem_tcm.imem.events.evt_ST_MISS
                                        , evt_ST_MISS_LAT  :  near_mem_caches.imem.events.evt_ST_MISS_LAT
                                                           || near_mem_tcm.imem.events.evt_ST_MISS_LAT
                                        , evt_AMO          :  near_mem_caches.imem.events.evt_AMO
                                                           || near_mem_tcm.imem.events.evt_AMO
                                        , evt_AMO_MISS     :  near_mem_caches.imem.events.evt_AMO_MISS
                                                           || near_mem_tcm.imem.events.evt_AMO_MISS
                                        , evt_AMO_MISS_LAT :  near_mem_caches.imem.events.evt_AMO_MISS_LAT
                                                           || near_mem_tcm.imem.events.evt_AMO_MISS_LAT
                                        , evt_TLB          :  near_mem_caches.imem.events.evt_TLB
                                                           || near_mem_tcm.imem.events.evt_TLB
                                        , evt_TLB_MISS     :  near_mem_caches.imem.events.evt_TLB_MISS
                                                           || near_mem_tcm.imem.events.evt_TLB_MISS
                                        , evt_TLB_MISS_LAT :  near_mem_caches.imem.events.evt_TLB_MISS_LAT
                                                           || near_mem_tcm.imem.events.evt_TLB_MISS_LAT
                                        , evt_TLB_FLUSH    :  near_mem_caches.imem.events.evt_TLB_FLUSH
                                                           || near_mem_tcm.imem.events.evt_TLB_FLUSH
                                        , evt_EVICT        :  near_mem_caches.imem.events.evt_EVICT
                                                           || near_mem_tcm.imem.events.evt_EVICT
                                        };
         return evts;
      endmethod
`endif
   endinterface

   interface AXI4_Master imem_master = near_mem_caches.imem_master;

   interface DMem_IFC dmem;
      // CPU side: DMem request
      method Action  req ( CacheOp op
                         , Bit #(3) f3
                         , Bool is_unsigned
`ifdef ISA_A
                         , Bit #(5) amo_funct5
`endif
                         , Addr addr
                         , Tuple2#(Bool, Bit #(128)) store_value
                           // The following  args for VM
`ifdef ISA_PRIV_S
                         , Priv_Mode  priv
                         , Bit #(1)   sstatus_SUM
                         , Bit #(1)   mstatus_MXR
                         , WordXL     satp
`endif
                          );    // { VM_Mode, ASID, PPN_for_page_table }
         f_dmem_req.enq (DMem_Req { op         : op
                                  , f3         : f3
                                  , is_unsigned: is_unsigned
`ifdef ISA_A
                                  , amo_funct5 : amo_funct5
`endif
                                  , addr       : addr
                                  , store_value: store_value
`ifdef ISA_PRIV_S
                                  , priv       : priv
                                  , sstatus_SUM: sstatus_SUM
                                  , mstatus_MXR: mstatus_MXR
                                  , satp       : satp
`endif
                                  });
         w_dmem_req_active <= True;
      endmethod

      method Action commit;
         if (rg_dmem_dest == TCM) begin
            near_mem_tcm.dmem.commit;
         end else begin
            near_mem_caches.dmem.commit;
         end
      endmethod

      // CPU side: DMem response
      method Bool                      valid      = fn_sel_cache_or_tcm_dmem (rg_dmem_dest).valid;
      method Tuple2#(Bool, Bit #(128)) word128    = fn_sel_cache_or_tcm_dmem (rg_dmem_dest).word128;      // Load-value
      method Bit #(128)                st_amo_val = fn_sel_cache_or_tcm_dmem (rg_dmem_dest).st_amo_val;  // Final store-value for ST, SC, AMO
      method Bool                      exc        = fn_sel_cache_or_tcm_dmem (rg_dmem_dest).exc;
      method Exc_Code                  exc_code   = fn_sel_cache_or_tcm_dmem (rg_dmem_dest).exc_code;

`ifdef PERFORMANCE_MONITORING
      method EventsCache events;
         EventsCache evts = EventsCache { evt_LD           :  near_mem_caches.dmem.events.evt_LD
                                                           || near_mem_tcm.dmem.events.evt_LD
                                        , evt_LD_MISS      :  near_mem_caches.dmem.events.evt_LD_MISS
                                                           || near_mem_tcm.dmem.events.evt_LD_MISS
                                        , evt_LD_MISS_LAT  :  near_mem_caches.dmem.events.evt_LD_MISS_LAT
                                                           || near_mem_tcm.dmem.events.evt_LD_MISS_LAT
                                        , evt_ST           :  near_mem_caches.dmem.events.evt_ST
                                                           || near_mem_tcm.dmem.events.evt_ST
                                        , evt_ST_MISS      :  near_mem_caches.dmem.events.evt_ST_MISS
                                                           || near_mem_tcm.dmem.events.evt_ST_MISS
                                        , evt_ST_MISS_LAT  :  near_mem_caches.dmem.events.evt_ST_MISS_LAT
                                                           || near_mem_tcm.dmem.events.evt_ST_MISS_LAT
                                        , evt_AMO          :  near_mem_caches.dmem.events.evt_AMO
                                                           || near_mem_tcm.dmem.events.evt_AMO
                                        , evt_AMO_MISS     :  near_mem_caches.dmem.events.evt_AMO_MISS
                                                           || near_mem_tcm.dmem.events.evt_AMO_MISS
                                        , evt_AMO_MISS_LAT :  near_mem_caches.dmem.events.evt_AMO_MISS_LAT
                                                           || near_mem_tcm.dmem.events.evt_AMO_MISS_LAT
                                        , evt_TLB          :  near_mem_caches.dmem.events.evt_TLB
                                                           || near_mem_tcm.dmem.events.evt_TLB
                                        , evt_TLB_MISS     :  near_mem_caches.dmem.events.evt_TLB_MISS
                                                           || near_mem_tcm.dmem.events.evt_TLB_MISS
                                        , evt_TLB_MISS_LAT :  near_mem_caches.dmem.events.evt_TLB_MISS_LAT
                                                           || near_mem_tcm.dmem.events.evt_TLB_MISS_LAT
                                        , evt_TLB_FLUSH    :  near_mem_caches.dmem.events.evt_TLB_FLUSH
                                                           || near_mem_tcm.dmem.events.evt_TLB_FLUSH
                                        , evt_EVICT        :  near_mem_caches.dmem.events.evt_EVICT
                                                           || near_mem_tcm.dmem.events.evt_EVICT
                                        };
         return evts;
      endmethod
`endif

   endinterface

   interface mem_master = near_mem_caches.mem_master;

   interface server_fence_i = near_mem_caches.server_fence_i;
   interface server_fence = near_mem_caches.server_fence;

   interface dma_server = near_mem_tcm.tcm_dma_server;

   method ma_ddr4_ready = near_mem_caches.ma_ddr4_ready;

   method mv_status = near_mem_caches.mv_status;

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      let dest = inRange (soc_map.m_tcm_addr_range, tohost_addr) ? TCM
                                                                 : CACHES;
      rg_tohost_src <= dest;
      if (dest == CACHES) begin
         $display ("%m set_watch_tohost CACHES  watch_tohost: ", fshow (watch_tohost), "  addr: :", fshow (tohost_addr));
         near_mem_caches.set_watch_tohost (watch_tohost, tohost_addr);
         near_mem_tcm.set_watch_tohost (False, tohost_addr);
      end else begin
         $display ("%m set_watch_tohost TCM  watch_tohost: ", fshow (watch_tohost), "  addr: :", fshow (tohost_addr));
         near_mem_caches.set_watch_tohost (False, tohost_addr);
         near_mem_tcm.set_watch_tohost (watch_tohost, tohost_addr);
      end
   endmethod

   method Bit #(64) mv_tohost_value;
      if (rg_tohost_src == CACHES) begin
         return near_mem_caches.mv_tohost_value;
      end else begin
         return near_mem_tcm.mv_tohost_value;
      end
   endmethod
`endif

endmodule


endpackage
