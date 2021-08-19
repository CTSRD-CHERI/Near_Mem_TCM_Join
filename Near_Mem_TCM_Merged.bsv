// TODO not sure about copyright for this file
package Near_Mem_TCM_Merged;

// ================================================================
// BSV lib imports

import ConfigReg        :: *;
import FIFOF            :: *;
import GetPut           :: *;
import ClientServer     :: *;
import BRAMCore         :: *;
import Connectable      :: *;
import Vector           :: *;
import Memory :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;
import Near_Mem_TCM_IFC :: *;
import MMU_Cache_Common :: *;
// import cache interface
import Near_Mem_IFC :: *;


import AXI :: *;
import SourceSink :: *;
import SpecialRegs  :: *;

`ifdef ISA_CHERI
import CHERICap         :: *;
import CHERICC_Fat      :: *;
`endif
import DReg :: *;

import SoC_Map          :: *;

`ifdef RVFI_DII
import Mem_Model :: *;
`endif

// ================================================================
// Exports

export mkNear_Mem_TCM;

// ================================================================
// BRAM config constants

typedef TMul #(32, 1024) Words_per_BRAM;

// ================================================================
// TCM interface

// Merged interface, containing both an IMem_IFC and a DMem_IFC
// Also has an AXI Slave port, for direct access from peripherals like the
// debug module
// Requests made to this module do not get forwarded if the requested address
// is not within the TCM's address range; instead, the responses are filled
// with ?s. It is expected that only requests that have this module as their
// final destination should make it to this module
// As such this module does not have any bus master interfaces.
interface IDTCM_AXI_IFC #( numeric type sid_
                         , numeric type addr_
                         , numeric type data_
                         , numeric type awuser_
                         , numeric type wuser_
                         , numeric type buser_
                         , numeric type aruser_
                         , numeric type ruser_
                         );
   method Action reset;

   interface IMem_IFC imem;
   interface DMem_IFC dmem;

   interface AXI4_Slave #( sid_, addr_, data_
                         , awuser_, wuser_, buser_
                         , aruser_, ruser_) tcm_dma_server;

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
   method Bit #(64) mv_tohost_value;
`endif
endinterface

interface ITCM_IFC;
   method Action reset;
   interface IMem_IFC imem;
endinterface

interface DTCM_IFC;
   method Action reset;
   interface DMem_IFC dmem;
endinterface

interface DTCM_AXI_IFC #( numeric type sid_
                        , numeric type addr_
                        , numeric type data_
                        , numeric type awuser_
                        , numeric type wuser_
                        , numeric type buser_
                        , numeric type aruser_
                        , numeric type ruser_);
   method Action reset;
   interface DMem_IFC dmem;
   interface AXI4_Slave #( sid_, addr_, data_
                         , awuser_, wuser_, buser_
                         , aruser_, ruser_) tcm_dma_server;

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
   method Bit #(64) mv_tohost_value;
`endif
endinterface


// ================================================================
// Near_Mem_TCM module

(* synthesize *)
module mkNear_Mem_TCM (Near_Mem_TCM_IFC);
   Bit #(2) i_verbosity = 0;
   Bit #(2) d_verbosity = 0;

   FIFOF #(Token) f_reset_rsps <- mkFIFOF1;

   // ----------------
   // The TCM instantiation
   let idtcm <- mkIDTCM_AXI (i_verbosity, d_verbosity);

   // Fence request/response queues
   FIFOF #(Token) f_fence_req_rsp <- mkFIFOF1;

   // ================================================================
   // INTERFACE

   // ----------------
   // Reset
   interface Server server_reset;
      interface Put request;
         method Action put (Token t);
            idtcm.reset;
            f_reset_rsps.enq (?);
         endmethod
      endinterface

      interface Get response = toGet (f_reset_rsps);
   endinterface

   // ----------------
   // CPU side

   interface imem = idtcm.imem;
   interface dmem = idtcm.dmem;

   // ----------------
   // Fabric side
   interface tcm_dma_server = idtcm.tcm_dma_server;

   // ----------------
   // Fence.I, Fence -- all fences are nops, right?
   interface Server server_fence_i;
      interface Put request;
         method Action put (Token t);
            noAction;
         endmethod
      endinterface
      interface Get response;
         method ActionValue #(Token) get;
            noAction;
            return (?);
         endmethod
      endinterface
   endinterface

   interface Server server_fence;
      interface Put request;
         method Action put (Fence_Ordering fo);
            f_fence_req_rsp.enq (?);
         endmethod
      endinterface
      interface response = toGet (f_fence_req_rsp);
   endinterface

`ifdef ISA_PRIV_S
   // ----------------
   // SFENCE_VMA: flush TLBs (no op in this module)
   method Action sfence_vma;
      noAction;
   endmethod
`endif

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr

`ifdef WATCH_TOHOST
   // TODO this is not implemented
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      idtcm.set_watch_tohost (watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value = idtcm.mv_tohost_value;
`endif

endmodule: mkNear_Mem_TCM

// ================================================================
// Internal types and constants

// Convert the CPU address we get in requests to a local address suitable
// for indexing into the TCM
// The local address is a TCM-word address (ie it is an index into an array
// holding data_-sized elements)
// The CPU address is assumed to be a byte address, so it gets right-shifted
// to be a TCM-word address.
// If the local address is shorter, then the CPU address is truncated.
// If the local address is longer, then the CPU address is zeroExtended.
function Bit #(addr_) fv_cpu_addr_to_local_addr (WordXL addr, Integer data_);
   Integer num_bytes_in_tcm_word = data_ / valueOf (SizeOf #(Byte));
   Integer rshift_amt = log2 (num_bytes_in_tcm_word);
   Bit #(TAdd #(SizeOf #(WordXL), addr_)) res = zeroExtend (pack (addr));
   Bit #(TAdd #(SizeOf #(WordXL), addr_)) res_rshift = zeroExtend (res >> rshift_amt);
   Integer hi = valueOf (addr_) - 1;
   return res_rshift [hi:0];
endfunction

function Bit #(be_) fv_req_to_byte_mask (MMU_Cache_Req req)
   provisos (Add#(a__, TLog#(be_), 64));
   Bit #(TLog #(be_)) offset = truncate (req.va);
   Bit #(be_) mask = ~(~0 << (1 << (req.width_code)));
   mask = mask << offset;
   return mask;
endfunction

function Bit #(TMul #(byte_sz, SizeOf #(Byte))) fv_byte_mask_to_bit_mask (Bit #(byte_sz_) byte_mask)
   provisos (Add#(a__, SizeOf #(Byte), TMul#(byte_sz, SizeOf #(Byte))));
   Byte zero = 0;
   Bit #(TMul #(byte_sz, SizeOf #(Byte))) res = 0;
   for (Integer i = 0; i < valueOf (byte_sz_); i = i+1) begin
      res = res << valueOf (SizeOf #(Byte));
      res = res | zeroExtend (byte_mask[valueOf (byte_sz_) - i - 1] == 1'b1 ? ~zero : zero);
   end
   return res;
endfunction

`ifdef ISA_A
// Lifted from elsewhere (L1 WT caches)
// original function extracted the value from the word using the address - this
// version doesn't need to
function Tuple2 #( Tuple2 #(Bool, Bit #(128))
                 , Tuple2 #(Bool, Bit #(128))) fv_amo_op ( Bit #(3) funct3
                                                         , Bit #(5) funct5
                                                         , Tuple2 #(Bool, Bit #(128)) ld_val_cap
                                                         , Tuple2 #(Bool, Bit #(128)) st_val_cap);
   Bit #(128) ld_val   = tpl_2(ld_val_cap);
   Bit #(128) st_val   = tpl_2(st_val_cap);
   Bit #(64)  ld_val_w = truncate (ld_val);
   Bit #(64)  st_val_w = truncate (st_val);
   Int #(64)  ld_val_i = unpack (ld_val_w);
   Int #(64)  st_val_i = unpack (st_val_w);
   if (funct3 == f3_AMO_W) begin
      ld_val_i = unpack (signExtend (ld_val_w[31:0]));
      st_val_i = unpack (signExtend (st_val_w[31:0]));
      ld_val_w = zeroExtend (ld_val_w[31:0]);
      st_val_w = zeroExtend (st_val_w[31:0]);
   end

   Bit #(128) new_st_val_128;
   Bool new_st_tag = False;
   Bool old_ld_tag = False;
   if (funct3 == f3_AMO_CAP) begin
      new_st_val_128 = st_val;
      new_st_tag = tpl_1 (st_val_cap);
      old_ld_tag = tpl_1 (ld_val_cap);
   end else begin
      Bit #(64) new_st_val_64 = ?;
      case (funct5)
         f5_AMO_SWAP: new_st_val_64 = st_val_w;
         f5_AMO_ADD:  new_st_val_64 = pack (ld_val_i + st_val_i);
         f5_AMO_XOR:  new_st_val_64 = ld_val_w ^ st_val_w;
         f5_AMO_AND:  new_st_val_64 = ld_val_w & st_val_w;
         f5_AMO_OR:   new_st_val_64 = ld_val_w | st_val_w;
         // TODO this uses 2 each of the greater-than and less-than comparators
         // in hardware, and could be simplified to use just 1
         f5_AMO_MINU: new_st_val_64 = (ld_val_w < st_val_w) ? ld_val_w : st_val_w;
         f5_AMO_MAXU: new_st_val_64 = (ld_val_w > st_val_w) ? ld_val_w : st_val_w;
         f5_AMO_MIN:  new_st_val_64 = (ld_val_i < st_val_i) ? ld_val_w : st_val_w;
         f5_AMO_MAX:  new_st_val_64 = (ld_val_i > st_val_i) ? ld_val_w : st_val_w;
      endcase

      if (funct3 == f3_AMO_W) begin
         new_st_val_64 = zeroExtend (new_st_val_64[31:0]);
      end

      new_st_val_128 = zeroExtend (new_st_val_64);
   end

   return tuple2 ( tuple2 (old_ld_tag, ld_val)
                 , tuple2 (new_st_tag, zeroExtend (new_st_val_128)));
endfunction
`endif // ISA_A

// An IDTCM module which also has an AXI interface. Requests on the CPU
// interface take priority over requests on the AXI interface.
module mkIDTCM_AXI #(Bit #(2) i_verbosity, Bit #(2) d_verbosity)
                    (IDTCM_AXI_IFC #( sid_, addr_, data_
                                    , awuser_, wuser_, buser_
                                    , aruser_, ruser_))
                    provisos ( Add #(a__, data_, 128)
                             , Add #(b__, addr_, 64)
                             , Add #(c__, TLog#(data_), 64)
                             , Add #(d__, TLog#(data_), addr_)
                             );
`ifdef ISA_CHERI
`ifdef RV64
`ifdef RVFI_DII
   // ISA_CHERI, RV64, RVFI_DII
   // want to use mem_model as the primitive for this since it has reset

   Integer words_per_bram = valueOf (Words_per_BRAM);
   Integer addr_sz = log2(words_per_bram);
   Vector #(4, Vector #(4, Mem_Model_Gen_IFC #( TLog #(Words_per_BRAM)
                                              , 8
                                              , 0, TSub #(Words_per_BRAM, 1)
                                              , 0, 0
                                              )))
      v_v_mem_models = newVector;
   for (Integer i = 0; i < 4; i = i+1) begin
      Vector #(4, Mem_Model_Gen_IFC #( TLog #(Words_per_BRAM)
                                     , 8
                                     , 0, TSub #(Words_per_BRAM, 1)
                                     , 0, 0
                                     ))
         v_mem_models = newVector;
      for (Integer j = 0; j < 4; j = j+1) begin
         v_mem_models[j] <- mkMem_Model_General (words_per_bram);
      end
      v_v_mem_models[i] = v_mem_models;
   end
   Vector #(4, BRAM_PORT_BE #(Bit #(TLog #(Words_per_BRAM)), Bit #(32), 4)) v_brams = newVector;
   for (Integer i = 0; i < 4; i = i+1) begin
      v_brams[i] <- mkBRAM_from_Mem_Models (v_v_mem_models[i]);
   end

   Vector #(1, Mem_Model_Gen_IFC #( TLog #(Words_per_BRAM)
                                  , 8
                                  , 0, TSub #(Words_per_BRAM, 1)
                                  , 0, 0
                                  ))
      v_tag_models = newVector;
   v_tag_models[0] <- mkMem_Model_General (words_per_bram);
   Vector #(1, BRAM_PORT_BE #(Bit #(TLog #(Words_per_BRAM)), Bit #(8), 1)) v_tag_brams = newVector;
   v_tag_brams[0] <- mkBRAM_from_Mem_Models (v_tag_models);


   let dport <- mkMergeBRAMs (v_brams, v_tag_brams);

   BRAM_PORT_BE #(Bit #(32), Bit #(136), 17) iport <- mkDummyBRAM;
`else
   // ISA_CHERI, RV64, no RVFI_DII
   Integer words_per_bram = valueOf (Words_per_BRAM);
   Integer addr_sz = log2 (words_per_bram);

   Vector #(4, BRAM_DUAL_PORT_BE #(Bit #(TLog #(Words_per_BRAM)), Bit #(32), 4)) v_brams = newVector;
   for (Integer i = 0; i < 4; i = i+1) begin
`ifdef BRAM_LOAD
      v_brams[i] <- mkBRAMCore2BELoad (words_per_bram, False, "Mem-TCM-" + integerToString(i) + ".hex", False);
`else
      v_brams[i] <- mkBRAMCore2BE (words_per_bram, False);
`endif
   end

   Vector #(1, BRAM_DUAL_PORT_BE #(Bit #(TLog #(Words_per_BRAM)), Bit #(8), 1)) v_tag_brams = newVector;
`ifdef BRAM_LOAD
   v_tag_brams[0] <- mkBRAMCore2BELoad (words_per_bram, False, "Mem-TCM-tags-0.hex", False);
`else
   v_tag_brams[0] <- mkBRAMCore2BE (words_per_bram, False);
`endif

   let tcm <- mkMergeDualPortBRAMs (v_brams, v_tag_brams);
   let dport = tcm.a;
   let iport = tcm.b;
`endif
`endif
`endif

// TODO populate this
`ifdef ISA_CHERI
`ifndef RV64
`ifdef RVFI_DII
   // ISA_CHERI, no RV64, RVFI_DII
`else
   // ISA_CHERI, no RV64, no RVFI_DII
`endif
`endif
`endif

// TODO populate this
`ifndef ISA_CHERI
`ifdef RV64
`ifdef RVFI_DII
   // no ISA_CHERI, RV64, RVFI_DII
`else
   // no ISA_CHERI, RV64, no RVFI_DII
`endif
`endif
`endif

// TODO populate this
`ifndef ISA_CHERI
`ifndef RV64
`ifdef RVFI_DII
   // no ISA_CHERI, no RV64, RVFI_DII
`else
   // no ISA_CHERI, no RV64, no RVFI_DII
`endif
`endif
`endif

   // The instruction port to the TCM. This port should only ever be read from.
   let itcm <- mkITCM_from_BRAM (iport, i_verbosity);
   // The data port to the TCM. This port is also used to handle accesses from
   // the fabric
   let dtcm <- mkDTCM_AXI_from_BRAM (dport, d_verbosity);

   // -------------------------------------------
   // Instruction accesses


   // Inteface
   // TODO reset submodules
   method Action reset = noAction;

   interface IMem_IFC imem = itcm.imem;
   interface DMem_IFC dmem = dtcm.dmem;
   interface AXI4_Slave tcm_dma_server = dtcm.tcm_dma_server;

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      dtcm.set_watch_tohost (watch_tohost, tohost_addr);
   endmethod

   method Bit #(64) mv_tohost_value = dtcm.mv_tohost_value;
`endif

endmodule


module mkMergeDualPortBRAMs #( Vector #(4, BRAM_DUAL_PORT_BE #(Bit #(addr_), Bit #(32), 4)) data_ports
                             , Vector #(1, BRAM_DUAL_PORT_BE #(Bit #(addr_), Bit #( 8), 1)) tag_ports)
                             (BRAM_DUAL_PORT_BE #( Bit #(addr_)
                                                 , Bit #(total_data_)
                                                 , total_be_))
                             provisos ( NumAlias #(total_data_, TAdd #(TMul #(32, 4), TMul #(8, 1)))
                                      , NumAlias #(total_be_  , TAdd #(TMul #( 4, 4), TMul #(1, 1))));
   Vector #(4, BRAM_PORT_BE #(Bit #(addr_), Bit #(32), 4)) a_data_ports = newVector;
   for (Integer i = 0; i < 4; i = i+1) begin
      a_data_ports[i] = data_ports[i].a;
   end
   Vector #(1, BRAM_PORT_BE #(Bit #(addr_), Bit #(8), 1)) a_tag_ports = newVector;
   for (Integer i = 0; i < 1; i = i+1) begin
      a_tag_ports[i] = tag_ports[i].a;
   end

   Vector #(4, BRAM_PORT_BE #(Bit #(addr_), Bit #(32), 4)) b_data_ports = newVector;
   for (Integer i = 0; i < 4; i = i+1) begin
      b_data_ports[i] = data_ports[i].b;
   end
   Vector #(1, BRAM_PORT_BE #(Bit #(addr_), Bit #(8), 1)) b_tag_ports = newVector;
   for (Integer i = 0; i < 1; i = i+1) begin
      b_tag_ports[i] = tag_ports[i].b;
   end

   let a_merged <- mkMergeBRAMs (a_data_ports, a_tag_ports);
   let b_merged <- mkMergeBRAMs (b_data_ports, b_tag_ports);

   interface a = a_merged;
   interface b = b_merged;
endmodule

// Merge vectors of BRAMs
// BSC claims that Vivado can't infer BRAMs which are more than 32 bits wide,
// so this is used to merge several smaller BRAMs into one larger BRAM
// So far this has only been tested with 4x32b data and 1x8b tag BRAMs
// TODO Currently these modules don't know how to handle having more than one
// tag in each word of the tag BRAMs
module mkMergeBRAMs #( Vector #(n_data_, BRAM_PORT_BE #(Bit #(addr_), Bit #(data_), TDiv #(data_, SizeOf #(Byte)))) data_ports
                     , Vector #(n_tags_, BRAM_PORT_BE #(Bit #(addr_), Bit #(tags_), TDiv #(tags_, SizeOf #(Byte)))) tag_ports)
                     (BRAM_PORT_BE #(Bit #(addr_), Bit #(total_data_), total_be_))
                     provisos ( NumAlias #(total_data_, TAdd #(TMul #(data_, n_data_), TMul #(tags_, n_tags_)))
                              , NumAlias #(total_be_,   TAdd #( TMul #(TDiv #(data_, SizeOf #(Byte)), n_data_)
                                                              , TMul #(TDiv #(tags_, SizeOf #(Byte)), n_tags_)))
                              , Add #(a__, data_, TAdd#(TMul#(data_, n_data_), TMul#(tags_, n_tags_)))
                              , Add #(b__, tags_, TAdd#(TMul#(data_, n_data_), TMul#(tags_, n_tags_))));
   Reg #(Bool) drg_req <- mkDReg (False);
   rule rl_debug (drg_req);
      $display ("%m mkMergeBRAMs -- read data:");
      for (Integer i = 0; i < valueOf (n_tags_); i = i+1) begin
         $display ("    tag_ports[", i, "].read: ", fshow (tag_ports[i].read));
      end
      for (Integer i = 0; i < valueOf (n_data_); i = i+1) begin
         $display ("    data_ports[", i, "].read: ", fshow (data_ports[i].read));
      end
   endrule
   method Action put (Bit #(total_be_) writeen, Bit #(addr_) addr, Bit #(total_data_) data);
      if (writeen == 0) begin
         drg_req <= True;
      end
      for (Integer i = 0; i < valueOf (n_data_); i = i+1) begin
         data_ports[i].put ( writeen[ valueOf (TDiv #(data_, SizeOf #(Byte)))*(i+1) - 1
                                    : valueOf (TDiv #(data_, SizeOf #(Byte)))*i]
                           , addr
                           , data[ valueOf (data_)*(i+1) - 1
                                 : valueOf (data_)*i]
                           );
      end
      for (Integer i = 0; i < valueOf (n_tags_); i = i+1) begin
         tag_ports[i].put ( writeen[ valueOf (TMul #(n_data_, TDiv #(data_, SizeOf #(Byte)))) + valueOf (TDiv #(tags_, SizeOf #(Byte)))*(i+1) - 1
                                   : valueOf (TMul #(n_data_, TDiv #(data_, SizeOf #(Byte)))) + valueOf (TDiv #(tags_, SizeOf #(Byte)))*i]
                          , addr
                          , data[ valueOf (TMul #(n_data_, data_)) + valueOf (SizeOf #(Byte))*(i+1) - 1
                                : valueOf (TMul #(n_data_, data_)) + valueOf (SizeOf #(Byte))*i]
                          );
      end
   endmethod

   method Bit #(total_data_) read;
      Bit #(total_data_) res = ?;
      for (Integer i = 0; i < valueOf (n_tags_); i = i+1) begin
         res[valueOf (n_data_)*32 + i*8 + 7 : valueOf (n_data_)*32 + i*8] = tag_ports[i].read;
      end
      for (Integer i = 0; i < valueOf (n_data_); i = i+1) begin
         res[i*32 + 31 : i*32] = data_ports[i].read;
      end
      return res;
   endmethod
endmodule

// This uses the BRAM port and provides an ITCM_IFC interface
// It is assumed that the data_ size is at least the size of instructions
// In order to accommodate capability tags within the BRAM, each entry in
// the BRAM contains extra bits to hold these tags.
// This module assumes that the capability tags are held in the top bits of the
// BRAM
// The module uses the size of a capability in memory to calculate the number
// of tags
// This module assumes that accesses don't straddle BRAM entries - that is, all
// accesses are at least Instr-aligned.
// This module assumes that accesses are always Instr-wide (although this is
// not not known to have an impact on its behaviour).

// It is assumed that the capability tags will be in the most significant bits
// of each entry, and that there will be enough words used for capability
// tags to allow for one tag per capability-sized chunk of bits in the BRAM.
// Within the Most Significant Bits used for capabilities, the tags are assumed
// to be in the lower bits.
// For example, if capabilities are 128 bits and there are 4 capabilities per
// BRAM entry then it is assumed that the BRAM
// width will be 4*128 + 4, rounded up to the nearest 8 bits, giving a size of
// 520 bits. Bits [511:0] are assumed to be capabilities or data, and bits
// [515:512] are assumed to be the capability bits, with bit 512 being the tag
// for the capability in bits [127:0] of the entry.
// For 128-bit capabilities with 1 capability in each entry, this means each
// entry must be 136 bits with the capability being held in bits [127:0] and
// the tag being held in bit 128.
// This has only been tested with one capability per TCM entry
module mkITCM_from_BRAM #( BRAM_PORT_BE #(Bit #(addr_), Bit #(data_), be_) port
                         , Bit #(2) verbosity
                         )
                         (ITCM_IFC)
                         provisos ( Add #(a__, SizeOf #(Instr), data_)
                                  // the maximum number of full capabilities that can fit in each BRAM entry
                                  // calculate the floor of the result of dividing data_ by the size of CapMem
                                  // Done this way because TDiv gives the ceiling of the result of division rather
                                  // than the floor
                                  // TODO if a function exists to do floor division, use it here
                                  , NumAlias #(caps_in_data_floor_, TDiv #( TSub #(data_, TSub #(SizeOf #(CapMem), 2))
                                                                          , SizeOf #(CapMem)))
                                  // the number of bits of data in each bram entry
                                  // (ie does not include the bits used for tags in the top of each entry
                                  , NumAlias #(data_only_, TMul #( TSub #(SizeOf #(CapMem), 1)
                                                                 , caps_in_data_floor_))
                                  , Add #(b__, data_only_, data_)
                                  , Add #(c__, SizeOf #(Instr), data_only_)
                                  // num_instr_in_word's size has to be at most the size of WordXL
                                  //     meaning our words can be at most 2^WordXL bytes long
                                  //, Add #(c__, TLog #(TDiv #(data_only_, SizeOf #(Instr))), SizeOf #(WordXL))
                                  , Add #(d__, TLog#(data_only_), SizeOf #(Addr))
                                  );
   Reg #(Bool) rg_first_req <- mkReg (False);
   Wire #(Bool) dw_commit <- mkDWire (False);
   Wire #(Bool) dw_req <- mkDWire (False);

   Wire #(Bit #(3))  w_f3          <- mkWire;
   // TODO might want to give the pc a default value, so we can always make
   // requests from the BRAM even when we have not received a request
   Wire #(WordXL)    w_pc          <- mkWire;
   Reg #(WordXL)     rg_pc         <- mkRegU;
`ifdef ISA_PRIV_S
   Wire #(Priv_Mode) w_priv        <- mkWire;
   Wire #(Bit #(1))  w_sstatus_SUM <- mkWire;
   Wire #(Bit #(1))  w_mstatus_MXR <- mkWire;
   Wire #(WordXL)    w_satp        <- mkWire;
`endif
`ifdef RVFI_DII
   // If we are compiling with RVFI_DII, then we should never be fetching
   // instructions from this module
   Reg #(Dii_Id)     rg_seq_req     <- mkWire;
`endif

   Reg #(Bit #(data_)) crg_tcm_out[2] <- mkCRegU (2);

   // Use the lower bits of the address to decide which chunk to extract from the TCM word
   // Assumes that the address is instruction-aligned
   function Instr fv_extract_instr_bits (WordXL addr, Bit #(data_only_) tcm_word);
      // left-shift the address (which is a byte address) by 3 to make it a bit address,
      // then truncate that and use the lower bits as an index into the BRAM word
      Bit #(TLog #(data_only_)) low_bit = truncate (addr << valueOf (TLog #(SizeOf #(Byte))));
      Instr res = tcm_word[low_bit + fromInteger (valueOf (SizeOf #(Instr))) - 1:low_bit];
      return res;
   endfunction

   // The IMem supports only read requests
   rule rl_send_req_to_port (dw_req);
      Bit #(addr_) local_addr = fv_cpu_addr_to_local_addr (w_pc, valueOf (data_only_));
      port.put (0, local_addr, ?);
      if (verbosity > 0) begin
         $display ("raw addr: ", fshow (w_pc));
         $display ("local addr: ", fshow (local_addr));
      end
   endrule

   // this assumes that dw_commit will be set to true exactly one cycle
   // after a request that the CPU deems to be "committed" (ie should actually
   // go through and not be cancelled
   // TODO the CPU currently holds commit true all the time; not sure what
   // happens when we call read after not calling a put the cycle before
   //    On Xilinx, it seems like it keeps the last read value (so this logic
   //    is probably unnecessary)
   //    Not sure what happens on altera
   rule rl_update_creg (rg_first_req && dw_commit);
      crg_tcm_out[0] <= port.read;
      if (verbosity > 0) begin
         $display ("IMem TCM read value: ", fshow (port.read));
         $display ("    extracted: ", fshow (fv_extract_instr_bits (rg_pc, truncate (port.read))));
         $display ("    registered addr: ", fshow (rg_pc));
      end
   endrule


   method Action reset;
      // There is nothing to reset
      noAction;
   endmethod

   interface IMem_IFC imem;
      method Action  req ( Bit #(3) f3
                         , WordXL addr
`ifdef ISA_PRIV_S
                         // The following  args for VM
                         , Priv_Mode  priv
                         , Bit #(1)   sstatus_SUM
                         , Bit #(1)   mstatus_MXR
                         , WordXL     satp
`endif
`ifdef RVFI_DII
                         , Dii_Id seq_req
`endif
                         );
         w_f3 <= f3;
         w_pc <= addr;
         rg_pc <= addr;
`ifdef ISA_PRIV_S
         w_priv <= priv;
         w_sstatus_SUM <= sstatus_SUM;
         w_mstatus_MXR <= mstatus_MXR;
         w_satp <= satp;
`endif
`ifdef RVFI_DII
         rg_seq_req <= seq_req;
`endif
         dw_req <= True;
         rg_first_req <= True;
         if (verbosity > 0) begin
            $display ("TCM IMem request for addr: ", fshow (addr));
         end
         if (verbosity > 1) begin
            $display ("    f3: ", fshow (f3));
`ifdef ISA_PRIV_S
            $display ("    priv: ", fshow (priv));
            $display ("    sstatus_SUM: ", fshow (sstatus_SUM));
            $display ("    mstatus_MXR: ", fshow (mstatus_MXR));
            $display ("    satp: ", fshow (satp));
`endif
`ifdef RVFI_DII
            $display ("    seq_req: ", fshow (seq_req));
`endif
         end
      endmethod

`ifdef ISA_CHERI
      // intended behaviour: the memory should wait for a commit in order to confirm
      // whether it should actually perform an access
      // Since the IMem port is only used for reading, we don't care whether
      // the read gets committed or not because reading BRAM has no side-effects
      // so we do it anyway and report valid if the access was committed
      method Action commit;
         dw_commit <= True;
      endmethod
`endif

      // CPU side: IMem response
      method Bool     valid = dw_commit;
      // TODO this might not be supposed to be always true?
      method Bool     is_i32_not_i16 = True;
      method WordXL   pc = rg_pc;
`ifdef RVFI_DII
      // If we are compiling with RVFI_DII, then we should never be fetching
      // instructions from this module
      method Tuple2#(Instr, Dii_Id) instr
         = tuple2 (fv_extract_instr_bits (rg_pc, truncate (crg_tcm_out[1])), 0);
`else
      method Instr    instr
         = fv_extract_instr_bits (rg_pc, truncate (crg_tcm_out[1]));
`endif

      // TODO for now, this throws no exceptions
      method Bool     exc      = False;
      method Exc_Code exc_code = ?;
      method WordXL   tval     = ?;

`ifdef PERFORMANCE_MONITORING
      // This is a BRAM, and the only requests that end up here will be fulfilled
      // here the next cycle, so there is no stalling
      method EventsCache events;
         EventsCache evts = EventsCache { evt_LD           : dw_req
                                        , evt_LD_MISS      : False
                                        , evt_LD_MISS_LAT  : False
                                        , evt_ST           : False
                                        , evt_ST_MISS      : False
                                        , evt_ST_MISS_LAT  : False
                                        , evt_AMO          : False
                                        , evt_AMO_MISS     : False
                                        , evt_AMO_MISS_LAT : False
                                        , evt_TLB          : False
                                        , evt_TLB_MISS     : False
                                        , evt_TLB_MISS_LAT : False
                                        , evt_TLB_FLUSH    : False
                                        , evt_EVICT        : False
                                        };
         return evts;
      endmethod
`endif
   endinterface
endmodule




// Create a DTCM with an AXI interface
// Requests on the CPU interface take precendence over requests on the AXI
// interface.
// This module provides no backpressure to the CPU (the CPU is responsible for
// ensuring that it only makes one outstanding request at a time) but does
// provide backpressure on the AXI slave port
// This module assumes that accesses do not straddle BRAM entries - that is,
// all accesses must be aligned to their size (word accesses must be word
// aligned, halfword accesses must be halfword aligned).

// Similarly to the ITCM, this assumes that the capability tags are in the
// upper bits, and calculates how many bits are used for capability using
// the size of a CapMem and the size of the BRAM.

// This module uses a store buffer in order to handle cases where we get a
// store immediately followed by a load
//    On the cycle after we receive the store request, we need to wait for a
//    commit to check whether we actually should perform the store. However,
//    on the same cycle we might receive another memory request. In this case
//    we would need to perform 2 accesses in the same cycle, which we can't do
// In order to handle this, we use the store buffer to hold the last store
// request. The request stays in the buffer until we get a cycle where there is
// not a load. This is because we want to service loads as fast as possible.
// When we encounter a cycle with no memory access or with another load, we can
// perform the store in the buffer. If there is also a new store incoming, we
// replace the contents of the store buffer with the new incoming store.

// TODO this currently cannot handle having more than 1 capability in each TCM
// entry
module mkDTCM_AXI_from_BRAM #( BRAM_PORT_BE #(Bit #(b_addr_), Bit #(b_data_), b_be_) port
                             , Bit #(2) verbosity
                             )
                             (DTCM_AXI_IFC #( sid_, addr_, data_
                                            , awuser, wuser, buser
                                            , aruser, ruser))
                             provisos ( Add #(a__, TSub #(SizeOf #(CapMem), 1), b_data_) // TODO this assumes we have capabilities in here
                                        // TODO comment this (should be similar to ITCM above)
                                        // calculate the number of capabilities + tags in each entry
                                      , NumAlias #(caps_in_data_floor_, TDiv #( TSub #(b_data_, TSub #(SizeOf #(CapMem), 2))
                                                                              , SizeOf #(CapMem)))
                                      , NumAlias #(b_data_only_, TMul #( TSub #(SizeOf #(CapMem), 1)
                                                                       , caps_in_data_floor_))
                                      , Add #(b__, b_data_only_, b_data_)
                                      , Add #(c__, TSub #(SizeOf #(CapMem), 1), b_data_only_)
                                      , Add #(d__, TLog#(b_data_only_), SizeOf #(Addr))
                                      , Add #(e__, TLog#(caps_in_data_floor_), SizeOf #(WordXL))
                                      , Add #(f__, TLog#(b_data_), SizeOf #(WordXL))
                                      , Add #(g__, TLog#(b_be_), SizeOf #(WordXL))
                                      , Add #(h__, 128, b_data_only_)
                                      , Add #(i__, 64, b_data_only_)
                                      , Add #(j__, 32, b_data_only_)
                                      , Add #(k__, 16, b_data_only_)
                                      , Add #(l__, 8, b_data_only_)
                                      , Add #(m__, TDiv#(b_data_only_, SizeOf #(Byte)), b_be_)
                                      , Add #(n__, TLog#(TDiv#(b_data_only_, SizeOf #(Byte))), 64)
                                      , Add #(o__, TLog#(TSub#(b_data_, b_data_only_)), 64)
                                      , Add #(p__, TLog#(TSub#(b_data_, b_data_only_)), TLog#(b_data_))
                                      , Add #(q__, TLog#(caps_in_data_floor_), TLog#(b_data_))
                                      , Add #(r__, data_, b_data_only_)
                                      , Add #(s__, addr_, SizeOf #(WordXL))
                                      , Add #(t__, data_, 128)
                                      , Add #(u__, b_data_only_, 128)
                                      , Add #(v__, TLog#(data_), 64)
                                      , Add #(w__, TLog#(data_), addr_)
                                      , Mul #(x__, SizeOf #(Byte), b_data_)
                                      , Add #(y__, SizeOf #(Byte), TMul#(x__, SizeOf #(Byte)))

                                      // bsc required provisos
                                      //, Div#(TAdd#(TSub#(b_data_, SizeOf #(CapMem)), 1), SizeOf #(CapMem), 1)
                                      );
   // TODO move this into its own separate function?
   // Use the lower bits of the address and the f3 to decide which chunk
   // to extract from the TCM word
   // The extracted chunk is put in the LSB of the result
   // Assumes that the address is access-aligned
   function Bit #(b_data_only_) fv_extract_bits (WordXL addr, Bit #(3) f3, Bool is_unsigned, Bit #(b_data_only_) tcm_word);
      Bit #(TLog #(b_data_only_)) low_bit = truncate(addr) << valueOf (TLog #(SizeOf #(Byte)));
      Bit #(b_data_only_) res = ?;
      let u_s_extend = is_unsigned ? zeroExtend : signExtend;
      if (f3 == 3'b000) begin
         Bit #(TMul #(SizeOf #(Byte), 1)) val = truncate (tcm_word >> low_bit);
         res = u_s_extend (val);
      end else if (f3 == 3'b001) begin
         Bit #(TMul #(SizeOf #(Byte), 2)) val = truncate (tcm_word >> low_bit);
         res = u_s_extend (val);
      end else if (f3 == 3'b010) begin
         Bit #(TMul #(SizeOf #(Byte), 4)) val = truncate (tcm_word >> low_bit);
         res = u_s_extend (val);
      end else if (f3 == 3'b011) begin
         Bit #(TMul #(SizeOf #(Byte), 8)) val = truncate (tcm_word >> low_bit);
         res = u_s_extend (val);
      end else if (f3 == 3'b100) begin
         Bit #(TMul #(SizeOf #(Byte), 16)) val = truncate (tcm_word >> low_bit);
         res = u_s_extend (val);
      end
      return res;
   endfunction

`ifdef ISA_CHERI
   function Bool fv_extract_cap_tag (WordXL addr, Bit #(b_data_) tcm_word);
      Bit #(TLog #(caps_in_data_floor_)) cap_index = truncate (addr >> valueOf (TLog #(TSub #(SizeOf #(CapMem), 1))));
      Bit #(TSub #(b_data_, b_data_only_)) cap_tags = truncateLSB (tcm_word);
      Bool cap_valid = cap_tags[cap_index] == 1'b1;
      return cap_valid;
   endfunction
`endif

   function Tuple3 #(Bit #(b_be_), Bit #(b_addr_), Bit #(b_data_)) fv_mem_req_to_port_req (MMU_Cache_Req req);
      Bit #(TLog #(b_data_only_)) low_bit = truncate (req.va << valueOf (TLog #(SizeOf #(Byte))));
      Bit #(b_data_) data = zeroExtend (tpl_2 (req.st_value) & fv_size_code_to_mask (req.width_code)) << low_bit;
      // Which tag bit we are setting; only considers the bits dedicated for tags
      Bit #(TLog #(caps_in_data_floor_)) tag_bit = truncate (req.va >> valueOf (TLog #(TDiv #(TSub #(SizeOf #(CapMem), 1), SizeOf #(Byte)))));
      // Which bit inside the entire b_data_ element is the tag bit that we are setting
      Bit #(TLog #(b_data_)) tag_index = zeroExtend (tag_bit) + fromInteger (valueOf (b_data_only_));
      data[tag_index] = tpl_1(req.st_value) ? 1'b1 : 1'b0;
      Bit #(b_addr_) addr = fv_cpu_addr_to_local_addr (req.va, valueOf (b_data_only_));
      // Use the data_only size to find the number of byte-enables used for data only
      Bit #(TDiv #(b_data_only_, SizeOf #(Byte))) be_data = fv_req_to_byte_mask (req);
      Bit #(b_be_) be = zeroExtend (be_data);
      be[fromInteger (valueOf (b_be_)) - 1] = req.op == CACHE_ST ? 1'b1 : 1'b0;
      return tuple3 (be, addr, data);
   endfunction


   Wire #(Bool)          dw_req_start <- mkDWire (False);
   Wire #(Bool)          dw_commit <- mkDWire (False);
   Wire #(Bool)          dw_read <- mkDWire (False);
   Wire #(MMU_Cache_Req) dw_req <- mkDWire (?);
   Reg #(MMU_Cache_Req) rg_req <- mkRegU;
   Reg #(Bool) crg_valid[2]       <- mkCReg (2, False);
   Reg #(Bool) drg_pending <- mkDReg (False);
   Reg #(Bool) rg_pending <- mkReg (False);
   Reg #(Bool) drg_read <- mkDReg (False);
   Reg #(Bool) rg_started <- mkReg (False);
   Wire #(Bit #(b_data_)) dw_raw_output <- mkDWire (?);

   Reg #(Bit #(b_data_)) crg_tcm_out[2] <- mkCRegU (2);
   Reg #(Bool) rg_exc <- mkRegU;
   Reg #(Exc_Code) rg_exc_code <- mkRegU;

   Wire #(Bool)          dw_valid <- mkDWire (False);
   Wire #(Bool) dw_exc <- mkDWire (False);
   Wire #(Exc_Code) dw_exc_code <- mkDWire (exc_code_LOAD_ADDR_MISALIGNED);

   let axi_shim <- mkAXI4ShimFF;
   let ug_axi_master <- toUnguarded_AXI4_Master (axi_shim.master);
   Reg #(Bool) drg_axi_rsp_w <- mkDReg (False);
   Reg #(Bool) drg_axi_rsp_r <- mkDReg (False);
   Reg #(Bit #(sid_)) rg_axi_id <- mkRegU;

   // We have a single-element store buffer to allow stores to take one cycle
   // while still using the commit signal
   Reg #(Bool) crg_store_pending[2] <- mkCReg (2, False);
   Reg #(Bool) crg_store_accepted[2] <- mkCReg (2, False);
   Reg #(WordXL) rg_store_addr <- mkRegU;
   Reg #(Bit #(3)) rg_store_width_code <- mkRegU;
   Reg #(Tuple2 #(Bool, Bit #(128))) rg_store_val <- mkRegU;
   Reg #(Bool) drg_read_from_buffer <- mkDReg (False);

`ifdef WATCH_TOHOST
   Reg #(Bool)      rg_watch_tohost  <- mkReg (False);
   Reg #(Bit #(64)) rg_tohost_addr   <- mkReg ('h1000);
   Reg #(Bit #(64)) rg_tohost_value  <- mkReg (0);
`endif

`ifdef PERFORMANCE_MONITORING
   Array #(Reg #(EventsCache)) aw_events <- mkDRegOR (7, unpack (0));
`endif

`ifdef ISA_A
   // When a LR is made, set rg_amo_valid to True and set addr to the address
   // When a non-SC store is made to the same address, amo_valid is set to false
   //   and store value is stored
   // When a SC writes and rg_amo_valid is false, it fails
   // When a SC writes, amo_valid is true but the address does not match it fails
   // When a SC writes, amo_valid is true and the address matches it succeeds
   // The value returned is 1 if failed, or 0 if succeeded
   // amo_valid is set to False afterwards
   // On other AMO requests: just atomically change word in memory without
   // CPU involvement. This is returned to the CPU but not used
   Reg #(Bool) rg_lrsc_valid <- mkReg (False);
   Reg #(Bool) rg_lrsc_success <- mkReg (True);
   Reg #(WordXL) rg_lrsc_addr <- mkRegU;
`endif

`ifdef ISA_A
   let rg_is_AMO_LR = (rg_req.op == CACHE_AMO) && (rg_req.amo_funct5 == f5_AMO_LR);
   let rg_is_AMO_SC = (rg_req.op == CACHE_AMO) && (rg_req.amo_funct5 == f5_AMO_SC);
`else
   let rg_is_AMO_LR = False;
   let rg_is_AMO_SC = False;
`endif

   Wire #(Bool) dw_store_pending <- mkDWire (False);
   rule rl_decouple;
      dw_store_pending <= crg_store_pending[0];
   endrule



   // Register a request from the AXI shim only if the CPU is not making a
   // request and the module is not handling a CPU request
   //    We know the CPU is not handling a request if rg_pending is False,
   //    because the only requests that take multiple cycles to handle are
   //    CPU write requests, which set rg_pending to True
   // The requests coming in on this bus are 1-flit requests
   rule rl_handle_req_from_axi ((ug_axi_master.ar.canPeek
                                 || (ug_axi_master.aw.canPeek
                                     && ug_axi_master.w.canPeek))
                                && !dw_req_start
                                && !rg_pending
                                && !drg_pending
                                && !crg_store_pending[0]
                                && !dw_store_pending
                                && ug_axi_master.b.canPut
                                && ug_axi_master.r.canPut);
      if (ug_axi_master.ar.canPeek) begin
         let arreq = ug_axi_master.ar.peek;
         // read request
         Bit #(b_addr_) local_addr = fv_cpu_addr_to_local_addr (zeroExtend (arreq.araddr), valueOf (b_data_only_));
         let min_req = MMU_Cache_Req { op: CACHE_ST
                                     , width_code: pack (arreq.arsize)
                                     , va: zeroExtend (arreq.araddr)
                                     };
         port.put (0, local_addr, ?);
         drg_axi_rsp_r <= True;
         rg_axi_id <= arreq.arid;
         rg_req <= min_req;
         ug_axi_master.ar.drop;

         if (verbosity > 0) begin
            $display ("%m rl_handle_req_from_axi ar request");
            $display ("    arreq: ", fshow (arreq));
            $display ("    local addr: ", fshow (local_addr));
         end
      end else begin
         // write request
         let awreq = ug_axi_master.aw.peek;
         let wreq = ug_axi_master.w.peek;
         // TODO this might be a bit wasteful; we right shift and then
         // left shift again later
         Bit #(TLog #(data_)) data_rshift_amt = truncate (awreq.awaddr << 3);
         let min_req = MMU_Cache_Req { op: CACHE_ST
                                     , width_code: pack (awreq.awsize)
                                     , va: zeroExtend (awreq.awaddr)
                                     , st_value: tuple2(False, zeroExtend (wreq.wdata >> data_rshift_amt))
                                     };
         Bit #(b_addr_) local_addr = fv_cpu_addr_to_local_addr (zeroExtend (awreq.awaddr), valueOf (b_data_only_));
         let write_tuple = fv_mem_req_to_port_req (min_req);
         port.put (tpl_1 (write_tuple), tpl_2 (write_tuple), tpl_3 (write_tuple));
         drg_axi_rsp_w <= True;
         rg_axi_id <= awreq.awid;
         rg_req <= min_req;
         ug_axi_master.aw.drop;
         ug_axi_master.w.drop;

         if (verbosity > 0) begin
            $display ("%m rl_handle_req_from_axi aw request");
            $display ("    awreq: ", fshow (awreq));
            $display ("    wreq: ", fshow (wreq));
            $display ("    local addr: ", fshow (local_addr));
            $display ("    write tuple: ", fshow (write_tuple));
         end
      end
   endrule

   rule rl_handle_w_rsp_to_axi (drg_axi_rsp_w);
      // TODO what to do if the response is not OKAY?
      ug_axi_master.b.put (AXI4_BFlit { bid: rg_axi_id
                                      , bresp: OKAY
                                      , buser: 0});
   endrule


   // This might be a bit wasteful - we're right-shifting and then left shifting again
   rule rl_handle_r_rsp_to_axi (drg_axi_rsp_r);
      let raw_data = port.read;
      Bit #(TLog #(data_)) data_lshift_amt = truncate (rg_req.va << 3);
      let data = fv_extract_bits (rg_req.va, rg_req.width_code, False, truncate (raw_data));
      ug_axi_master.r.put (AXI4_RFlit { rid: rg_axi_id
                                      , rdata: truncate (data << data_lshift_amt)
                                      , rresp: OKAY
                                      , rlast: True
                                      , ruser: 0});
   endrule


   // Register the request from the CPU
   // If it is a load, also request the data from the BRAM
   (* conflict_free = "rl_register_from_cpu, rl_handle_pending_store" *)
   (* execution_order = "rl_handle_pending_store, rl_register_from_cpu" *)
   rule rl_register_from_cpu (dw_req_start);
      if (verbosity > 0) begin
         $display ("%m: rl_register_from_cpu");
      end
      rg_req <= dw_req;
      rg_started <= True;
      let pending = False;
      let is_AMO_LR = (dw_req.op == CACHE_AMO) && (dw_req.amo_funct5 == f5_AMO_LR);
      let is_AMO_SC = (dw_req.op == CACHE_AMO) && (dw_req.amo_funct5 == f5_AMO_SC);
      Bit #(b_addr_) local_addr = fv_cpu_addr_to_local_addr (dw_req.va, valueOf (b_data_only_));
      if (! fn_is_aligned (dw_req.width_code, dw_req.va)) begin
         rg_exc <= True;
         rg_exc_code <= dw_req.op == CACHE_LD ? exc_code_LOAD_ADDR_MISALIGNED
                                             : exc_code_STORE_AMO_ADDR_MISALIGNED;
         if (verbosity > 0) begin
            $display ("    requested misaligned access");
            $display ("    req: ", fshow (dw_req));
         end
      end else begin
         rg_exc <= False;
         if (dw_req.op == CACHE_LD || is_AMO_LR || (dw_req.op == CACHE_AMO && !is_AMO_SC)) begin
`ifdef ISA_A
            if (is_AMO_LR) begin
               rg_lrsc_valid <= True;
               rg_lrsc_addr <= dw_req.va;
               if (verbosity > 0) begin
                  $display ("    LR reserving addr: ", fshow (dw_req.va));
               end
            end
            if (dw_req.op == CACHE_AMO && !is_AMO_SC && !is_AMO_LR) begin
               // TODO should i invalidate when an AMO is performed on the same address
               // as the LRSC?
               pending = True;
               rg_pending <= True;
               rg_lrsc_valid <= False;
               if (verbosity > 0) begin
                  $display ("    ISA_A invalidating addr: ", fshow (dw_req.va), " due to AMO request");
               end
            end
`endif
            drg_read <= True;

            Bit #(b_addr_) store_local_addr = fv_cpu_addr_to_local_addr (rg_store_addr, valueOf (b_data_only_));
            if (crg_store_pending[0] && local_addr == store_local_addr) begin
               if (verbosity > 0) begin
                  $display ("    address matches buffer; forwarding from buffer");
               end
               drg_read_from_buffer <= True;
            end else begin
               if (verbosity > 0) begin
                  $display ("    address does not match buffer; not forwarding");
               end
            end
            // Still need to make the read request from BRAM because we might have written
            // less than we're reading now
            port.put (0, local_addr, ?);

            if (verbosity > 0) begin
               $display ("    requesting data from BRAM");
               $display ("    cpu addr: ", fshow (dw_req.va));
               $display ("    req: ", fshow (dw_req));
               $display ("    local addr: ", fshow (local_addr));
`ifdef ISA_A
               $display ("    is other AMO: ", fshow (dw_req.op == CACHE_AMO && !is_AMO_SC && !is_AMO_LR));
`endif
            end
         end else if (dw_req.op == CACHE_ST || is_AMO_SC) begin
            let do_write = True;
`ifdef ISA_A
            let lrsc_valid_new = rg_lrsc_valid;
            if (rg_lrsc_valid && dw_req.op == CACHE_ST && dw_req.va == rg_lrsc_addr) begin
               lrsc_valid_new = False;
               do_write = True;
               if (verbosity > 0) begin
                  $display ("    cancelling LRSC reservation, reserved addr: ", fshow (rg_lrsc_addr),
                            "  requested addr", fshow (dw_req.va));
               end
            end else if (is_AMO_SC) begin
               lrsc_valid_new = False;
               if (!rg_lrsc_valid) begin
                  do_write = False;
                  if (verbosity > 0) begin
                     $display ("    no valid LRSC reservation, requested addr: ", fshow (dw_req.va));
                  end
               end else if (dw_req.va != rg_lrsc_addr) begin
                  do_write = False;
                  if (verbosity > 0) begin
                     $display ("    SC addr does not match reservation. Reserved: ", fshow (rg_lrsc_addr),
                               "  requested: ", fshow (dw_req.va));
                  end
               end

               rg_lrsc_success <= do_write;
            end
            rg_lrsc_valid <= lrsc_valid_new;
`endif
            if (verbosity > 0) begin
               $display ("    requesting write to BRAM");
               $display ("    cpu addr: ", fshow (dw_req.va));
               $display ("    req: ", fshow (dw_req));
               $display ("    local addr: ", fshow (local_addr));
            end
            // We can now put the store being requested now in the buffer, since the
            // store that is currently in the buffer can execute now
            crg_store_pending[1] <= do_write;
            rg_store_addr <= dw_req.va;
            rg_store_val <= dw_req.st_value;
            rg_store_width_code <= dw_req.width_code;
            if (verbosity > 0) begin
               $display ("    adding new store to buffer");
               $display ("    do_write: ", fshow (do_write));
               $display ("    addr: ", fshow (dw_req.va));
               $display ("    st_value: ", fshow (dw_req.st_value));
               $display ("    width_code: ", fshow (dw_req.width_code));
            end

            pending = True;
         end
      end
      drg_pending <= pending;
   endrule

   // Perform the write currently held in rg_req
   (* conflict_free = "rl_check_commit_from_cpu, rl_register_read" *)
   (* conflict_free = "rl_check_commit_from_cpu, rl_drive_rsp" *)
   (* conflict_free = "rl_check_commit_from_cpu, rl_handle_pending_store" *)
   (* conflict_free = "rl_register_from_cpu, rl_check_commit_from_cpu" *)
   (* execution_order = "rl_register_read, rl_check_commit_from_cpu" *)
   rule rl_check_commit_from_cpu (drg_pending);
      $display ("%m: rl_check_commit_from_cpu");
      Bit #(b_addr_) local_addr = fv_cpu_addr_to_local_addr (rg_req.va, valueOf (b_data_only_));
      if ((rg_req.op == CACHE_ST || rg_is_AMO_SC) && dw_commit) begin
         //let write_tuple = fv_mem_req_to_port_req (rg_req);
         //Bit #(TLog #(b_data_only_)) low_bit = truncate (rg_req.va << valueOf (TLog #(SizeOf #(Byte))));
         //Bit #(b_data_) data = zeroExtend (tpl_2 (rg_req.st_value) & fv_size_code_to_mask (rg_req.width_code)) << low_bit;
         //Bit #(TLog #(caps_in_data_floor_)) tag_bit = truncate (rg_req.va >> valueOf (TLog #(TDiv #(TSub #(SizeOf #(CapMem), 1), SizeOf #(Byte)))));
         //Bit #(TLog #(b_data_)) tag_index = zeroExtend (tag_bit) + fromInteger (valueOf (b_data_only_));
         //if (verbosity > 0) begin
         //   $display ("    requesting write to BRAM");
         //   $display ("    raw req: ", fshow (rg_req));
         //   $display ("    local addr: ", fshow (local_addr));
         //   $display ("    raw write tuple: ", fshow (write_tuple));
         //   $display ("    low_bit: ", fshow (low_bit));
         //   $display ("    data: ", fshow (data));
         //   $display ("    tag_index: ", fshow (tag_index));
         //end
         if (rg_req.op == CACHE_ST
`ifdef ISA_A
             || (rg_is_AMO_SC && rg_lrsc_success)
`endif
             ) begin
            crg_store_accepted[0] <= True;
            if (verbosity > 0) begin
               $display ("    accepting write");
               $display ("    dw_store_pending: ", fshow (dw_store_pending));
               $display ("    rg_store_val: ", fshow (rg_store_val));
               $display ("    rg_store_addr: ", fshow (rg_store_addr));
               $display ("    rg_store_width_code : ", fshow (rg_store_width_code));
            end
         end
`ifdef ISA_A
         if (rg_is_AMO_SC) begin
            Bit #(TLog #(b_data_only_)) lshift_amt = truncate (rg_req.va << 3);
            crg_tcm_out[0] <= (rg_lrsc_success ? 0 : 1) << lshift_amt;
         end
`endif
`ifdef WATCH_TOHOST
         if (rg_req.va == rg_tohost_addr && rg_watch_tohost) begin
            rg_tohost_value <= truncate (tpl_2(rg_req.st_value));
            $display (" TOHOST TRIGGER ");
         end
`endif
      end else if (rg_req.op == CACHE_LD || rg_is_AMO_LR) begin
         let raw_data = port.read;
         let data_read = fv_extract_bits (rg_req.va, rg_req.width_code, rg_req.is_unsigned, truncate (raw_data));
         //let data_read = 0;
`ifdef ISA_CHERI
         let cap_tag = fv_extract_cap_tag (rg_req.va, raw_data);
`endif
`ifdef ISA_A
         if (!dw_commit) begin
            // TODO do we care if a LR is not committed?
            //    is it even possible?
         end
`endif

         if (verbosity > 0) begin
            $display ("    committing read");
            $display ("    cpu addr: ", fshow (rg_req.va));
            $display ("    local addr: ", fshow (local_addr));
            $display ("    raw data: ", fshow (raw_data));
            $display ("    data: ", fshow (data_read));
`ifdef ISA_CHERI
            $display ("    tag: ", fshow (cap_tag));
`endif
         end
         if ((valueOf (SizeOf #(CapMem)) == (64+1) && rg_req.width_code > w_SIZE_D)
             || (valueOf (SizeOf #(CapMem)) == (128+1) && rg_req.width_code > w_SIZE_Q)) begin
            $display ("    ERROR");
            $display ("    CPU requested read that was too big");
            $display ("    requested f3: ", fshow (rg_req.width_code));
         end
      end
`ifdef ISA_A
      else begin
         // handle all other AMO requests
         if (dw_commit) begin
            Tuple2 #(Bool, Bit #(128)) ld_val_cap = ?;
            Bit #(b_addr_) store_local_addr = fv_cpu_addr_to_local_addr (rg_store_addr, valueOf (b_data_only_));
            if (dw_store_pending && local_addr == store_local_addr) begin
               ld_val_cap = rg_store_val;
            end else begin
               let raw_data = port.read;
               ld_val_cap =
                  tuple2 ( fv_extract_cap_tag (rg_req.va, raw_data)
                         , truncate (fv_extract_bits (rg_req.va, rg_req.width_code, rg_req.is_unsigned, truncate (raw_data))));
            end

            if (dw_store_pending) begin
               // If there is already a pending store, we need to dispatch it
               // typically this would be done by rl_handle_pending_store but that
               // is not possible because this rule needs to happen both before and
               // after rl_handle_pending_store
               let tmp_req_old = MMU_Cache_Req { op: CACHE_ST
                                               , va: rg_store_addr
                                               , st_value: rg_store_val
                                               , width_code: rg_store_width_code};
               let write_tuple_old = fv_mem_req_to_port_req (tmp_req_old);
               port.put (tpl_1 (write_tuple_old), tpl_2 (write_tuple_old), tpl_3 (write_tuple_old));
               if (verbosity > 0) begin
                  $display ("    performed pending store");
                  $display ("    write_tuple_old: ", fshow (write_tuple_old));
               end
            end

            match { .new_ld_val
                  , .new_st_val} = fv_amo_op ( rg_req.width_code
                                             , rg_req.amo_funct5
                                             , ld_val_cap
                                             , rg_req.st_value);
            let tmp_req = rg_req;
            tmp_req.st_value = new_st_val;
            let write_tuple = fv_mem_req_to_port_req (tmp_req);
            rg_pending <= False;
            crg_store_pending[1] <= True;
            crg_store_accepted[1] <= True;
            rg_store_addr <= rg_req.va;
            rg_store_width_code <= rg_req.width_code;
            rg_store_val <= new_st_val;

            if (verbosity > 0) begin
               //$display ("    committing other AMO request");
               $display ("    buffering other AMO request");
               $display ("    request: ", fshow (rg_req));
               $display ("    ld_val_cap: ", fshow (ld_val_cap));
               $display ("    new_ld_val: ", fshow (new_ld_val));
               $display ("    new_st_val: ", fshow (new_st_val));
               $display ("    write_tuple: ", fshow (write_tuple));
            end
         end
      end
`endif // ISA_A
   endrule

   rule rl_register_read (drg_read);
      let read_val = ?;
      let write_tuple = ?;
      Bit #(b_data_) buffer_mask = ?;
      if (drg_read_from_buffer) begin
         let tmp_req = MMU_Cache_Req { va: rg_store_addr
                                     , st_value: rg_store_val
                                     , width_code: rg_store_width_code
                                     , op: CACHE_ST
                                     };
         write_tuple = fv_mem_req_to_port_req (tmp_req);

         buffer_mask = fv_byte_mask_to_bit_mask (tpl_1 (write_tuple));
         let buffer_val = tpl_3 (write_tuple);
         let bram_mask = ~buffer_mask;
         let bram_val = port.read;

         read_val = (bram_val & bram_mask)
                  | (buffer_val & buffer_mask);
      end else begin
         read_val = port.read;
      end
      crg_tcm_out[0] <= read_val;
      if (verbosity > 0) begin
         $display ("%m: rl_register_read");
         $display ("    drg_read_from_buffer: ", fshow (drg_read_from_buffer));
         $display ("    bram data: ", fshow (port.read));
         $display ("    buffer mask: ", fshow (buffer_mask));
         $display ("    buffer data: ", fshow (rg_store_val));
         $display ("    buffer write_tuple: ", fshow (write_tuple));
         $display ("    chosen data: ", fshow (read_val));
      end
   endrule

   rule rl_drive_exc_rsp (rg_exc);
      if (verbosity > 0) begin
         $display ("%m: rl_drive_exc_rsp");
         $display ("    code: ", fshow (rg_exc_code));
      end
      dw_valid <= True;
      dw_exc <= True;
      dw_exc_code <= rg_exc_code;
   endrule

   rule rl_drive_rsp (rg_started && !rg_pending && !rg_exc);
      if (verbosity > 0) begin
         $display ("%m: rl_drive_rsp");
         $display ("    raw response: ", fshow (crg_tcm_out[1]));
         let raw_data = crg_tcm_out[1];
         let cap_tag = fv_extract_cap_tag (rg_req.va, raw_data);
         let data = fv_extract_bits (rg_req.va, rg_req.width_code, rg_req.is_unsigned, truncate(raw_data));
         $display ("    extracted response: ", fshow (data));
      end
      dw_valid <= True;
      dw_raw_output <= crg_tcm_out[1];
      dw_exc <= False;
   endrule

   rule rl_handle_pending_store (dw_store_pending
                                 && crg_store_accepted[1]
                                 && (!dw_req_start || dw_req.op == CACHE_ST)
                                 && !(drg_pending && rg_req.op == CACHE_AMO && !rg_is_AMO_LR && !rg_is_AMO_SC));
                                 // do the actual request
      if (verbosity > 0) begin
         $display ("%m: rl_handle_pending_store");
      end

      // If there is a pending store, we can perform it now because this store
      // needs to wait for its commit signal
      let tmp_req = MMU_Cache_Req { op: CACHE_ST
                                  , va: rg_store_addr
                                  , st_value: rg_store_val
                                  , width_code: rg_store_width_code};
      let write_tuple = fv_mem_req_to_port_req (tmp_req);
      port.put (tpl_1 (write_tuple), tpl_2 (write_tuple), tpl_3 (write_tuple));
      crg_store_pending[0] <= False;
      if (verbosity > 0) begin
         $display ("    performing pending store");
         $display ("    write_tuple: ", fshow (write_tuple));
      end
   endrule

`ifdef PERFORMANCE_MONITORING
   // evt_LD counts the number of loads
   // evt_ST counts the number of stores
   // evt_AMO counts the number of AMO operations
   // evt_AMO_LAT counts the number of cycles stalled due to AMO operations
   //    This happens on non-LRSC AMO operations
   Wire #(Bool) dw_evt_LD <- mkDWire (False);
   Wire #(Bool) dw_evt_ST <- mkDWire (False);
   Wire #(Bool) dw_evt_AMO <- mkDWire (False);
   Wire #(Bool) dw_evt_AMO_LAT <- mkDWire (False);
   rule rl_detect_reqs;
      dw_evt_LD <= rg_req.op == CACHE_LD;
      dw_evt_ST <= rg_req.op == CACHE_ST;
`ifdef ISA_A
      dw_evt_AMO <= rg_req.op == CACHE_AMO;
      // non-LRSC AMO requests always have exactly 1 cycle of latency
      dw_evt_AMO_LAT <= rg_req.op == CACHE_AMO && !rg_is_AMO_LR && !rg_is_AMO_SC;
`endif
   endrule
`endif // PERFORMANCE_MONITORING

   method Action reset;

   endmethod

   interface DMem_IFC dmem;
   // CPU side: DMem request
      method Action  req ( CacheOp op
                         , Bit #(3) f3
                         , Bool is_unsigned
`ifdef ISA_A
                         , Bit #(5) amo_funct5
`endif
                         , Addr addr
                         // TODO non-cheri, and 32-bit
                         , Tuple2#(Bool, Bit #(128)) store_value
`ifdef ISA_PRIV_S
                           // The following  args for VM
                         , Priv_Mode  priv
                         , Bit #(1)   sstatus_SUM
                         , Bit #(1)   mstatus_MXR
                         , WordXL     satp
`endif
                          );
         dw_req <= MMU_Cache_Req { op          : op
                                 , width_code  : f3
                                 , is_unsigned : is_unsigned
                                 , is_cap      : ?
`ifdef ISA_A
                                 , amo_funct5  : amo_funct5
`endif
                                 , va          : addr
                                 , st_value    : store_value
`ifdef ISA_PRIV_S
                                 , priv        : priv
                                 , sstatus_SUM : sstatus_SUM
                                 , mstatus_MXR : mstatus_MXR
                                 , satp        : satp
`endif
                                 };

         dw_req_start <= True;
         $display ("dmem request: ", fshow (addr));
      endmethod

      method Action commit;
         dw_commit <= True;
         $display ("dmem commit");
      endmethod

      // CPU side: DMem response
      method Bool       valid = dw_valid;
`ifdef ISA_CHERI
      method Tuple2#(Bool, Bit #(128))  word128;      // Load-value
         let raw_data = dw_raw_output;
         let cap_tag = fv_extract_cap_tag (rg_req.va, raw_data);
         let data = fv_extract_bits (rg_req.va, rg_req.width_code, rg_req.is_unsigned, truncate(raw_data));
         return tuple2 (cap_tag, truncate (data));
      endmethod
`else
      // TODO non-cheri word128
`endif

      // This is never used by the CPU
      method Bit #(128)  st_amo_val;  // Final store-value for ST, SC, AMO
         return ?;
      endmethod

      method Bool       exc      = dw_exc;
      method Exc_Code   exc_code = dw_exc_code;

`ifdef PERFORMANCE_MONITORING
      method EventsCache events;
         EventsCache evts = EventsCache { evt_LD           : dw_evt_LD
                                        , evt_LD_MISS      : False
                                        , evt_LD_MISS_LAT  : False
                                        , evt_ST           : dw_evt_ST
                                        , evt_ST_MISS      : False
                                        , evt_ST_MISS_LAT  : False
                                        , evt_AMO          : dw_evt_AMO
                                        , evt_AMO_MISS     : False
                                        , evt_AMO_MISS_LAT : dw_evt_AMO_LAT
                                        , evt_TLB          : False
                                        , evt_TLB_MISS     : False
                                        , evt_TLB_MISS_LAT : False
                                        , evt_TLB_FLUSH    : False
                                        , evt_EVICT        : False
                                        };
         return evts;
      endmethod
`endif
   endinterface

   // TODO implement this
   interface AXI4_Slave tcm_dma_server = axi_shim.slave;

`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
      rg_watch_tohost <= watch_tohost;
      rg_tohost_addr  <= tohost_addr;
      $display ("%m: set_watch_tohost: watch ", fshow (watch_tohost), "  addr: ", fshow (tohost_addr));
   endmethod

   method Bit #(64) mv_tohost_value = rg_tohost_value;
`endif
endmodule

module mkDummyBRAM (BRAM_PORT_BE #(Bit #(addr_), Bit #(data_), be_));
   method Action put (Bit #(be_) be, Bit #(addr_) addr, Bit #(data_) data);
      noAction;
   endmethod
   method read = 0;
endmodule

`ifdef RVFI_DII
// Use Mem_Model as the underlying memory primitive when building with RVFI_DII
// in order to make use of the reset implemented in Mem_Model
module mkBRAM_from_Mem_Models #(Vector #(n_, Mem_Model_Gen_IFC #(addr_i_, SizeOf #(Byte), a_, b_, c_, d_)) mem_models)
                              (BRAM_PORT_BE #(Bit #(addr_o_), Bit #(data_o_), n_))
                              provisos ( Add #(a__, addr_i_, addr_o_)
                                       , Mul #(n_, SizeOf #(Byte), data_o_)
                                       );
   Integer verbosity = 0;
   Reg #(Bit #(data_o_)) dw_val <- mkDWire (?);
   Reg #(Bool) rg_started <- mkReg (False);

   rule rl_read (rg_started);
      Vector #(n_, MemoryResponse #(8)) rsps = newVector;
      for (Integer i = 0; i < valueOf (n_); i = i+1) begin
         rsps[i] <- mem_models[i].mem_server.response.get;
      end
      dw_val <= pack (rsps);
      if (verbosity > 0) begin
         $display ("%m mkBRAM_From_Mem_Model rl_read -- raw data: ", fshow (rsps),
                   "  |  packed: ", fshow (pack (rsps)));
      end
   endrule

   method Action put (Bit #(n_) be, Bit #(addr_o_) addr, Bit #(data_o_) data);
      if (verbosity > 0) begin
         $display ("%m mkBRAM_from_Mem_Models put - ",
                   " addr: ", fshow (addr),
                   " be: ", fshow (be),
                   " data: ", fshow (data));
      end
      rg_started <= True;
      if (be != 0) begin
         for (Integer i = 0; i < valueOf (n_); i = i+1) begin
            if (be[i] == 1'b1) begin
               mem_models[i].mem_server.request.put (MemoryRequest { write: True
                                                                   , byteen: signExtend (1'b1)
                                                                   , address: truncate (addr)
                                                                   , data: data[i*8+7:i*8]
                                                                   });
            end
         end
      end else begin
         for (Integer i = 0; i < valueOf (n_); i = i+1) begin
            mem_models[i].mem_server.request.put (MemoryRequest { write   : False
                                                                , byteen  : signExtend (1'b1)
                                                                , address : truncate (addr)
                                                                , data    : data[i*8+7:i*8]
                                                                });
         end
      end
   endmethod
   method Bit #(data_o_) read;
      return dw_val;
   endmethod
endmodule
`endif

endpackage: Near_Mem_TCM_Merged
