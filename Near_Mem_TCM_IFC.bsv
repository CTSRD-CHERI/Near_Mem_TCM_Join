package Near_Mem_TCM_IFC;

// ================================================================
// BSV lib imports

import ClientServer :: *;

// ================================================================
// Project imports

import ISA_Decls        :: *;

import Fabric_Defs      :: *;

import AXI              :: *;

import Near_Mem_IFC     :: *;

// ================================================================

interface Near_Mem_TCM_IFC;
   // Reset
   interface Server #(Token, Token) server_reset;

   // ----------------
   // CPU side
   // These come from whatever Near_Mem_IFC we import above
   interface IMem_IFC  imem;
   interface DMem_IFC  dmem;

   // ----------------
   // Fabric side
   // DMA server interface for back-door access to the IDTCM
   // Used to write into the TCM from peripherals that aren't the main CPU
   interface AXI4_Slave #( Wd_SId_2x4, Wd_Addr, Wd_Data
                         , Wd_AW_User, Wd_W_User, Wd_B_User
                         , Wd_AR_User, Wd_R_User)  tcm_dma_server;

   // ----------------
   // Fences
   interface Server #(Token, Token) server_fence_i;
   interface Server #(Fence_Ordering, Token) server_fence;
`ifdef ISA_PRIV_S
   interface Server #(Token, Token) sfence_vma_server;
`endif

   // ----------------
   // For ISA tests: watch memory writes to <tohost> addr
`ifdef WATCH_TOHOST
   method Action set_watch_tohost (Bool watch_tohost, Bit #(64) tohost_addr);
   method Bit #(64) mv_tohost_value;
`endif

endinterface

endpackage: Near_Mem_TCM_IFC
