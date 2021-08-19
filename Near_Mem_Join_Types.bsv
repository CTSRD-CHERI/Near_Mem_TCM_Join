package Near_Mem_Join_Types;

import ISA_Decls :: *;
import MMU_Cache_Common :: *;

typedef struct {
   Bit #(3) f3;
   WordXL   addr;
`ifdef ISA_PRIV_S
   Priv_Mod priv;
   Bit #(1) sstatus_SUM;
   Bit #(1) mstatus_MXR;
   WordXL   satp;
`endif
`ifdef RVFI_DII
   Dii_Id seq_req;
`endif
} IMem_Req deriving (Bits, Eq, FShow);

typedef struct {
   CacheOp op;
   Bit #(3) f3;
   Bool is_unsigned;
`ifdef ISA_A
   Bit #(5) amo_funct5;
`endif
   Addr   addr;
   Tuple2 #(Bool, Bit #(XLEN_2)) store_value;
`ifdef ISA_PRIV_S
   Priv_Mod priv;
   Bit #(1) sstatus_SUM;
   Bit #(1) mstatus_MXR;
   WordXL   satp;
`endif
} DMem_Req deriving (Bits, Eq, FShow);


endpackage
