This is an implementation of BRAM-based Tightly Coupled Memory for Flute.
This uses several 32-bit-based BRAMs and merges them into an arbitrarily wide
BRAM.
The reason that 32-bit-wide BRAMs are used is that bsc claims that BRAMs with
byte-enables which are wider than 32-bits wide will not be correctly inferred
by Vivado.
The upper bits of the BRAM are used to store tags, and the lower bits store
the data.

This has been tested with 128-bit capabilities and 1 bit of tag.
