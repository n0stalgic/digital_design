// Defines for SDR SDRAM controller

`define  SDR_SDRAM_ADDR_SIZE           23
`define  SDR_SDRAM_MEM_SIZE_MB         64
`ifdef  SDR_SDRAM_MEM_SIZE_MB
`define  SDR_SDRAM_MEM_SIZE         SDR_SDRAM_MEM_SIZE_MB*1_048_576
`endif

// memory geometry
`define SDR_SDRAM_BA_SIZE     2
`define SDR_SDRAM_ROW_SIZE    13
`define SDR_SDRAM_COL_SIZE    10
`define SDR_SDRAM_DATA_WIDTH  16
`define SDR_SDRAM_CASL        2

// memory timing (cycles @ 133 MHz speed grade 7)
`define  SDR_SDRAM_TRP         2
`define  SDR_SDRAM_TRCD        2
`define  SDR_SDRAM_TRC         8
`define  SDR_SDRAM_CLK_STAB    5
`define  SDR_SDRAM_TMRD        2
`define  SDR_SDRAM_PWR_ON_DELAY  13_250

// memory refresh timings
`define SDR_SDRAM_RFR_BIT_SZ  11
`define SDR_SDRAM_RFR_CYCLE   1041


// LOAD MODE REG memory mode control
//--------------------------------------------------------------------------------
// Burst Length: 1- 3'b000, 2- 3'b001, 4- 3'b010, 8- 3'b011, full page- 3'b111
// Burst Type: Sequential- 1'b0, Interleaved 1'b1
// Latency Mode: 2- 3'b010, 3- 3'b011
// Write Burst: Programmed Burst Length- 1'b0, Single location access 1'b1

// Burst Length of 4
`define SDR_BURST_LEN       3'b010
// Burst Type - Interleaved
`define SDR_BURST_TYPE      1'b1
// CAS latency of 2
`define SDR_CAS_LATENCY     3'b010
// Write burst 
`define SDR_WRITE_BURST     1'b0
// Operating Mode
`define SDR_OP_MODE         2'b00


