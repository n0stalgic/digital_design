`include "sdram_def.v"
`timescale 1ns/1ns
module sdram16_ctrl (

    // user interface
    wr_req, wr_gnt, wr_addr, wr_data, rd_req, rd_gnt, rd_addr, rd_data, rd_valid,

    // SDRAM control
    ba, addr, cmd, cke, cs_n, dq,

    // system control
    clk,rst                          // 133 MHz clk
);

// system control
input clk,rst;

// data bus size
localparam data_sz = `SDR_SDRAM_DATA_WIDTH;

// memory geometry
localparam ba_sz   = `SDR_SDRAM_BA_SIZE;
localparam row_sz  = `SDR_SDRAM_ROW_SIZE;
localparam col_sz  = `SDR_SDRAM_COL_SIZE;
localparam casl    = `SDR_SDRAM_CASL;

// memory timing
localparam pwr_on_delay = `SDR_SDRAM_PWR_ON_DELAY;
localparam pwr_on_clk_stab = `SDR_SDRAM_CLK_STAB;
localparam pwr_on_ok = pwr_on_clk_stab + pwr_on_delay;
localparam tRP = `SDR_SDRAM_TRP;
localparam tRC = `SDR_SDRAM_TRC;
localparam tMRD = `SDR_SDRAM_TMRD;
localparam sdram_rfr_rate = `SDR_SDRAM_RFR_CYCLE;
localparam init_precharge = pwr_on_ok + tRP + tRP;
localparam init_autoref = init_precharge + (10*tRC);
localparam init_done = init_autoref + tMRD;

// SDRAM controller to user write agent 
input wr_req;
output wr_gnt;
input [row_sz+col_sz-1:0] wr_addr;
input [data_sz-1:0] wr_data;

// SDRAM controller to user read agent
input  rd_req;
output rd_gnt;
input  [col_sz+row_sz-1:0] rd_addr;
output reg [data_sz-1:0] rd_data;
output rd_valid;

// SDRAM to controller I/O
output cke;                 // Clock Enable
output cs_n;                       
output reg [2:0] cmd;                 // cmd consists of {RASn, CASn, WEn}
output reg [ba_sz-1:0] ba;                  // 2-bit Bank Address
output reg [row_sz:0] addr;
inout  reg [data_sz-1:0] dq;                  // DQ - Data I/O


// SDRAM commands - we drop the CSn bit since its active low in all commands
localparam [2:0] sdram_cmd_nop       = 3'b111,
                 sdram_cmd_precharge = 3'b010,
                 sdram_cmd_refresh   = 3'b001,
                 sdram_cmd_mrs       = 3'b000; 


// SDRAM reg mode programming
localparam [3:0] sdram_init_wb      = `SDR_WRITE_BURST,
                 sdram_init_op_mode = `SDR_OP_MODE,
                 sdram_init_casl    = `SDR_CAS_LATENCY,
                 sdram_init_btype   = `SDR_BURST_TYPE,
                 sdram_init_blen    = `SDR_BURST_LEN;

wire stall;					
wire [0:31] sdram_delay_cnt;
wire [0:10] sdram_rfr_cnt;
reg 		   sdram_rfr_needed;
reg [3:0] 	init_autoref_cnt;
reg 			sdram_initialized;


reg[3:0] state, next_state;

// SDRAM control states
`define FSM_INIT 3'b000
`define FSM_IDLE 3'b001
`define FSM_RFR  3'b010
`define FSM_PCH  3'b011

assign cke  = 1'b1;                    // CLK will always be enabled
assign dqm  = 2'b11;
assign cs_n = 1'b0;
assign stall = 1'b0;

// counter for overall DRAM state machine
cnt_binary_ce_clear #(.WIDTH(32)) cnt0 (
            .cke(!stall),
            .clear(state!=next_state),
            .q(sdram_delay_cnt),
            .rst(rst),
            .clk(clk)
);

// next state logic

always @(posedge clk or posedge rst) begin
    if (rst)
        state <= `FSM_INIT;
    else 
        state <= next_state; 
end

always @ (posedge clk or posedge rst) begin
	if (rst)
		sdram_initialized <= 0;
	else
		if (state > `FSM_INIT) 
			sdram_initialized <= 1'b1;
	end 


// controller

always @(*) begin
	 next_state = state;
    case(state)
        `FSM_INIT:
            if (sdram_delay_cnt == init_done)
                next_state = `FSM_IDLE;
        `FSM_IDLE:
            if (sdram_delay_cnt == sdram_rfr_rate)
                next_state = `FSM_RFR;
        `FSM_RFR:
            if (sdram_delay_cnt == tRP + tRC + tRC)
					next_state = `FSM_IDLE;
			

				
    endcase
            
end

// refresh logic
always @ (*) begin
    if (rst) begin
        sdram_rfr_needed <= 0;
    end else
        if (sdram_delay_cnt == sdram_rfr_rate)
            sdram_rfr_needed <= 1'b1;
        else if (state == `FSM_RFR) 
            sdram_rfr_needed <= 1'b0;    
end

// increment auto refresh counter during initialization
always @ (*) begin
	if (rst)
		init_autoref_cnt = 0;
	else
		if (state == `FSM_INIT)
			if (sdram_delay_cnt == (init_precharge + (init_autoref_cnt*tRC)))
				init_autoref_cnt = init_autoref_cnt + 1;
		
end 

// output vectors

always @(*) begin
    cmd = sdram_cmd_nop;
    case(state)

        `FSM_INIT: begin
            if (sdram_delay_cnt == (pwr_on_ok)) begin        // 100 us delay satisfied, hit the precharge
                {ba,addr,cmd} = {15'b110010000000000,sdram_cmd_precharge};
				end else if (init_autoref_cnt < 10) begin // precharge delay satisfied, hit the autoref
                cmd = sdram_cmd_refresh;
            end else if (sdram_delay_cnt == init_autoref) begin // first auto refresh done
                cmd = {5'b00000, sdram_init_wb, sdram_init_op_mode, sdram_init_casl, 
                        sdram_init_btype, sdram_init_blen, sdram_cmd_mrs };
            end
			end
        `FSM_RFR: begin
				if (sdram_delay_cnt == 0)
					cmd = sdram_cmd_precharge;
				else if (sdram_delay_cnt == tRP)
					cmd = sdram_cmd_refresh;
				else if (sdram_delay_cnt == tRP + tRC)
					cmd = sdram_cmd_refresh;
        end
        endcase

    end 
endmodule



// N-bit binary counter with clock enable, clear, and asynch reset
module cnt_binary_ce_clear 
		#( parameter WIDTH=11) 
		(cke, clear, q, dir, rst, clk);

    input cke, clear;
    input dir;
    output reg [0:WIDTH-1] q;
    input rst;
    input clk;

    always @ (posedge clk or posedge rst) begin
        if (rst)
            q <= 0;
        else
            if (cke)
                if (clear)
                    q <= 0;
                else 
                    q <= q + 1;

        end

endmodule

