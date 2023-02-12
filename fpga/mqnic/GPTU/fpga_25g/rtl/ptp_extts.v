/*

PTP External Timestamp Latcher
By Xiaohai Li (haixiaolee@gmail.com)

*/

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * PTP period out module
 */
module ptp_extts #
(
    parameter FNS_ENABLE = 1,
    parameter EXTTS_CALI_S = 48'h0,
    parameter EXTTS_CALI_NS = 30'h14,
    parameter EXTTS_CALI_FNS = 16'h0000
)
(
    input  wire         clk,
    input  wire         rst,

    input  wire         ptp_clk,
    input  wire         ptp_rst,

    /*
     * External timestamp trigger input
     */    
    input  wire         extts_trig_in,

    /*
     * Timestamp input from PTP clock
     */
    input  wire [95:0]  input_ts_96,
    input  wire         input_ts_step,

    /*
     * Control
     */
    input  wire         enable,
    input  wire         arm,
    input  wire [95:0]  input_cali,
    input  wire         input_cali_valid,

    /*
     * Status
     */
    output wire [95:0]  extts_latched,
    output wire         locked,
    output wire         step
);

// 
reg [4:0] sync_trig_reg = 0;
reg [96:0] ts_96_reg = 0;
reg ts_96_valid_reg = 0;
reg ts_96_step_reg = 0;

reg [47:0] cali_s_reg = EXTTS_CALI_S;
reg [30:0] cali_ns_reg = EXTTS_CALI_NS;
reg [15:0] cali_fns_reg = EXTTS_CALI_FNS;

reg [47:0] time_s_reg = 0;
reg [30:0] time_ns_reg = 0;
reg [15:0] time_fns_reg = 0;
reg [96:0] time_ts_sync_0_reg = 0;
reg [96:0] time_ts_sync_1_reg = 0;
reg [1:0]  time_valid_reg = 0;
reg [1:0]  time_step_reg = 0;

reg locked_reg = 1'b0;
reg step_reg = 1'b0;

wire sync_trig_rise;
wire sync_trig_fall;

assign sync_trig_rise = (~ sync_trig_reg[4]) & sync_trig_reg[3];
assign sync_trig_fall = (~ sync_trig_reg[3]) & sync_trig_reg[4];

assign locked = locked_reg;
assign step = step_reg;
assign extts_latched = {time_s_reg[47:0], 2'b00, time_ns_reg[29:0], time_fns_reg[15:0]};

always @(posedge ptp_clk) begin
    // Sync registers will cause 5 cycles of delay (e.g. 20ns if ptp clock = 250MHz)
    sync_trig_reg[0] <= extts_trig_in;
    sync_trig_reg[1] <= sync_trig_reg[0];
    sync_trig_reg[2] <= sync_trig_reg[1];
    sync_trig_reg[3] <= sync_trig_reg[2];
    sync_trig_reg[4] <= sync_trig_reg[3];

    if (ptp_rst) begin
        sync_trig_reg <= 0;
    end
end

always @(posedge ptp_clk) begin    
    ts_96_valid_reg <= 1'b0;
    ts_96_step_reg <= 1'b0;

    if (input_ts_step) begin
        ts_96_step_reg <= 1'b1;
    end
    else if (sync_trig_rise) begin
        ts_96_reg <= input_ts_96;
        ts_96_valid_reg <= 1'b1;
    end

    if (ptp_rst) begin
        ts_96_reg <= 0;
        ts_96_step_reg <= 1'b0;
        ts_96_valid_reg <= 1'b0;
    end
end

always @(posedge clk) begin
    time_ts_sync_0_reg <= ts_96_reg;
    time_ts_sync_1_reg <= time_ts_sync_0_reg;

    time_valid_reg[0] <= ts_96_valid_reg;
    time_valid_reg[1] <= time_valid_reg[0];
    
    time_step_reg[0] <= ts_96_step_reg;
    time_step_reg[1] <= time_step_reg[0];

    if (ptp_rst) begin
        time_ts_sync_0_reg <= 0;
        time_ts_sync_1_reg <= 0;
        time_valid_reg <= 0;
        time_step_reg <= 0;
    end
end



always @(posedge clk) begin

    if (input_cali_valid) begin
        cali_s_reg <= input_cali[95:48];
        cali_ns_reg <= input_cali[45:16];
        if (FNS_ENABLE) begin
            cali_fns_reg <= input_cali[15:0];
        end
    end

    if (enable) begin
        if (time_step_reg[1]) begin
            step_reg <= 1'b1;
            locked_reg <= 1'b0;

            time_s_reg <= 0;
            time_ns_reg <= 0;
            time_fns_reg <= 0;
        end
        // Only support rise edge for now
        else if (time_valid_reg[1]) begin
            step_reg <= 1'b0;
            locked_reg <= 1'b1;
             
            time_s_reg <= time_ts_sync_1_reg[95:48] - cali_s_reg;
            time_ns_reg <= time_ts_sync_1_reg[45:16] - cali_ns_reg;
            if (FNS_ENABLE) begin
                time_fns_reg <= time_ts_sync_1_reg[15:0] - cali_fns_reg;
            end
        end
        // Triggered by control reg reading
        else if (arm) begin
            step_reg <= 1'b0;
            locked_reg <= 1'b0;

            time_s_reg <= 0;
            time_ns_reg <= 0;
            time_fns_reg <= 0;
        end
    end

    if (rst) begin
        cali_s_reg <= EXTTS_CALI_S;
        cali_ns_reg <= EXTTS_CALI_NS;
        cali_fns_reg <= EXTTS_CALI_FNS;

        time_s_reg <= 0;
        time_ns_reg <= 0;
        time_fns_reg <= 0;

        locked_reg <= 1'b0;
        step_reg <= 1'b0;
    end
end

endmodule

`resetall
