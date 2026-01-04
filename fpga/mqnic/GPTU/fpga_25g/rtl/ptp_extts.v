/*
 * File: ptp_extts.v
 * Description: PTP External Timestamp Latcher
 * Author: Xiaohai Li
 * Date: 2026-01-03
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module ptp_extts #
(
    parameter FNS_ENABLE = 0
)
(
    // clk domain (PCIe / register interface)
    input  wire         clk,
    input  wire         rst,

    // Control (clk domain)
    input  wire         enable,
    input  wire         arm,

    // Status (clk domain)
    output wire [95:0]  extts_latched,
    output wire         locked,
    output wire         step,        // kept for interface compatibility, but not used

    // ptp_clk domain
    input  wire         ptp_clk,
    input  wire         ptp_rst,

    // Timestamp input from PTP clock (ptp_clk domain)
    input  wire [95:0]  input_ts_96,
    input  wire         input_ts_step,

    // External timestamp trigger input (asynchronous to ptp_clk)
    input  wire         extts_trig_in
);

    // -------------------------------------------------------------------------
    // ptp_clk domain
    // -------------------------------------------------------------------------

    // Synchronize extts_trig_in into ptp_clk and detect rising edge
    reg extts_sync_0_reg = 1'b0;
    reg extts_sync_1_reg = 1'b0;
    reg extts_sync_2_reg = 1'b0;

    always @(posedge ptp_clk) begin
        if (ptp_rst) begin
            extts_sync_0_reg <= 1'b0;
            extts_sync_1_reg <= 1'b0;
            extts_sync_2_reg <= 1'b0;
        end
        else begin
            extts_sync_0_reg <= extts_trig_in;
            extts_sync_1_reg <= extts_sync_0_reg;
            extts_sync_2_reg <= extts_sync_1_reg;
        end
    end

    wire extts_rise_ptp = extts_sync_1_reg & ~extts_sync_2_reg;

    // Latched timestamp in ptp_clk domain
    reg [95:0] ts_96_ptp_reg = 96'd0;

    // Event toggle used to signal "new timestamp" into clk domain
    reg ts_event_toggle_reg = 1'b0;

    always @(posedge ptp_clk) begin
        if (ptp_rst) begin
            ts_96_ptp_reg      <= 96'd0;
            ts_event_toggle_reg <= 1'b0;
        end
        else begin
            // On rising edge of external trigger, capture timestamp and
            // toggle event flag
            if (extts_rise_ptp) begin
                ts_96_ptp_reg      <= input_ts_96;
                ts_event_toggle_reg <= ~ts_event_toggle_reg;
            end
        end
    end

    // -------------------------------------------------------------------------
    // CDC handshake to clk domain
    // -------------------------------------------------------------------------

    // Synchronize event toggle into clk domain
    reg ts_event_sync_0_reg = 1'b0;
    reg ts_event_sync_1_reg = 1'b0;
    reg ts_event_sync_2_reg = 1'b0;
    reg ts_event_sync_3_reg = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            ts_event_sync_0_reg <= 1'b0;
            ts_event_sync_1_reg <= 1'b0;
            ts_event_sync_2_reg <= 1'b0;
            ts_event_sync_3_reg <= 1'b0;
        end
        else begin
            ts_event_sync_0_reg <= ts_event_toggle_reg;
            ts_event_sync_1_reg <= ts_event_sync_0_reg;
            ts_event_sync_2_reg <= ts_event_sync_1_reg;
            ts_event_sync_3_reg <= ts_event_sync_2_reg;
        end
    end

    // New timestamp event in clk domain (any change of toggle)
    wire ts_valid = ts_event_sync_2_reg ^ ts_event_sync_3_reg;

    // Re-register timestamp into clk domain
    reg [95:0] ts_96_sync_0_reg = 96'd0;
    reg [95:0] ts_96_sync_1_reg = 96'd0;

    always @(posedge clk) begin
        if (rst) begin
            ts_96_sync_0_reg <= 96'd0;
            ts_96_sync_1_reg <= 96'd0;
        end
        else begin
            ts_96_sync_0_reg <= ts_96_ptp_reg;
            ts_96_sync_1_reg <= ts_96_sync_0_reg;
        end
    end

    // Latched outputs (clk domain)
    reg [47:0] time_s_reg   = 48'd0;
    reg [30:0] time_ns_reg  = 31'd0;
    reg [15:0] time_fns_reg = 16'd0;

    reg locked_reg = 1'b0;
    reg step_reg   = 1'b0;  // not used, kept for interface

    always @(posedge clk) begin
        if (rst) begin
            time_s_reg   <= 48'd0;
            time_ns_reg  <= 31'd0;
            time_fns_reg <= 16'd0;

            locked_reg <= 1'b0;
            step_reg   <= 1'b0;
        end
        else if (enable) begin
            // No PHC step handling in this version
            step_reg <= 1'b0;

            // Latch new timestamp only if not already locked
            if (ts_valid && ~locked_reg) begin
                locked_reg <= 1'b1;

                time_s_reg  <= ts_96_sync_1_reg[95:48];
                time_ns_reg <= ts_96_sync_1_reg[45:16];

                if (FNS_ENABLE) begin
                    time_fns_reg <= ts_96_sync_1_reg[15:0];
                end else begin
                    time_fns_reg <= 16'd0;
                end
            end
            // Software re-arm to allow the next latch
            else if (arm) begin
                locked_reg <= 1'b0;
            end
        end
        else begin
            step_reg   <= 1'b0;
        end
    end

    assign locked        = locked_reg;
    assign step          = step_reg;
    assign extts_latched = {time_s_reg[47:0], 2'b00, time_ns_reg[29:0], time_fns_reg[15:0]};

endmodule

`resetall
