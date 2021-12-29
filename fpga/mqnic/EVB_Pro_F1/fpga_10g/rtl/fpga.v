/*

Copyright 2019, The Regents of the University of California.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE REGENTS OF THE UNIVERSITY OF CALIFORNIA ''AS
IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OF THE UNIVERSITY OF CALIFORNIA OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of The Regents of the University of California.

*/

// Language: Verilog 2001

`timescale 1ns / 1ps

/*
 * FPGA top-level module
 */
module fpga (
    /*
     * Clock: 156.25MHz LVDS
     */
    input  wire         clk_161mhz_p,
    input  wire         clk_161mhz_n,

    /*
     * GPIO
     */
    output wire         user_led,

    /*
     *************** FPGA0 D0 ***************
     */

    /*
     * PCI express
     */
    input  wire [7:0]   pcie0_rx_p,
    input  wire [7:0]   pcie0_rx_n,
    output wire [7:0]   pcie0_tx_p,
    output wire [7:0]   pcie0_tx_n,
    input  wire         pcie0_refclk_0_p,
    input  wire         pcie0_refclk_0_n,
    // input  wire         pcie0_refclk_1_p,
    // input  wire         pcie0_refclk_1_n,
    input  wire         pcie0_reset_n,

    /*
     * Ethernet: QSFP28
     */
    output wire         qsfp0_1_tx1_p,
    output wire         qsfp0_1_tx1_n,
    input  wire         qsfp0_1_rx1_p,
    input  wire         qsfp0_1_rx1_n,
    output wire         qsfp0_1_tx2_p,
    output wire         qsfp0_1_tx2_n,
    input  wire         qsfp0_1_rx2_p,
    input  wire         qsfp0_1_rx2_n,
    output wire         qsfp0_1_tx3_p,
    output wire         qsfp0_1_tx3_n,
    input  wire         qsfp0_1_rx3_p,
    input  wire         qsfp0_1_rx3_n,
    output wire         qsfp0_1_tx4_p,
    output wire         qsfp0_1_tx4_n,
    input  wire         qsfp0_1_rx4_p,
    input  wire         qsfp0_1_rx4_n,
    input  wire         qsfp0_1_mgt_refclk_0_p,
    input  wire         qsfp0_1_mgt_refclk_0_n,

    output wire         qsfp0_2_tx1_p,
    output wire         qsfp0_2_tx1_n,
    input  wire         qsfp0_2_rx1_p,
    input  wire         qsfp0_2_rx1_n,
    output wire         qsfp0_2_tx2_p,
    output wire         qsfp0_2_tx2_n,
    input  wire         qsfp0_2_rx2_p,
    input  wire         qsfp0_2_rx2_n,
    output wire         qsfp0_2_tx3_p,
    output wire         qsfp0_2_tx3_n,
    input  wire         qsfp0_2_rx3_p,
    input  wire         qsfp0_2_rx3_n,
    output wire         qsfp0_2_tx4_p,
    output wire         qsfp0_2_tx4_n,
    input  wire         qsfp0_2_rx4_p,
    input  wire         qsfp0_2_rx4_n,
    // input  wire         qsfp0_2_mgt_refclk_0_p,
    // input  wire         qsfp0_2_mgt_refclk_0_n,

    /*
     *************** FPGA0 D1 ***************
     */

    /*
     * PCI express
     */
    input  wire [7:0]   pcie1_rx_p,
    input  wire [7:0]   pcie1_rx_n,
    output wire [7:0]   pcie1_tx_p,
    output wire [7:0]   pcie1_tx_n,
    input  wire         pcie1_refclk_0_p,
    input  wire         pcie1_refclk_0_n,
    // input  wire         pcie1_refclk_1_p,
    // input  wire         pcie1_refclk_1_n,
    input  wire         pcie1_reset_n,

    /*
     * Ethernet: QSFP28
     */
    output wire         qsfp1_1_tx1_p,
    output wire         qsfp1_1_tx1_n,
    input  wire         qsfp1_1_rx1_p,
    input  wire         qsfp1_1_rx1_n,
    output wire         qsfp1_1_tx2_p,
    output wire         qsfp1_1_tx2_n,
    input  wire         qsfp1_1_rx2_p,
    input  wire         qsfp1_1_rx2_n,
    output wire         qsfp1_1_tx3_p,
    output wire         qsfp1_1_tx3_n,
    input  wire         qsfp1_1_rx3_p,
    input  wire         qsfp1_1_rx3_n,
    output wire         qsfp1_1_tx4_p,
    output wire         qsfp1_1_tx4_n,
    input  wire         qsfp1_1_rx4_p,
    input  wire         qsfp1_1_rx4_n,
    input  wire         qsfp1_1_mgt_refclk_0_p,
    input  wire         qsfp1_1_mgt_refclk_0_n,

    output wire         qsfp1_2_tx1_p,
    output wire         qsfp1_2_tx1_n,
    input  wire         qsfp1_2_rx1_p,
    input  wire         qsfp1_2_rx1_n,
    output wire         qsfp1_2_tx2_p,
    output wire         qsfp1_2_tx2_n,
    input  wire         qsfp1_2_rx2_p,
    input  wire         qsfp1_2_rx2_n,
    output wire         qsfp1_2_tx3_p,
    output wire         qsfp1_2_tx3_n,
    input  wire         qsfp1_2_rx3_p,
    input  wire         qsfp1_2_rx3_n,
    output wire         qsfp1_2_tx4_p,
    output wire         qsfp1_2_tx4_n,
    input  wire         qsfp1_2_rx4_p,
    input  wire         qsfp1_2_rx4_n,
    // input  wire         qsfp1_2_mgt_refclk_0_p,
    // input  wire         qsfp1_2_mgt_refclk_0_n,

    /*
     *************** FPGA0 D2 ***************
     */

    /*
     * PCI express
     */
    input  wire [7:0]   pcie2_rx_p,
    input  wire [7:0]   pcie2_rx_n,
    output wire [7:0]   pcie2_tx_p,
    output wire [7:0]   pcie2_tx_n,
    input  wire         pcie2_refclk_0_p,
    input  wire         pcie2_refclk_0_n,
    // input  wire         pcie2_refclk_1_p,
    // input  wire         pcie2_refclk_1_n,
    input  wire         pcie2_reset_n,

    /*
     * Ethernet: QSFP28
     */
    output wire         qsfp2_1_tx1_p,
    output wire         qsfp2_1_tx1_n,
    input  wire         qsfp2_1_rx1_p,
    input  wire         qsfp2_1_rx1_n,
    output wire         qsfp2_1_tx2_p,
    output wire         qsfp2_1_tx2_n,
    input  wire         qsfp2_1_rx2_p,
    input  wire         qsfp2_1_rx2_n,
    output wire         qsfp2_1_tx3_p,
    output wire         qsfp2_1_tx3_n,
    input  wire         qsfp2_1_rx3_p,
    input  wire         qsfp2_1_rx3_n,
    output wire         qsfp2_1_tx4_p,
    output wire         qsfp2_1_tx4_n,
    input  wire         qsfp2_1_rx4_p,
    input  wire         qsfp2_1_rx4_n,
    input  wire         qsfp2_1_mgt_refclk_0_p,
    input  wire         qsfp2_1_mgt_refclk_0_n,

    output wire         qsfp2_2_tx1_p,
    output wire         qsfp2_2_tx1_n,
    input  wire         qsfp2_2_rx1_p,
    input  wire         qsfp2_2_rx1_n,
    output wire         qsfp2_2_tx2_p,
    output wire         qsfp2_2_tx2_n,
    input  wire         qsfp2_2_rx2_p,
    input  wire         qsfp2_2_rx2_n,
    output wire         qsfp2_2_tx3_p,
    output wire         qsfp2_2_tx3_n,
    input  wire         qsfp2_2_rx3_p,
    input  wire         qsfp2_2_rx3_n,
    output wire         qsfp2_2_tx4_p,
    output wire         qsfp2_2_tx4_n,
    input  wire         qsfp2_2_rx4_p,
    input  wire         qsfp2_2_rx4_n
    // input  wire         qsfp2_2_mgt_refclk_0_p,
    // input  wire         qsfp2_2_mgt_refclk_0_n
);

// PCIe bus config refered to ExaNIC_X25
parameter AXIS_PCIE_DATA_WIDTH = 256;
parameter AXIS_PCIE_KEEP_WIDTH = (AXIS_PCIE_DATA_WIDTH/32);
parameter AXIS_PCIE_RC_USER_WIDTH = 75;
parameter AXIS_PCIE_RQ_USER_WIDTH = 62;
parameter AXIS_PCIE_CQ_USER_WIDTH = 88;
parameter AXIS_PCIE_CC_USER_WIDTH = 33;
parameter RQ_SEQ_NUM_WIDTH = 6;
parameter BAR0_APERTURE = 26;

// Clock and reset
wire pcie0_user_clk;
wire pcie1_user_clk;
wire pcie2_user_clk;
wire pcie0_user_reset;
wire pcie1_user_reset;
wire pcie2_user_reset;

wire clk_161mhz_ibufg; // All three dies share the same system clock

wire clk_125mhz_mmcm_out0;
wire clk_125mhz_mmcm_out1;
wire clk_125mhz_mmcm_out2;

// Internal 125 MHz clock
wire clk_125mhz_int0;
wire clk_125mhz_int1;
wire clk_125mhz_int2;
wire rst_125mhz_int0;
wire rst_125mhz_int1;
wire rst_125mhz_int2;

// Internal 156.25 MHz clock
wire clk_156mhz_int0;
wire clk_156mhz_int1;
wire clk_156mhz_int2;
wire rst_156mhz_int0;
wire rst_156mhz_int1;
wire rst_156mhz_int2;

wire mmcm_rst = pcie1_user_reset;
wire mmcm_locked;
wire mmcm_clkfb;

IBUFGDS #(
   .DIFF_TERM("FALSE"),
   .IBUF_LOW_PWR("FALSE")   
)
clk_161mhz_ibufg_inst (
   .O   (clk_161mhz_ibufg),
   .I   (clk_161mhz_p),
   .IB  (clk_161mhz_n) 
);

// MMCM instance
// 161.1328125 MHz in, 125 MHz out
// PFD range: 10 MHz to 500 MHz
// VCO range: 800 MHz to 1600 MHz
// M = 128, D = 15 sets Fvco = 1375 MHz (in range)
// Divide by 11 to get output frequency of 125 MHz
MMCME4_BASE #(
    .BANDWIDTH("OPTIMIZED"),
    .CLKOUT0_DIVIDE_F(11),
    .CLKOUT0_DUTY_CYCLE(0.5),
    .CLKOUT0_PHASE(0),
    .CLKOUT1_DIVIDE(11),
    .CLKOUT1_DUTY_CYCLE(0.5),
    .CLKOUT1_PHASE(0),
    .CLKOUT2_DIVIDE(11),
    .CLKOUT2_DUTY_CYCLE(0.5),
    .CLKOUT2_PHASE(0),
    .CLKOUT3_DIVIDE(1),
    .CLKOUT3_DUTY_CYCLE(0.5),
    .CLKOUT3_PHASE(0),
    .CLKOUT4_DIVIDE(1),
    .CLKOUT4_DUTY_CYCLE(0.5),
    .CLKOUT4_PHASE(0),
    .CLKOUT5_DIVIDE(1),
    .CLKOUT5_DUTY_CYCLE(0.5),
    .CLKOUT5_PHASE(0),
    .CLKOUT6_DIVIDE(1),
    .CLKOUT6_DUTY_CYCLE(0.5),
    .CLKOUT6_PHASE(0),
    .CLKFBOUT_MULT_F(128),
    .CLKFBOUT_PHASE(0),
    .DIVCLK_DIVIDE(15),
    .REF_JITTER1(0.010),
    .CLKIN1_PERIOD(6.206),
    .STARTUP_WAIT("FALSE"),
    .CLKOUT4_CASCADE("FALSE")
)
clk_mmcm_inst (
    .CLKIN1(clk_161mhz_ibufg),
    .CLKFBIN(mmcm_clkfb),
    .RST(mmcm_rst),
    .PWRDWN(1'b0),
    .CLKOUT0(clk_125mhz_mmcm_out0),
    .CLKOUT0B(),
    .CLKOUT1(clk_125mhz_mmcm_out1),
    .CLKOUT1B(),
    .CLKOUT2(clk_125mhz_mmcm_out2),
    .CLKOUT2B(),
    .CLKOUT3(),
    .CLKOUT3B(),
    .CLKOUT4(),
    .CLKOUT5(),
    .CLKOUT6(),
    .CLKFBOUT(mmcm_clkfb),
    .CLKFBOUTB(),
    .LOCKED(mmcm_locked)
);

BUFG
clk_125mhz_bufg_inst0 (
    .I(clk_125mhz_mmcm_out0),
    .O(clk_125mhz_int0)
);

sync_reset #(
    .N(4)
)
sync_reset_125mhz_inst0 (
    .clk(clk_125mhz_int0),
    .rst(~mmcm_locked),
    .out(rst_125mhz_int0)
);

BUFG
clk_125mhz_bufg_inst1 (
    .I(clk_125mhz_mmcm_out1),
    .O(clk_125mhz_int1)
);

sync_reset #(
    .N(4)
)
sync_reset_125mhz_inst1 (
    .clk(clk_125mhz_int1),
    .rst(~mmcm_locked),
    .out(rst_125mhz_int1)
);

BUFG
clk_125mhz_bufg_inst2 (
    .I(clk_125mhz_mmcm_out2),
    .O(clk_125mhz_int2)
);

sync_reset #(
    .N(4)
)
sync_reset_125mhz_inst2 (
    .clk(clk_125mhz_int2),
    .rst(~mmcm_locked),
    .out(rst_125mhz_int2)
);

// Flash
wire qspi_clk_int;
wire [3:0] qspi_0_dq_int;
wire [3:0] qspi_0_dq_i_int;
wire [3:0] qspi_0_dq_o_int;
wire [3:0] qspi_0_dq_oe_int;
wire qspi_0_cs_int;

reg qspi_clk_reg;
reg [3:0] qspi_0_dq_o_reg;
reg [3:0] qspi_0_dq_oe_reg;
reg qspi_0_cs_reg;

always @(posedge pcie1_user_clk) begin
    qspi_clk_reg <= qspi_clk_int;
    qspi_0_dq_o_reg <= qspi_0_dq_o_int;
    qspi_0_dq_oe_reg <= qspi_0_dq_oe_int;
    qspi_0_cs_reg <= qspi_0_cs_int;
end

sync_signal #(
    .WIDTH(4),
    .N(2)
)
flash_sync_signal_inst (
    .clk(pcie1_user_clk),
    .in(qspi_0_dq_int),
    .out(qspi_0_dq_i_int)
);

STARTUPE3
startupe3_inst (
    .CFGCLK(),
    .CFGMCLK(),
    .DI(qspi_0_dq_int),
    .DO(qspi_0_dq_o_reg),
    .DTS(~qspi_0_dq_oe_reg),
    .EOS(),
    .FCSBO(qspi_0_cs_reg),
    .FCSBTS(1'b0),
    .GSR(1'b0),
    .GTS(1'b0),
    .KEYCLEARB(1'b1),
    .PACK(1'b0),
    .PREQ(),
    .USRCCLKO(qspi_clk_reg),
    .USRCCLKTS(1'b0),
    .USRDONEO(1'b0),
    .USRDONETS(1'b1)
);

// FPGA boot
wire fpga_boot;

reg fpga_boot_sync_reg_0 = 1'b0;
reg fpga_boot_sync_reg_1 = 1'b0;
reg fpga_boot_sync_reg_2 = 1'b0;

wire icap_avail;
reg [2:0] icap_state = 0;
reg icap_csib_reg = 1'b1;
reg icap_rdwrb_reg = 1'b0;
reg [31:0] icap_di_reg = 32'hffffffff;

wire [31:0] icap_di_rev;

assign icap_di_rev[ 7] = icap_di_reg[ 0];
assign icap_di_rev[ 6] = icap_di_reg[ 1];
assign icap_di_rev[ 5] = icap_di_reg[ 2];
assign icap_di_rev[ 4] = icap_di_reg[ 3];
assign icap_di_rev[ 3] = icap_di_reg[ 4];
assign icap_di_rev[ 2] = icap_di_reg[ 5];
assign icap_di_rev[ 1] = icap_di_reg[ 6];
assign icap_di_rev[ 0] = icap_di_reg[ 7];

assign icap_di_rev[15] = icap_di_reg[ 8];
assign icap_di_rev[14] = icap_di_reg[ 9];
assign icap_di_rev[13] = icap_di_reg[10];
assign icap_di_rev[12] = icap_di_reg[11];
assign icap_di_rev[11] = icap_di_reg[12];
assign icap_di_rev[10] = icap_di_reg[13];
assign icap_di_rev[ 9] = icap_di_reg[14];
assign icap_di_rev[ 8] = icap_di_reg[15];

assign icap_di_rev[23] = icap_di_reg[16];
assign icap_di_rev[22] = icap_di_reg[17];
assign icap_di_rev[21] = icap_di_reg[18];
assign icap_di_rev[20] = icap_di_reg[19];
assign icap_di_rev[19] = icap_di_reg[20];
assign icap_di_rev[18] = icap_di_reg[21];
assign icap_di_rev[17] = icap_di_reg[22];
assign icap_di_rev[16] = icap_di_reg[23];

assign icap_di_rev[31] = icap_di_reg[24];
assign icap_di_rev[30] = icap_di_reg[25];
assign icap_di_rev[29] = icap_di_reg[26];
assign icap_di_rev[28] = icap_di_reg[27];
assign icap_di_rev[27] = icap_di_reg[28];
assign icap_di_rev[26] = icap_di_reg[29];
assign icap_di_rev[25] = icap_di_reg[30];
assign icap_di_rev[24] = icap_di_reg[31];

always @(posedge clk_125mhz_int1) begin
    case (icap_state)
        0: begin
            icap_state <= 0;
            icap_csib_reg <= 1'b1;
            icap_rdwrb_reg <= 1'b0;
            icap_di_reg <= 32'hffffffff; // dummy word

            if (fpga_boot_sync_reg_2 && icap_avail) begin
                icap_state <= 1;
                icap_csib_reg <= 1'b0;
                icap_rdwrb_reg <= 1'b0;
                icap_di_reg <= 32'hffffffff; // dummy word
            end
        end
        1: begin
            icap_state <= 2;
            icap_csib_reg <= 1'b0;
            icap_rdwrb_reg <= 1'b0;
            icap_di_reg <= 32'hAA995566; // sync word
        end
        2: begin
            icap_state <= 3;
            icap_csib_reg <= 1'b0;
            icap_rdwrb_reg <= 1'b0;
            icap_di_reg <= 32'h20000000; // type 1 noop
        end
        3: begin
            icap_state <= 4;
            icap_csib_reg <= 1'b0;
            icap_rdwrb_reg <= 1'b0;
            icap_di_reg <= 32'h30008001; // write 1 word to CMD
        end
        4: begin
            icap_state <= 5;
            icap_csib_reg <= 1'b0;
            icap_rdwrb_reg <= 1'b0;
            icap_di_reg <= 32'h0000000F; // IPROG
        end
        5: begin
            icap_state <= 0;
            icap_csib_reg <= 1'b0;
            icap_rdwrb_reg <= 1'b0;
            icap_di_reg <= 32'h20000000; // type 1 noop
        end
    endcase

    fpga_boot_sync_reg_0 <= fpga_boot;
    fpga_boot_sync_reg_1 <= fpga_boot_sync_reg_0;
    fpga_boot_sync_reg_2 <= fpga_boot_sync_reg_1;
end

ICAPE3
icape3_inst (
    .AVAIL(icap_avail),
    .CLK(clk_125mhz_int1),
    .CSIB(icap_csib_reg),
    .I(icap_di_rev),
    .O(),
    .PRDONE(),
    .PRERROR(),
    .RDWRB(icap_rdwrb_reg)
);

/*
 ****************************** FPGA0 D0 ********************************
 */

// PCIe
wire pcie0_sys_clk;
wire pcie0_sys_clk_gt;

IBUFDS_GTE4 #(
    .REFCLK_HROW_CK_SEL(2'b00)
)
ibufds_gte4_pcie_mgt_refclk_inst0 (
    .I             (pcie0_refclk_0_p),
    .IB            (pcie0_refclk_0_n),
    .CEB           (1'b0),
    .O             (pcie0_sys_clk_gt),
    .ODIV2         (pcie0_sys_clk)
);

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis0_rq_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis0_rq_tkeep;
wire                               axis0_rq_tlast;
wire                               axis0_rq_tready;
wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis0_rq_tuser;
wire                               axis0_rq_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis0_rc_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis0_rc_tkeep;
wire                               axis0_rc_tlast;
wire                               axis0_rc_tready;
wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis0_rc_tuser;
wire                               axis0_rc_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis0_cq_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis0_cq_tkeep;
wire                               axis0_cq_tlast;
wire                               axis0_cq_tready;
wire [AXIS_PCIE_CQ_USER_WIDTH-1:0] axis0_cq_tuser;
wire                               axis0_cq_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis0_cc_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis0_cc_tkeep;
wire                               axis0_cc_tlast;
wire                               axis0_cc_tready;
wire [AXIS_PCIE_CC_USER_WIDTH-1:0] axis0_cc_tuser;
wire                               axis0_cc_tvalid;

wire [RQ_SEQ_NUM_WIDTH-1:0]        pcie0_rq_seq_num0;
wire                               pcie0_rq_seq_num_vld0;
wire [RQ_SEQ_NUM_WIDTH-1:0]        pcie0_rq_seq_num1;
wire                               pcie0_rq_seq_num_vld1;

wire [3:0] pcie0_tfc_nph_av;
wire [3:0] pcie0_tfc_npd_av;

wire [2:0] cfg0_max_payload;
wire [2:0] cfg0_max_read_req;

wire [9:0]  cfg0_mgmt_addr;
wire [7:0]  cfg0_mgmt_function_number;
wire        cfg0_mgmt_write;
wire [31:0] cfg0_mgmt_write_data;
wire [3:0]  cfg0_mgmt_byte_enable;
wire        cfg0_mgmt_read;
wire [31:0] cfg0_mgmt_read_data;
wire        cfg0_mgmt_read_write_done;

wire [7:0]  cfg0_fc_ph;
wire [11:0] cfg0_fc_pd;
wire [7:0]  cfg0_fc_nph;
wire [11:0] cfg0_fc_npd;
wire [7:0]  cfg0_fc_cplh;
wire [11:0] cfg0_fc_cpld;
wire [2:0]  cfg0_fc_sel;

wire [3:0]  cfg0_interrupt_msi_enable;
wire [11:0] cfg0_interrupt_msi_mmenable;
wire        cfg0_interrupt_msi_mask_update;
wire [31:0] cfg0_interrupt_msi_data;
wire [3:0]  cfg0_interrupt_msi_select;
wire [31:0] cfg0_interrupt_msi_int;
wire [31:0] cfg0_interrupt_msi_pending_status;
wire        cfg0_interrupt_msi_pending_status_data_enable;
wire [3:0]  cfg0_interrupt_msi_pending_status_function_num;
wire        cfg0_interrupt_msi_sent;
wire        cfg0_interrupt_msi_fail;
wire [2:0]  cfg0_interrupt_msi_attr;
wire        cfg0_interrupt_msi_tph_present;
wire [1:0]  cfg0_interrupt_msi_tph_type;
wire [8:0]  cfg0_interrupt_msi_tph_st_tag;
wire [3:0]  cfg0_interrupt_msi_function_number;

wire status0_error_cor;
wire status0_error_uncor;

// extra register for pcie0_user_reset signal
wire pcie0_user_reset_int;
(* shreg_extract = "no" *)
reg pcie0_user_reset_reg_1 = 1'b1;
(* shreg_extract = "no" *)
reg pcie0_user_reset_reg_2 = 1'b1;

always @(posedge pcie0_user_clk) begin
    pcie0_user_reset_reg_1 <= pcie0_user_reset_int;
    pcie0_user_reset_reg_2 <= pcie0_user_reset_reg_1;
end

assign pcie0_user_reset = pcie0_user_reset_reg_2;

pcie4_uscale_plus_0
pcie4_uscale_plus_inst0 (
    .pci_exp_txn(pcie0_tx_n),
    .pci_exp_txp(pcie0_tx_p),
    .pci_exp_rxn(pcie0_rx_n),
    .pci_exp_rxp(pcie0_rx_p),
    .user_clk(pcie0_user_clk),
    .user_reset(pcie0_user_reset_int),
    .user_lnk_up(),

    .s_axis_rq_tdata(axis0_rq_tdata),
    .s_axis_rq_tkeep(axis0_rq_tkeep),
    .s_axis_rq_tlast(axis0_rq_tlast),
    .s_axis_rq_tready(axis0_rq_tready),
    .s_axis_rq_tuser(axis0_rq_tuser),
    .s_axis_rq_tvalid(axis0_rq_tvalid),

    .m_axis_rc_tdata(axis0_rc_tdata),
    .m_axis_rc_tkeep(axis0_rc_tkeep),
    .m_axis_rc_tlast(axis0_rc_tlast),
    .m_axis_rc_tready(axis0_rc_tready),
    .m_axis_rc_tuser(axis0_rc_tuser),
    .m_axis_rc_tvalid(axis0_rc_tvalid),

    .m_axis_cq_tdata(axis0_cq_tdata),
    .m_axis_cq_tkeep(axis0_cq_tkeep),
    .m_axis_cq_tlast(axis0_cq_tlast),
    .m_axis_cq_tready(axis0_cq_tready),
    .m_axis_cq_tuser(axis0_cq_tuser),
    .m_axis_cq_tvalid(axis0_cq_tvalid),

    .s_axis_cc_tdata(axis0_cc_tdata),
    .s_axis_cc_tkeep(axis0_cc_tkeep),
    .s_axis_cc_tlast(axis0_cc_tlast),
    .s_axis_cc_tready(axis0_cc_tready),
    .s_axis_cc_tuser(axis0_cc_tuser),
    .s_axis_cc_tvalid(axis0_cc_tvalid),

    .pcie_rq_seq_num0(pcie0_rq_seq_num0),
    .pcie_rq_seq_num_vld0(pcie0_rq_seq_num_vld0),
    .pcie_rq_seq_num1(pcie0_rq_seq_num1),
    .pcie_rq_seq_num_vld1(pcie0_rq_seq_num_vld1),
    .pcie_rq_tag0(),
    .pcie_rq_tag1(),
    .pcie_rq_tag_av(),
    .pcie_rq_tag_vld0(),
    .pcie_rq_tag_vld1(),

    .pcie_tfc_nph_av(pcie0_tfc_nph_av),
    .pcie_tfc_npd_av(pcie0_tfc_npd_av),

    .pcie_cq_np_req(1'b1),
    .pcie_cq_np_req_count(),

    .cfg_phy_link_down(),
    .cfg_phy_link_status(),
    .cfg_negotiated_width(),
    .cfg_current_speed(),
    .cfg_max_payload(cfg0_max_payload),
    .cfg_max_read_req(cfg0_max_read_req),
    .cfg_function_status(),
    .cfg_function_power_state(),
    .cfg_vf_status(),
    .cfg_vf_power_state(),
    .cfg_link_power_state(),

    .cfg_mgmt_addr(cfg0_mgmt_addr),
    .cfg_mgmt_function_number(cfg0_mgmt_function_number),
    .cfg_mgmt_write(cfg0_mgmt_write),
    .cfg_mgmt_write_data(cfg0_mgmt_write_data),
    .cfg_mgmt_byte_enable(cfg0_mgmt_byte_enable),
    .cfg_mgmt_read(cfg0_mgmt_read),
    .cfg_mgmt_read_data(cfg0_mgmt_read_data),
    .cfg_mgmt_read_write_done(cfg0_mgmt_read_write_done),
    .cfg_mgmt_debug_access(1'b0),

    .cfg_err_cor_out(),
    .cfg_err_nonfatal_out(),
    .cfg_err_fatal_out(),
    .cfg_local_error_valid(),
    .cfg_local_error_out(),
    .cfg_ltssm_state(),
    .cfg_rx_pm_state(),
    .cfg_tx_pm_state(),
    .cfg_rcb_status(),
    .cfg_obff_enable(),
    .cfg_pl_status_change(),
    .cfg_tph_requester_enable(),
    .cfg_tph_st_mode(),
    .cfg_vf_tph_requester_enable(),
    .cfg_vf_tph_st_mode(),

    .cfg_msg_received(),
    .cfg_msg_received_data(),
    .cfg_msg_received_type(),
    .cfg_msg_transmit(1'b0),
    .cfg_msg_transmit_type(3'd0),
    .cfg_msg_transmit_data(32'd0),
    .cfg_msg_transmit_done(),

    .cfg_fc_ph(cfg0_fc_ph),
    .cfg_fc_pd(cfg0_fc_pd),
    .cfg_fc_nph(cfg0_fc_nph),
    .cfg_fc_npd(cfg0_fc_npd),
    .cfg_fc_cplh(cfg0_fc_cplh),
    .cfg_fc_cpld(cfg0_fc_cpld),
    .cfg_fc_sel(cfg0_fc_sel),

    .cfg_dsn(64'd0),

    .cfg_power_state_change_ack(1'b1),
    .cfg_power_state_change_interrupt(),

    .cfg_err_cor_in(status0_error_cor),
    .cfg_err_uncor_in(status0_error_uncor),
    .cfg_flr_in_process(),
    .cfg_flr_done(4'd0),
    .cfg_vf_flr_in_process(),
    .cfg_vf_flr_func_num(8'd0),
    .cfg_vf_flr_done(8'd0),

    .cfg_link_training_enable(1'b1),

    .cfg_interrupt_int(4'd0),
    .cfg_interrupt_pending(4'd0),
    .cfg_interrupt_sent(),
    .cfg_interrupt_msi_enable(cfg0_interrupt_msi_enable),
    .cfg_interrupt_msi_mmenable(cfg0_interrupt_msi_mmenable),
    .cfg_interrupt_msi_mask_update(cfg0_interrupt_msi_mask_update),
    .cfg_interrupt_msi_data(cfg0_interrupt_msi_data),
    .cfg_interrupt_msi_select(cfg0_interrupt_msi_select),
    .cfg_interrupt_msi_int(cfg0_interrupt_msi_int),
    .cfg_interrupt_msi_pending_status(cfg0_interrupt_msi_pending_status),
    .cfg_interrupt_msi_pending_status_data_enable(cfg0_interrupt_msi_pending_status_data_enable),
    .cfg_interrupt_msi_pending_status_function_num(cfg0_interrupt_msi_pending_status_function_num),
    .cfg_interrupt_msi_sent(cfg0_interrupt_msi_sent),
    .cfg_interrupt_msi_fail(cfg0_interrupt_msi_fail),
    .cfg_interrupt_msi_attr(cfg0_interrupt_msi_attr),
    .cfg_interrupt_msi_tph_present(cfg0_interrupt_msi_tph_present),
    .cfg_interrupt_msi_tph_type(cfg0_interrupt_msi_tph_type),
    .cfg_interrupt_msi_tph_st_tag(cfg0_interrupt_msi_tph_st_tag),
    .cfg_interrupt_msi_function_number(cfg0_interrupt_msi_function_number),

    .cfg_pm_aspm_l1_entry_reject(1'b0),
    .cfg_pm_aspm_tx_l0s_entry_disable(1'b0),

    .cfg_hot_reset_out(),

    .cfg_config_space_enable(1'b1),
    .cfg_req_pm_transition_l23_ready(1'b0),
    .cfg_hot_reset_in(1'b0),

    .cfg_ds_port_number(8'd0),
    .cfg_ds_bus_number(8'd0),
    .cfg_ds_device_number(5'd0),

    .sys_clk(pcie0_sys_clk),
    .sys_clk_gt(pcie0_sys_clk_gt),
    .sys_reset(pcie0_reset_n),

    .phy_rdy_out()
);

// XGMII 10G PHY
wire        qsfp0_1_tx_clk_1_int;
wire        qsfp0_1_tx_rst_1_int;
wire [63:0] qsfp0_1_txd_1_int;
wire [7:0]  qsfp0_1_txc_1_int;
wire        qsfp0_1_tx_prbs31_enable_1_int;
wire        qsfp0_1_rx_clk_1_int;
wire        qsfp0_1_rx_rst_1_int;
wire [63:0] qsfp0_1_rxd_1_int;
wire [7:0]  qsfp0_1_rxc_1_int;
wire        qsfp0_1_rx_prbs31_enable_1_int;
wire [6:0]  qsfp0_1_rx_error_count_1_int;
wire        qsfp0_1_tx_clk_2_int;
wire        qsfp0_1_tx_rst_2_int;
wire [63:0] qsfp0_1_txd_2_int;
wire [7:0]  qsfp0_1_txc_2_int;
wire        qsfp0_1_tx_prbs31_enable_2_int;
wire        qsfp0_1_rx_clk_2_int;
wire        qsfp0_1_rx_rst_2_int;
wire [63:0] qsfp0_1_rxd_2_int;
wire [7:0]  qsfp0_1_rxc_2_int;
wire        qsfp0_1_rx_prbs31_enable_2_int;
wire [6:0]  qsfp0_1_rx_error_count_2_int;
wire        qsfp0_1_tx_clk_3_int;
wire        qsfp0_1_tx_rst_3_int;
wire [63:0] qsfp0_1_txd_3_int;
wire [7:0]  qsfp0_1_txc_3_int;
wire        qsfp0_1_tx_prbs31_enable_3_int;
wire        qsfp0_1_rx_clk_3_int;
wire        qsfp0_1_rx_rst_3_int;
wire [63:0] qsfp0_1_rxd_3_int;
wire [7:0]  qsfp0_1_rxc_3_int;
wire        qsfp0_1_rx_prbs31_enable_3_int;
wire [6:0]  qsfp0_1_rx_error_count_3_int;
wire        qsfp0_1_tx_clk_4_int;
wire        qsfp0_1_tx_rst_4_int;
wire [63:0] qsfp0_1_txd_4_int;
wire [7:0]  qsfp0_1_txc_4_int;
wire        qsfp0_1_tx_prbs31_enable_4_int;
wire        qsfp0_1_rx_clk_4_int;
wire        qsfp0_1_rx_rst_4_int;
wire [63:0] qsfp0_1_rxd_4_int;
wire [7:0]  qsfp0_1_rxc_4_int;
wire        qsfp0_1_rx_prbs31_enable_4_int;
wire [6:0]  qsfp0_1_rx_error_count_4_int;

wire        qsfp0_2_tx_clk_1_int;
wire        qsfp0_2_tx_rst_1_int;
wire [63:0] qsfp0_2_txd_1_int;
wire [7:0]  qsfp0_2_txc_1_int;
wire        qsfp0_2_tx_prbs31_enable_1_int;
wire        qsfp0_2_rx_clk_1_int;
wire        qsfp0_2_rx_rst_1_int;
wire [63:0] qsfp0_2_rxd_1_int;
wire [7:0]  qsfp0_2_rxc_1_int;
wire        qsfp0_2_rx_prbs31_enable_1_int;
wire [6:0]  qsfp0_2_rx_error_count_1_int;
wire        qsfp0_2_tx_clk_2_int;
wire        qsfp0_2_tx_rst_2_int;
wire [63:0] qsfp0_2_txd_2_int;
wire [7:0]  qsfp0_2_txc_2_int;
wire        qsfp0_2_tx_prbs31_enable_2_int;
wire        qsfp0_2_rx_clk_2_int;
wire        qsfp0_2_rx_rst_2_int;
wire [63:0] qsfp0_2_rxd_2_int;
wire [7:0]  qsfp0_2_rxc_2_int;
wire        qsfp0_2_rx_prbs31_enable_2_int;
wire [6:0]  qsfp0_2_rx_error_count_2_int;
wire        qsfp0_2_tx_clk_3_int;
wire        qsfp0_2_tx_rst_3_int;
wire [63:0] qsfp0_2_txd_3_int;
wire [7:0]  qsfp0_2_txc_3_int;
wire        qsfp0_2_tx_prbs31_enable_3_int;
wire        qsfp0_2_rx_clk_3_int;
wire        qsfp0_2_rx_rst_3_int;
wire [63:0] qsfp0_2_rxd_3_int;
wire [7:0]  qsfp0_2_rxc_3_int;
wire        qsfp0_2_rx_prbs31_enable_3_int;
wire [6:0]  qsfp0_2_rx_error_count_3_int;
wire        qsfp0_2_tx_clk_4_int;
wire        qsfp0_2_tx_rst_4_int;
wire [63:0] qsfp0_2_txd_4_int;
wire [7:0]  qsfp0_2_txc_4_int;
wire        qsfp0_2_tx_prbs31_enable_4_int;
wire        qsfp0_2_rx_clk_4_int;
wire        qsfp0_2_rx_rst_4_int;
wire [63:0] qsfp0_2_rxd_4_int;
wire [7:0]  qsfp0_2_rxc_4_int;
wire        qsfp0_2_rx_prbs31_enable_4_int;
wire [6:0]  qsfp0_2_rx_error_count_4_int;

wire qsfp0_1_rx_block_lock_1;
wire qsfp0_1_rx_block_lock_2;
wire qsfp0_1_rx_block_lock_3;
wire qsfp0_1_rx_block_lock_4;

wire qsfp0_2_rx_block_lock_1;
wire qsfp0_2_rx_block_lock_2;
wire qsfp0_2_rx_block_lock_3;
wire qsfp0_2_rx_block_lock_4;

wire qsfp0_1_mgt_refclk_0;

wire [7:0] gt0_txclkout;
wire gt0_txusrclk;

wire [7:0] gt0_rxclkout;
wire [7:0] gt0_rxusrclk;

wire gt0_reset_tx_done;
wire gt0_reset_rx_done;

wire [7:0] gt0_txprgdivresetdone;
wire [7:0] gt0_txpmaresetdone;
wire [7:0] gt0_rxprgdivresetdone;
wire [7:0] gt0_rxpmaresetdone;

wire gt0_tx_reset = ~((&gt0_txprgdivresetdone) & (&gt0_txpmaresetdone));
wire gt0_rx_reset = ~&gt0_rxpmaresetdone;

reg gt0_userclk_tx_active = 1'b0;
reg [7:0] gt0_userclk_rx_active = 1'b0;

IBUFDS_GTE4 ibufds_gte4_qsfp0_1_mgt_refclk_0_inst (
    .I             (qsfp0_1_mgt_refclk_0_p),
    .IB            (qsfp0_1_mgt_refclk_0_n),
    .CEB           (1'b0),
    .O             (qsfp0_1_mgt_refclk_0),
    .ODIV2         ()
);

// IBUFDS_GTE4 ibufds_gte4_qsfp0_2_mgt_refclk_0_inst (
//     .I             (qsfp0_2_mgt_refclk_0_p),
//     .IB            (qsfp0_2_mgt_refclk_0_n),
//     .CEB           (1'b0),
//     .O             (qsfp0_2_mgt_refclk_0),
//     .ODIV2         ()
// );

BUFG_GT bufg_gt0_tx_usrclk_inst (
    .CE      (1'b1),
    .CEMASK  (1'b0),
    .CLR     (gt0_tx_reset),
    .CLRMASK (1'b0),
    .DIV     (3'd0),
    .I       (gt0_txclkout[0]),
    .O       (gt0_txusrclk)
);

assign clk_156mhz_int0 = gt0_txusrclk;

always @(posedge gt0_txusrclk, posedge gt0_tx_reset) begin
    if (gt0_tx_reset) begin
        gt0_userclk_tx_active <= 1'b0;
    end else begin
        gt0_userclk_tx_active <= 1'b1;
    end
end

genvar n0;

generate

for (n0 = 0; n0 < 8; n0 = n0 + 1) begin

    BUFG_GT bufg_gt0_rx_usrclk_inst (
        .CE      (1'b1),
        .CEMASK  (1'b0),
        .CLR     (gt0_rx_reset),
        .CLRMASK (1'b0),
        .DIV     (3'd0),
        .I       (gt0_rxclkout[n0]),
        .O       (gt0_rxusrclk[n0])
    );

    always @(posedge gt0_rxusrclk[n0], posedge gt0_rx_reset) begin
        if (gt0_rx_reset) begin
            gt0_userclk_rx_active[n0] <= 1'b0;
        end else begin
            gt0_userclk_rx_active[n0] <= 1'b1;
        end
    end

end

endgenerate

sync_reset #(
    .N(4)
)
sync_reset_156mhz_inst0 (
    .clk(clk_156mhz_int0),
    .rst(~gt0_reset_tx_done),
    .out(rst_156mhz_int0)
);

wire [5:0] qsfp0_1_gt_txheader_1;
wire [63:0] qsfp0_1_gt_txdata_1;
wire qsfp0_1_gt_rxgearboxslip_1;
wire [5:0] qsfp0_1_gt_rxheader_1;
wire [1:0] qsfp0_1_gt_rxheadervalid_1;
wire [63:0] qsfp0_1_gt_rxdata_1;
wire [1:0] qsfp0_1_gt_rxdatavalid_1;

wire [5:0] qsfp0_1_gt_txheader_2;
wire [63:0] qsfp0_1_gt_txdata_2;
wire qsfp0_1_gt_rxgearboxslip_2;
wire [5:0] qsfp0_1_gt_rxheader_2;
wire [1:0] qsfp0_1_gt_rxheadervalid_2;
wire [63:0] qsfp0_1_gt_rxdata_2;
wire [1:0] qsfp0_1_gt_rxdatavalid_2;

wire [5:0] qsfp0_1_gt_txheader_3;
wire [63:0] qsfp0_1_gt_txdata_3;
wire qsfp0_1_gt_rxgearboxslip_3;
wire [5:0] qsfp0_1_gt_rxheader_3;
wire [1:0] qsfp0_1_gt_rxheadervalid_3;
wire [63:0] qsfp0_1_gt_rxdata_3;
wire [1:0] qsfp0_1_gt_rxdatavalid_3;

wire [5:0] qsfp0_1_gt_txheader_4;
wire [63:0] qsfp0_1_gt_txdata_4;
wire qsfp0_1_gt_rxgearboxslip_4;
wire [5:0] qsfp0_1_gt_rxheader_4;
wire [1:0] qsfp0_1_gt_rxheadervalid_4;
wire [63:0] qsfp0_1_gt_rxdata_4;
wire [1:0] qsfp0_1_gt_rxdatavalid_4;

wire [5:0] qsfp0_2_gt_txheader_1;
wire [63:0] qsfp0_2_gt_txdata_1;
wire qsfp0_2_gt_rxgearboxslip_1;
wire [5:0] qsfp0_2_gt_rxheader_1;
wire [1:0] qsfp0_2_gt_rxheadervalid_1;
wire [63:0] qsfp0_2_gt_rxdata_1;
wire [1:0] qsfp0_2_gt_rxdatavalid_1;

wire [5:0] qsfp0_2_gt_txheader_2;
wire [63:0] qsfp0_2_gt_txdata_2;
wire qsfp0_2_gt_rxgearboxslip_2;
wire [5:0] qsfp0_2_gt_rxheader_2;
wire [1:0] qsfp0_2_gt_rxheadervalid_2;
wire [63:0] qsfp0_2_gt_rxdata_2;
wire [1:0] qsfp0_2_gt_rxdatavalid_2;

wire [5:0] qsfp0_2_gt_txheader_3;
wire [63:0] qsfp0_2_gt_txdata_3;
wire qsfp0_2_gt_rxgearboxslip_3;
wire [5:0] qsfp0_2_gt_rxheader_3;
wire [1:0] qsfp0_2_gt_rxheadervalid_3;
wire [63:0] qsfp0_2_gt_rxdata_3;
wire [1:0] qsfp0_2_gt_rxdatavalid_3;

wire [5:0] qsfp0_2_gt_txheader_4;
wire [63:0] qsfp0_2_gt_txdata_4;
wire qsfp0_2_gt_rxgearboxslip_4;
wire [5:0] qsfp0_2_gt_rxheader_4;
wire [1:0] qsfp0_2_gt_rxheadervalid_4;
wire [63:0] qsfp0_2_gt_rxdata_4;
wire [1:0] qsfp0_2_gt_rxdatavalid_4;

gtwizard_ultrascale_0
qsfp_gty_inst0 (
    .gtwiz_userclk_tx_active_in(&gt0_userclk_tx_active),
    .gtwiz_userclk_rx_active_in(&gt0_userclk_rx_active),

    .gtwiz_reset_clk_freerun_in(clk_125mhz_int0),
    .gtwiz_reset_all_in(rst_125mhz_int0),

    .gtwiz_reset_tx_pll_and_datapath_in(1'b0),
    .gtwiz_reset_tx_datapath_in(1'b0),

    .gtwiz_reset_rx_pll_and_datapath_in(1'b0),
    .gtwiz_reset_rx_datapath_in(1'b0),

    .gtwiz_reset_rx_cdr_stable_out(),

    .gtwiz_reset_tx_done_out(gt0_reset_tx_done),
    .gtwiz_reset_rx_done_out(gt0_reset_rx_done),

    .gtrefclk00_in({2{qsfp0_1_mgt_refclk_0}}),

    .qpll0outclk_out(),
    .qpll0outrefclk_out(),

    .gtyrxn_in({qsfp0_1_rx4_n, qsfp0_1_rx2_n, qsfp0_1_rx3_n, qsfp0_1_rx1_n, qsfp0_2_rx4_n, qsfp0_2_rx3_n, qsfp0_2_rx2_n, qsfp0_2_rx1_n}),
    .gtyrxp_in({qsfp0_1_rx4_p, qsfp0_1_rx2_p, qsfp0_1_rx3_p, qsfp0_1_rx1_p, qsfp0_2_rx4_p, qsfp0_2_rx3_p, qsfp0_2_rx2_p, qsfp0_2_rx1_p}),

    .rxusrclk_in({gt0_rxusrclk[3], gt0_rxusrclk[1], gt0_rxusrclk[2], gt0_rxusrclk[0], gt0_rxusrclk[7:4]}),
    .rxusrclk2_in({gt0_rxusrclk[3], gt0_rxusrclk[1], gt0_rxusrclk[2], gt0_rxusrclk[0], gt0_rxusrclk[7:4]}),

    .gtwiz_userdata_tx_in({qsfp0_1_gt_txdata_4, qsfp0_1_gt_txdata_2, qsfp0_1_gt_txdata_3, qsfp0_1_gt_txdata_1, qsfp0_2_gt_txdata_4, qsfp0_2_gt_txdata_3, qsfp0_2_gt_txdata_2, qsfp0_2_gt_txdata_1}),
    .txheader_in({qsfp0_1_gt_txheader_4, qsfp0_1_gt_txheader_2, qsfp0_1_gt_txheader_3, qsfp0_1_gt_txheader_1, qsfp0_2_gt_txheader_4, qsfp0_2_gt_txheader_3, qsfp0_2_gt_txheader_2, qsfp0_2_gt_txheader_1}),
    .txsequence_in({8{1'b0}}),

    .txusrclk_in({8{gt0_txusrclk}}),
    .txusrclk2_in({8{gt0_txusrclk}}),

    .gtpowergood_out(),

    .gtytxn_out({qsfp0_1_tx4_n, qsfp0_1_tx2_n, qsfp0_1_tx3_n, qsfp0_1_tx1_n, qsfp0_2_tx4_n, qsfp0_2_tx3_n, qsfp0_2_tx2_n, qsfp0_2_tx1_n}),
    .gtytxp_out({qsfp0_1_tx4_p, qsfp0_1_tx2_p, qsfp0_1_tx3_p, qsfp0_1_tx1_p, qsfp0_2_tx4_p, qsfp0_2_tx3_p, qsfp0_2_tx2_p, qsfp0_2_tx1_p}),

    .rxgearboxslip_in({qsfp0_1_gt_rxgearboxslip_4, qsfp0_1_gt_rxgearboxslip_2, qsfp0_1_gt_rxgearboxslip_3, qsfp0_1_gt_rxgearboxslip_1, qsfp0_2_gt_rxgearboxslip_4, qsfp0_2_gt_rxgearboxslip_3, qsfp0_2_gt_rxgearboxslip_2, qsfp0_2_gt_rxgearboxslip_1}),
    .gtwiz_userdata_rx_out({qsfp0_1_gt_rxdata_4, qsfp0_1_gt_rxdata_2, qsfp0_1_gt_rxdata_3, qsfp0_1_gt_rxdata_1, qsfp0_2_gt_rxdata_4, qsfp0_2_gt_rxdata_3, qsfp0_2_gt_rxdata_2, qsfp0_2_gt_rxdata_1}),
    .rxdatavalid_out({qsfp0_1_gt_rxdatavalid_4, qsfp0_1_gt_rxdatavalid_2, qsfp0_1_gt_rxdatavalid_3, qsfp0_1_gt_rxdatavalid_1, qsfp0_2_gt_rxdatavalid_4, qsfp0_2_gt_rxdatavalid_3, qsfp0_2_gt_rxdatavalid_2, qsfp0_2_gt_rxdatavalid_1}),
    .rxheader_out({qsfp0_1_gt_rxheader_4, qsfp0_1_gt_rxheader_2, qsfp0_1_gt_rxheader_3, qsfp0_1_gt_rxheader_1, qsfp0_2_gt_rxheader_4, qsfp0_2_gt_rxheader_3, qsfp0_2_gt_rxheader_2, qsfp0_2_gt_rxheader_1}),
    .rxheadervalid_out({qsfp0_1_gt_rxheadervalid_4, qsfp0_1_gt_rxheadervalid_2, qsfp0_1_gt_rxheadervalid_3, qsfp0_1_gt_rxheadervalid_1, qsfp0_2_gt_rxheadervalid_4, qsfp0_2_gt_rxheadervalid_3, qsfp0_2_gt_rxheadervalid_2, qsfp0_2_gt_rxheadervalid_1}),
    .rxoutclk_out({gt0_rxclkout[3], gt0_rxclkout[1], gt0_rxclkout[2], gt0_rxclkout[0], gt0_rxclkout[7:4]}),
    .rxpmaresetdone_out(gt0_rxpmaresetdone),
    .rxprgdivresetdone_out(gt0_rxprgdivresetdone),
    .rxstartofseq_out(),

    .txoutclk_out(gt0_txclkout),
    .txpmaresetdone_out(gt0_txpmaresetdone),
    .txprgdivresetdone_out(gt0_txprgdivresetdone)
);

assign qsfp0_1_tx_clk_1_int = clk_156mhz_int0;
assign qsfp0_1_tx_rst_1_int = rst_156mhz_int0;

assign qsfp0_1_rx_clk_1_int = gt0_rxusrclk[0];

sync_reset #(
    .N(4)
)
qsfp0_1_rx_rst_1_reset_sync_inst (
    .clk(qsfp0_1_rx_clk_1_int),
    .rst(~gt0_reset_rx_done),
    .out(qsfp0_1_rx_rst_1_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp0_1_phy_1_inst (
    .tx_clk(qsfp0_1_tx_clk_1_int),
    .tx_rst(qsfp0_1_tx_rst_1_int),
    .rx_clk(qsfp0_1_rx_clk_1_int),
    .rx_rst(qsfp0_1_rx_rst_1_int),
    .xgmii_txd(qsfp0_1_txd_1_int),
    .xgmii_txc(qsfp0_1_txc_1_int),
    .xgmii_rxd(qsfp0_1_rxd_1_int),
    .xgmii_rxc(qsfp0_1_rxc_1_int),
    .serdes_tx_data(qsfp0_1_gt_txdata_1),
    .serdes_tx_hdr(qsfp0_1_gt_txheader_1),
    .serdes_rx_data(qsfp0_1_gt_rxdata_1),
    .serdes_rx_hdr(qsfp0_1_gt_rxheader_1),
    .serdes_rx_bitslip(qsfp0_1_gt_rxgearboxslip_1),
    .rx_error_count(qsfp0_1_rx_error_count_1_int),
    .rx_block_lock(qsfp0_1_rx_block_lock_1),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp0_1_tx_prbs31_enable_1_int),
    .rx_prbs31_enable(qsfp0_1_rx_prbs31_enable_1_int)
);

assign qsfp0_1_tx_clk_2_int = clk_156mhz_int0;
assign qsfp0_1_tx_rst_2_int = rst_156mhz_int0;

assign qsfp0_1_rx_clk_2_int = gt0_rxusrclk[1];

sync_reset #(
    .N(4)
)
qsfp0_1_rx_rst_2_reset_sync_inst (
    .clk(qsfp0_1_rx_clk_2_int),
    .rst(~gt0_reset_rx_done),
    .out(qsfp0_1_rx_rst_2_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp0_1_phy_2_inst (
    .tx_clk(qsfp0_1_tx_clk_2_int),
    .tx_rst(qsfp0_1_tx_rst_2_int),
    .rx_clk(qsfp0_1_rx_clk_2_int),
    .rx_rst(qsfp0_1_rx_rst_2_int),
    .xgmii_txd(qsfp0_1_txd_2_int),
    .xgmii_txc(qsfp0_1_txc_2_int),
    .xgmii_rxd(qsfp0_1_rxd_2_int),
    .xgmii_rxc(qsfp0_1_rxc_2_int),
    .serdes_tx_data(qsfp0_1_gt_txdata_2),
    .serdes_tx_hdr(qsfp0_1_gt_txheader_2),
    .serdes_rx_data(qsfp0_1_gt_rxdata_2),
    .serdes_rx_hdr(qsfp0_1_gt_rxheader_2),
    .serdes_rx_bitslip(qsfp0_1_gt_rxgearboxslip_2),
    .rx_error_count(qsfp0_1_rx_error_count_2_int),
    .rx_block_lock(qsfp0_1_rx_block_lock_2),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp0_1_tx_prbs31_enable_2_int),
    .rx_prbs31_enable(qsfp0_1_rx_prbs31_enable_2_int)
);

assign qsfp0_1_tx_clk_3_int = clk_156mhz_int0;
assign qsfp0_1_tx_rst_3_int = rst_156mhz_int0;

assign qsfp0_1_rx_clk_3_int = gt0_rxusrclk[2];

sync_reset #(
    .N(4)
)
qsfp0_1_rx_rst_3_reset_sync_inst (
    .clk(qsfp0_1_rx_clk_3_int),
    .rst(~gt0_reset_rx_done),
    .out(qsfp0_1_rx_rst_3_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp0_1_phy_3_inst (
    .tx_clk(qsfp0_1_tx_clk_3_int),
    .tx_rst(qsfp0_1_tx_rst_3_int),
    .rx_clk(qsfp0_1_rx_clk_3_int),
    .rx_rst(qsfp0_1_rx_rst_3_int),
    .xgmii_txd(qsfp0_1_txd_3_int),
    .xgmii_txc(qsfp0_1_txc_3_int),
    .xgmii_rxd(qsfp0_1_rxd_3_int),
    .xgmii_rxc(qsfp0_1_rxc_3_int),
    .serdes_tx_data(qsfp0_1_gt_txdata_3),
    .serdes_tx_hdr(qsfp0_1_gt_txheader_3),
    .serdes_rx_data(qsfp0_1_gt_rxdata_3),
    .serdes_rx_hdr(qsfp0_1_gt_rxheader_3),
    .serdes_rx_bitslip(qsfp0_1_gt_rxgearboxslip_3),
    .rx_error_count(qsfp0_1_rx_error_count_3_int),
    .rx_block_lock(qsfp0_1_rx_block_lock_3),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp0_1_tx_prbs31_enable_3_int),
    .rx_prbs31_enable(qsfp0_1_rx_prbs31_enable_3_int)
);

assign qsfp0_1_tx_clk_4_int = clk_156mhz_int0;
assign qsfp0_1_tx_rst_4_int = rst_156mhz_int0;

assign qsfp0_1_rx_clk_4_int = gt0_rxusrclk[3];

sync_reset #(
    .N(4)
)
qsfp0_1_rx_rst_4_reset_sync_inst (
    .clk(qsfp0_1_rx_clk_4_int),
    .rst(~gt0_reset_rx_done),
    .out(qsfp0_1_rx_rst_4_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp0_1_phy_4_inst (
    .tx_clk(qsfp0_1_tx_clk_4_int),
    .tx_rst(qsfp0_1_tx_rst_4_int),
    .rx_clk(qsfp0_1_rx_clk_4_int),
    .rx_rst(qsfp0_1_rx_rst_4_int),
    .xgmii_txd(qsfp0_1_txd_4_int),
    .xgmii_txc(qsfp0_1_txc_4_int),
    .xgmii_rxd(qsfp0_1_rxd_4_int),
    .xgmii_rxc(qsfp0_1_rxc_4_int),
    .serdes_tx_data(qsfp0_1_gt_txdata_4),
    .serdes_tx_hdr(qsfp0_1_gt_txheader_4),
    .serdes_rx_data(qsfp0_1_gt_rxdata_4),
    .serdes_rx_hdr(qsfp0_1_gt_rxheader_4),
    .serdes_rx_bitslip(qsfp0_1_gt_rxgearboxslip_4),
    .rx_error_count(qsfp0_1_rx_error_count_4_int),
    .rx_block_lock(qsfp0_1_rx_block_lock_4),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp0_1_tx_prbs31_enable_4_int),
    .rx_prbs31_enable(qsfp0_1_rx_prbs31_enable_4_int)
);

assign qsfp0_2_tx_clk_1_int = clk_156mhz_int0;
assign qsfp0_2_tx_rst_1_int = rst_156mhz_int0;

assign qsfp0_2_rx_clk_1_int = gt0_rxusrclk[4];

sync_reset #(
    .N(4)
)
qsfp0_2_rx_rst_1_reset_sync_inst (
    .clk(qsfp0_2_rx_clk_1_int),
    .rst(~gt0_reset_rx_done),
    .out(qsfp0_2_rx_rst_1_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp0_2_phy_1_inst (
    .tx_clk(qsfp0_2_tx_clk_1_int),
    .tx_rst(qsfp0_2_tx_rst_1_int),
    .rx_clk(qsfp0_2_rx_clk_1_int),
    .rx_rst(qsfp0_2_rx_rst_1_int),
    .xgmii_txd(qsfp0_2_txd_1_int),
    .xgmii_txc(qsfp0_2_txc_1_int),
    .xgmii_rxd(qsfp0_2_rxd_1_int),
    .xgmii_rxc(qsfp0_2_rxc_1_int),
    .serdes_tx_data(qsfp0_2_gt_txdata_1),
    .serdes_tx_hdr(qsfp0_2_gt_txheader_1),
    .serdes_rx_data(qsfp0_2_gt_rxdata_1),
    .serdes_rx_hdr(qsfp0_2_gt_rxheader_1),
    .serdes_rx_bitslip(qsfp0_2_gt_rxgearboxslip_1),
    .rx_error_count(qsfp0_2_rx_error_count_1_int),
    .rx_block_lock(qsfp0_2_rx_block_lock_1),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp0_2_tx_prbs31_enable_1_int),
    .rx_prbs31_enable(qsfp0_2_rx_prbs31_enable_1_int)
);

assign qsfp0_2_tx_clk_2_int = clk_156mhz_int0;
assign qsfp0_2_tx_rst_2_int = rst_156mhz_int0;

assign qsfp0_2_rx_clk_2_int = gt0_rxusrclk[5];

sync_reset #(
    .N(4)
)
qsfp0_2_rx_rst_2_reset_sync_inst (
    .clk(qsfp0_2_rx_clk_2_int),
    .rst(~gt0_reset_rx_done),
    .out(qsfp0_2_rx_rst_2_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp0_2_phy_2_inst (
    .tx_clk(qsfp0_2_tx_clk_2_int),
    .tx_rst(qsfp0_2_tx_rst_2_int),
    .rx_clk(qsfp0_2_rx_clk_2_int),
    .rx_rst(qsfp0_2_rx_rst_2_int),
    .xgmii_txd(qsfp0_2_txd_2_int),
    .xgmii_txc(qsfp0_2_txc_2_int),
    .xgmii_rxd(qsfp0_2_rxd_2_int),
    .xgmii_rxc(qsfp0_2_rxc_2_int),
    .serdes_tx_data(qsfp0_2_gt_txdata_2),
    .serdes_tx_hdr(qsfp0_2_gt_txheader_2),
    .serdes_rx_data(qsfp0_2_gt_rxdata_2),
    .serdes_rx_hdr(qsfp0_2_gt_rxheader_2),
    .serdes_rx_bitslip(qsfp0_2_gt_rxgearboxslip_2),
    .rx_error_count(qsfp0_2_rx_error_count_2_int),
    .rx_block_lock(qsfp0_2_rx_block_lock_2),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp0_2_tx_prbs31_enable_2_int),
    .rx_prbs31_enable(qsfp0_2_rx_prbs31_enable_2_int)
);

assign qsfp0_2_tx_clk_3_int = clk_156mhz_int0;
assign qsfp0_2_tx_rst_3_int = rst_156mhz_int0;

assign qsfp0_2_rx_clk_3_int = gt0_rxusrclk[6];

sync_reset #(
    .N(4)
)
qsfp0_2_rx_rst_3_reset_sync_inst (
    .clk(qsfp0_2_rx_clk_3_int),
    .rst(~gt0_reset_rx_done),
    .out(qsfp0_2_rx_rst_3_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp0_2_phy_3_inst (
    .tx_clk(qsfp0_2_tx_clk_3_int),
    .tx_rst(qsfp0_2_tx_rst_3_int),
    .rx_clk(qsfp0_2_rx_clk_3_int),
    .rx_rst(qsfp0_2_rx_rst_3_int),
    .xgmii_txd(qsfp0_2_txd_3_int),
    .xgmii_txc(qsfp0_2_txc_3_int),
    .xgmii_rxd(qsfp0_2_rxd_3_int),
    .xgmii_rxc(qsfp0_2_rxc_3_int),
    .serdes_tx_data(qsfp0_2_gt_txdata_3),
    .serdes_tx_hdr(qsfp0_2_gt_txheader_3),
    .serdes_rx_data(qsfp0_2_gt_rxdata_3),
    .serdes_rx_hdr(qsfp0_2_gt_rxheader_3),
    .serdes_rx_bitslip(qsfp0_2_gt_rxgearboxslip_3),
    .rx_error_count(qsfp0_2_rx_error_count_3_int),
    .rx_block_lock(qsfp0_2_rx_block_lock_3),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp0_2_tx_prbs31_enable_3_int),
    .rx_prbs31_enable(qsfp0_2_rx_prbs31_enable_3_int)
);

assign qsfp0_2_tx_clk_4_int = clk_156mhz_int0;
assign qsfp0_2_tx_rst_4_int = rst_156mhz_int0;

assign qsfp0_2_rx_clk_4_int = gt0_rxusrclk[7];

sync_reset #(
    .N(4)
)
qsfp0_2_rx_rst_4_reset_sync_inst (
    .clk(qsfp0_2_rx_clk_4_int),
    .rst(~gt0_reset_rx_done),
    .out(qsfp0_2_rx_rst_4_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp0_2_phy_4_inst (
    .tx_clk(qsfp0_2_tx_clk_4_int),
    .tx_rst(qsfp0_2_tx_rst_4_int),
    .rx_clk(qsfp0_2_rx_clk_4_int),
    .rx_rst(qsfp0_2_rx_rst_4_int),
    .xgmii_txd(qsfp0_2_txd_4_int),
    .xgmii_txc(qsfp0_2_txc_4_int),
    .xgmii_rxd(qsfp0_2_rxd_4_int),
    .xgmii_rxc(qsfp0_2_rxc_4_int),
    .serdes_tx_data(qsfp0_2_gt_txdata_4),
    .serdes_tx_hdr(qsfp0_2_gt_txheader_4),
    .serdes_rx_data(qsfp0_2_gt_rxdata_4),
    .serdes_rx_hdr(qsfp0_2_gt_rxheader_4),
    .serdes_rx_bitslip(qsfp0_2_gt_rxgearboxslip_4),
    .rx_error_count(qsfp0_2_rx_error_count_4_int),
    .rx_block_lock(qsfp0_2_rx_block_lock_4),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp0_2_tx_prbs31_enable_4_int),
    .rx_prbs31_enable(qsfp0_2_rx_prbs31_enable_4_int)
);

fpga_core #(
    .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
    .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
    .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
    .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
    .AXIS_PCIE_CQ_USER_WIDTH(AXIS_PCIE_CQ_USER_WIDTH),
    .AXIS_PCIE_CC_USER_WIDTH(AXIS_PCIE_CC_USER_WIDTH),
    .RQ_SEQ_NUM_WIDTH(RQ_SEQ_NUM_WIDTH),
    .BAR0_APERTURE(BAR0_APERTURE)
)
core_inst0 (
    /*
     * Clock: 250 MHz
     * Synchronous reset
     */
    .clk_250mhz(pcie0_user_clk),
    .rst_250mhz(pcie0_user_reset),

    /*
     * GPIO
     */
    .led(),

    /*
     * PCIe
     */
    .m_axis_rq_tdata(axis0_rq_tdata),
    .m_axis_rq_tkeep(axis0_rq_tkeep),
    .m_axis_rq_tlast(axis0_rq_tlast),
    .m_axis_rq_tready(axis0_rq_tready),
    .m_axis_rq_tuser(axis0_rq_tuser),
    .m_axis_rq_tvalid(axis0_rq_tvalid),

    .s_axis_rc_tdata(axis0_rc_tdata),
    .s_axis_rc_tkeep(axis0_rc_tkeep),
    .s_axis_rc_tlast(axis0_rc_tlast),
    .s_axis_rc_tready(axis0_rc_tready),
    .s_axis_rc_tuser(axis0_rc_tuser),
    .s_axis_rc_tvalid(axis0_rc_tvalid),

    .s_axis_cq_tdata(axis0_cq_tdata),
    .s_axis_cq_tkeep(axis0_cq_tkeep),
    .s_axis_cq_tlast(axis0_cq_tlast),
    .s_axis_cq_tready(axis0_cq_tready),
    .s_axis_cq_tuser(axis0_cq_tuser),
    .s_axis_cq_tvalid(axis0_cq_tvalid),

    .m_axis_cc_tdata(axis0_cc_tdata),
    .m_axis_cc_tkeep(axis0_cc_tkeep),
    .m_axis_cc_tlast(axis0_cc_tlast),
    .m_axis_cc_tready(axis0_cc_tready),
    .m_axis_cc_tuser(axis0_cc_tuser),
    .m_axis_cc_tvalid(axis0_cc_tvalid),

    .s_axis_rq_seq_num_0(pcie0_rq_seq_num0),
    .s_axis_rq_seq_num_valid_0(pcie0_rq_seq_num_vld0),
    .s_axis_rq_seq_num_1(pcie0_rq_seq_num1),
    .s_axis_rq_seq_num_valid_1(pcie0_rq_seq_num_vld1),

    .pcie_tfc_nph_av(pcie0_tfc_nph_av),
    .pcie_tfc_npd_av(pcie0_tfc_npd_av),

    .cfg_max_payload(cfg0_max_payload),
    .cfg_max_read_req(cfg0_max_read_req),

    .cfg_mgmt_addr(cfg0_mgmt_addr),
    .cfg_mgmt_function_number(cfg0_mgmt_function_number),
    .cfg_mgmt_write(cfg0_mgmt_write),
    .cfg_mgmt_write_data(cfg0_mgmt_write_data),
    .cfg_mgmt_byte_enable(cfg0_mgmt_byte_enable),
    .cfg_mgmt_read(cfg0_mgmt_read),
    .cfg_mgmt_read_data(cfg0_mgmt_read_data),
    .cfg_mgmt_read_write_done(cfg0_mgmt_read_write_done),

    .cfg_fc_ph(cfg0_fc_ph),
    .cfg_fc_pd(cfg0_fc_pd),
    .cfg_fc_nph(cfg0_fc_nph),
    .cfg_fc_npd(cfg0_fc_npd),
    .cfg_fc_cplh(cfg0_fc_cplh),
    .cfg_fc_cpld(cfg0_fc_cpld),
    .cfg_fc_sel(cfg0_fc_sel),

    .cfg_interrupt_msi_enable(cfg0_interrupt_msi_enable),
    .cfg_interrupt_msi_mmenable(cfg0_interrupt_msi_mmenable),
    .cfg_interrupt_msi_mask_update(cfg0_interrupt_msi_mask_update),
    .cfg_interrupt_msi_data(cfg0_interrupt_msi_data),
    .cfg_interrupt_msi_select(cfg0_interrupt_msi_select),
    .cfg_interrupt_msi_int(cfg0_interrupt_msi_int),
    .cfg_interrupt_msi_pending_status(cfg0_interrupt_msi_pending_status),
    .cfg_interrupt_msi_pending_status_data_enable(cfg0_interrupt_msi_pending_status_data_enable),
    .cfg_interrupt_msi_pending_status_function_num(cfg0_interrupt_msi_pending_status_function_num),
    .cfg_interrupt_msi_sent(cfg0_interrupt_msi_sent),
    .cfg_interrupt_msi_fail(cfg0_interrupt_msi_fail),
    .cfg_interrupt_msi_attr(cfg0_interrupt_msi_attr),
    .cfg_interrupt_msi_tph_present(cfg0_interrupt_msi_tph_present),
    .cfg_interrupt_msi_tph_type(cfg0_interrupt_msi_tph_type),
    .cfg_interrupt_msi_tph_st_tag(cfg0_interrupt_msi_tph_st_tag),
    .cfg_interrupt_msi_function_number(cfg0_interrupt_msi_function_number),

    .status_error_cor(status0_error_cor),
    .status_error_uncor(status0_error_uncor),

    /*
     * Ethernet: QSFP28
     */
    .qsfp1_tx_clk_1(qsfp0_1_tx_clk_1_int),
    .qsfp1_tx_rst_1(qsfp0_1_tx_rst_1_int),
    .qsfp1_txd_1(qsfp0_1_txd_1_int),
    .qsfp1_txc_1(qsfp0_1_txc_1_int),
    .qsfp1_tx_prbs31_enable_1(qsfp0_1_tx_prbs31_enable_1_int),
    .qsfp1_rx_clk_1(qsfp0_1_rx_clk_1_int),
    .qsfp1_rx_rst_1(qsfp0_1_rx_rst_1_int),
    .qsfp1_rxd_1(qsfp0_1_rxd_1_int),
    .qsfp1_rxc_1(qsfp0_1_rxc_1_int),
    .qsfp1_rx_prbs31_enable_1(qsfp0_1_rx_prbs31_enable_1_int),
    .qsfp1_rx_error_count_1(qsfp0_1_rx_error_count_1_int),
    .qsfp1_tx_clk_2(qsfp0_1_tx_clk_2_int),
    .qsfp1_tx_rst_2(qsfp0_1_tx_rst_2_int),
    .qsfp1_txd_2(qsfp0_1_txd_2_int),
    .qsfp1_txc_2(qsfp0_1_txc_2_int),
    .qsfp1_tx_prbs31_enable_2(qsfp0_1_tx_prbs31_enable_2_int),
    .qsfp1_rx_clk_2(qsfp0_1_rx_clk_2_int),
    .qsfp1_rx_rst_2(qsfp0_1_rx_rst_2_int),
    .qsfp1_rxd_2(qsfp0_1_rxd_2_int),
    .qsfp1_rxc_2(qsfp0_1_rxc_2_int),
    .qsfp1_rx_prbs31_enable_2(qsfp0_1_rx_prbs31_enable_2_int),
    .qsfp1_rx_error_count_2(qsfp0_1_rx_error_count_2_int),
    .qsfp1_tx_clk_3(qsfp0_1_tx_clk_3_int),
    .qsfp1_tx_rst_3(qsfp0_1_tx_rst_3_int),
    .qsfp1_txd_3(qsfp0_1_txd_3_int),
    .qsfp1_txc_3(qsfp0_1_txc_3_int),
    .qsfp1_tx_prbs31_enable_3(qsfp0_1_tx_prbs31_enable_3_int),
    .qsfp1_rx_clk_3(qsfp0_1_rx_clk_3_int),
    .qsfp1_rx_rst_3(qsfp0_1_rx_rst_3_int),
    .qsfp1_rxd_3(qsfp0_1_rxd_3_int),
    .qsfp1_rxc_3(qsfp0_1_rxc_3_int),
    .qsfp1_rx_prbs31_enable_3(qsfp0_1_rx_prbs31_enable_3_int),
    .qsfp1_rx_error_count_3(qsfp0_1_rx_error_count_3_int),
    .qsfp1_tx_clk_4(qsfp0_1_tx_clk_4_int),
    .qsfp1_tx_rst_4(qsfp0_1_tx_rst_4_int),
    .qsfp1_txd_4(qsfp0_1_txd_4_int),
    .qsfp1_txc_4(qsfp0_1_txc_4_int),
    .qsfp1_tx_prbs31_enable_4(qsfp0_1_tx_prbs31_enable_4_int),
    .qsfp1_rx_clk_4(qsfp0_1_rx_clk_4_int),
    .qsfp1_rx_rst_4(qsfp0_1_rx_rst_4_int),
    .qsfp1_rxd_4(qsfp0_1_rxd_4_int),
    .qsfp1_rxc_4(qsfp0_1_rxc_4_int),
    .qsfp1_rx_prbs31_enable_4(qsfp0_1_rx_prbs31_enable_4_int),
    .qsfp1_rx_error_count_4(qsfp0_1_rx_error_count_4_int),

    .qsfp2_tx_clk_1(qsfp0_2_tx_clk_1_int),
    .qsfp2_tx_rst_1(qsfp0_2_tx_rst_1_int),
    .qsfp2_txd_1(qsfp0_2_txd_1_int),
    .qsfp2_txc_1(qsfp0_2_txc_1_int),
    .qsfp2_tx_prbs31_enable_1(qsfp0_2_tx_prbs31_enable_1_int),
    .qsfp2_rx_clk_1(qsfp0_2_rx_clk_1_int),
    .qsfp2_rx_rst_1(qsfp0_2_rx_rst_1_int),
    .qsfp2_rxd_1(qsfp0_2_rxd_1_int),
    .qsfp2_rxc_1(qsfp0_2_rxc_1_int),
    .qsfp2_rx_prbs31_enable_1(qsfp0_2_rx_prbs31_enable_1_int),
    .qsfp2_rx_error_count_1(qsfp0_2_rx_error_count_1_int),
    .qsfp2_tx_clk_2(qsfp0_2_tx_clk_2_int),
    .qsfp2_tx_rst_2(qsfp0_2_tx_rst_2_int),
    .qsfp2_txd_2(qsfp0_2_txd_2_int),
    .qsfp2_txc_2(qsfp0_2_txc_2_int),
    .qsfp2_tx_prbs31_enable_2(qsfp0_2_tx_prbs31_enable_2_int),
    .qsfp2_rx_clk_2(qsfp0_2_rx_clk_2_int),
    .qsfp2_rx_rst_2(qsfp0_2_rx_rst_2_int),
    .qsfp2_rxd_2(qsfp0_2_rxd_2_int),
    .qsfp2_rxc_2(qsfp0_2_rxc_2_int),
    .qsfp2_rx_prbs31_enable_2(qsfp0_2_rx_prbs31_enable_2_int),
    .qsfp2_rx_error_count_2(qsfp0_2_rx_error_count_2_int),
    .qsfp2_tx_clk_3(qsfp0_2_tx_clk_3_int),
    .qsfp2_tx_rst_3(qsfp0_2_tx_rst_3_int),
    .qsfp2_txd_3(qsfp0_2_txd_3_int),
    .qsfp2_txc_3(qsfp0_2_txc_3_int),
    .qsfp2_tx_prbs31_enable_3(qsfp0_2_tx_prbs31_enable_3_int),
    .qsfp2_rx_clk_3(qsfp0_2_rx_clk_3_int),
    .qsfp2_rx_rst_3(qsfp0_2_rx_rst_3_int),
    .qsfp2_rxd_3(qsfp0_2_rxd_3_int),
    .qsfp2_rxc_3(qsfp0_2_rxc_3_int),
    .qsfp2_rx_prbs31_enable_3(qsfp0_2_rx_prbs31_enable_3_int),
    .qsfp2_rx_error_count_3(qsfp0_2_rx_error_count_3_int),
    .qsfp2_tx_clk_4(qsfp0_2_tx_clk_4_int),
    .qsfp2_tx_rst_4(qsfp0_2_tx_rst_4_int),
    .qsfp2_txd_4(qsfp0_2_txd_4_int),
    .qsfp2_txc_4(qsfp0_2_txc_4_int),
    .qsfp2_tx_prbs31_enable_4(qsfp0_2_tx_prbs31_enable_4_int),
    .qsfp2_rx_clk_4(qsfp0_2_rx_clk_4_int),
    .qsfp2_rx_rst_4(qsfp0_2_rx_rst_4_int),
    .qsfp2_rxd_4(qsfp0_2_rxd_4_int),
    .qsfp2_rxc_4(qsfp0_2_rxc_4_int),
    .qsfp2_rx_prbs31_enable_4(qsfp0_2_rx_prbs31_enable_4_int),
    .qsfp2_rx_error_count_4(qsfp0_2_rx_error_count_4_int),

    /*
     * QSPI flash
     */
    .fpga_boot(),
    .qspi_clk(),
    .qspi_0_dq_i(4'h0),
    .qspi_0_dq_o(),
    .qspi_0_dq_oe(),
    .qspi_0_cs()
);

/*
 ****************************** FPGA0 D1 ********************************
 */

// PCIe
wire pcie1_sys_clk;
wire pcie1_sys_clk_gt;

IBUFDS_GTE4 #(
    .REFCLK_HROW_CK_SEL(2'b00)
)
ibufds_gte4_pcie_mgt_refclk_inst1 (
    .I             (pcie1_refclk_0_p),
    .IB            (pcie1_refclk_0_n),
    .CEB           (1'b0),
    .O             (pcie1_sys_clk_gt),
    .ODIV2         (pcie1_sys_clk)
);

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis1_rq_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis1_rq_tkeep;
wire                               axis1_rq_tlast;
wire                               axis1_rq_tready;
wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis1_rq_tuser;
wire                               axis1_rq_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis1_rc_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis1_rc_tkeep;
wire                               axis1_rc_tlast;
wire                               axis1_rc_tready;
wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis1_rc_tuser;
wire                               axis1_rc_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis1_cq_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis1_cq_tkeep;
wire                               axis1_cq_tlast;
wire                               axis1_cq_tready;
wire [AXIS_PCIE_CQ_USER_WIDTH-1:0] axis1_cq_tuser;
wire                               axis1_cq_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis1_cc_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis1_cc_tkeep;
wire                               axis1_cc_tlast;
wire                               axis1_cc_tready;
wire [AXIS_PCIE_CC_USER_WIDTH-1:0] axis1_cc_tuser;
wire                               axis1_cc_tvalid;

wire [RQ_SEQ_NUM_WIDTH-1:0]        pcie1_rq_seq_num0;
wire                               pcie1_rq_seq_num_vld0;
wire [RQ_SEQ_NUM_WIDTH-1:0]        pcie1_rq_seq_num1;
wire                               pcie1_rq_seq_num_vld1;

wire [3:0] pcie1_tfc_nph_av;
wire [3:0] pcie1_tfc_npd_av;

wire [2:0] cfg1_max_payload;
wire [2:0] cfg1_max_read_req;

wire [9:0]  cfg1_mgmt_addr;
wire [7:0]  cfg1_mgmt_function_number;
wire        cfg1_mgmt_write;
wire [31:0] cfg1_mgmt_write_data;
wire [3:0]  cfg1_mgmt_byte_enable;
wire        cfg1_mgmt_read;
wire [31:0] cfg1_mgmt_read_data;
wire        cfg1_mgmt_read_write_done;

wire [7:0]  cfg1_fc_ph;
wire [11:0] cfg1_fc_pd;
wire [7:0]  cfg1_fc_nph;
wire [11:0] cfg1_fc_npd;
wire [7:0]  cfg1_fc_cplh;
wire [11:0] cfg1_fc_cpld;
wire [2:0]  cfg1_fc_sel;

wire [3:0]  cfg1_interrupt_msi_enable;
wire [11:0] cfg1_interrupt_msi_mmenable;
wire        cfg1_interrupt_msi_mask_update;
wire [31:0] cfg1_interrupt_msi_data;
wire [3:0]  cfg1_interrupt_msi_select;
wire [31:0] cfg1_interrupt_msi_int;
wire [31:0] cfg1_interrupt_msi_pending_status;
wire        cfg1_interrupt_msi_pending_status_data_enable;
wire [3:0]  cfg1_interrupt_msi_pending_status_function_num;
wire        cfg1_interrupt_msi_sent;
wire        cfg1_interrupt_msi_fail;
wire [2:0]  cfg1_interrupt_msi_attr;
wire        cfg1_interrupt_msi_tph_present;
wire [1:0]  cfg1_interrupt_msi_tph_type;
wire [8:0]  cfg1_interrupt_msi_tph_st_tag;
wire [3:0]  cfg1_interrupt_msi_function_number;

wire status1_error_cor;
wire status1_error_uncor;

// extra register for pcie1_user_reset signal
wire pcie1_user_reset_int;
(* shreg_extract = "no" *)
reg pcie1_user_reset_reg_1 = 1'b1;
(* shreg_extract = "no" *)
reg pcie1_user_reset_reg_2 = 1'b1;

always @(posedge pcie1_user_clk) begin
    pcie1_user_reset_reg_1 <= pcie1_user_reset_int;
    pcie1_user_reset_reg_2 <= pcie1_user_reset_reg_1;
end

assign pcie1_user_reset = pcie1_user_reset_reg_2;

pcie4_uscale_plus_1
pcie4_uscale_plus_inst1 (
    .pci_exp_txn(pcie1_tx_n),
    .pci_exp_txp(pcie1_tx_p),
    .pci_exp_rxn(pcie1_rx_n),
    .pci_exp_rxp(pcie1_rx_p),
    .user_clk(pcie1_user_clk),
    .user_reset(pcie1_user_reset_int),
    .user_lnk_up(),

    .s_axis_rq_tdata(axis1_rq_tdata),
    .s_axis_rq_tkeep(axis1_rq_tkeep),
    .s_axis_rq_tlast(axis1_rq_tlast),
    .s_axis_rq_tready(axis1_rq_tready),
    .s_axis_rq_tuser(axis1_rq_tuser),
    .s_axis_rq_tvalid(axis1_rq_tvalid),

    .m_axis_rc_tdata(axis1_rc_tdata),
    .m_axis_rc_tkeep(axis1_rc_tkeep),
    .m_axis_rc_tlast(axis1_rc_tlast),
    .m_axis_rc_tready(axis1_rc_tready),
    .m_axis_rc_tuser(axis1_rc_tuser),
    .m_axis_rc_tvalid(axis1_rc_tvalid),

    .m_axis_cq_tdata(axis1_cq_tdata),
    .m_axis_cq_tkeep(axis1_cq_tkeep),
    .m_axis_cq_tlast(axis1_cq_tlast),
    .m_axis_cq_tready(axis1_cq_tready),
    .m_axis_cq_tuser(axis1_cq_tuser),
    .m_axis_cq_tvalid(axis1_cq_tvalid),

    .s_axis_cc_tdata(axis1_cc_tdata),
    .s_axis_cc_tkeep(axis1_cc_tkeep),
    .s_axis_cc_tlast(axis1_cc_tlast),
    .s_axis_cc_tready(axis1_cc_tready),
    .s_axis_cc_tuser(axis1_cc_tuser),
    .s_axis_cc_tvalid(axis1_cc_tvalid),

    .pcie_rq_seq_num0(pcie1_rq_seq_num0),
    .pcie_rq_seq_num_vld0(pcie1_rq_seq_num_vld0),
    .pcie_rq_seq_num1(pcie1_rq_seq_num1),
    .pcie_rq_seq_num_vld1(pcie1_rq_seq_num_vld1),
    .pcie_rq_tag0(),
    .pcie_rq_tag1(),
    .pcie_rq_tag_av(),
    .pcie_rq_tag_vld0(),
    .pcie_rq_tag_vld1(),

    .pcie_tfc_nph_av(pcie1_tfc_nph_av),
    .pcie_tfc_npd_av(pcie1_tfc_npd_av),

    .pcie_cq_np_req(1'b1),
    .pcie_cq_np_req_count(),

    .cfg_phy_link_down(),
    .cfg_phy_link_status(),
    .cfg_negotiated_width(),
    .cfg_current_speed(),
    .cfg_max_payload(cfg1_max_payload),
    .cfg_max_read_req(cfg1_max_read_req),
    .cfg_function_status(),
    .cfg_function_power_state(),
    .cfg_vf_status(),
    .cfg_vf_power_state(),
    .cfg_link_power_state(),

    .cfg_mgmt_addr(cfg1_mgmt_addr),
    .cfg_mgmt_function_number(cfg1_mgmt_function_number),
    .cfg_mgmt_write(cfg1_mgmt_write),
    .cfg_mgmt_write_data(cfg1_mgmt_write_data),
    .cfg_mgmt_byte_enable(cfg1_mgmt_byte_enable),
    .cfg_mgmt_read(cfg1_mgmt_read),
    .cfg_mgmt_read_data(cfg1_mgmt_read_data),
    .cfg_mgmt_read_write_done(cfg1_mgmt_read_write_done),
    .cfg_mgmt_debug_access(1'b0),

    .cfg_err_cor_out(),
    .cfg_err_nonfatal_out(),
    .cfg_err_fatal_out(),
    .cfg_local_error_valid(),
    .cfg_local_error_out(),
    .cfg_ltssm_state(),
    .cfg_rx_pm_state(),
    .cfg_tx_pm_state(),
    .cfg_rcb_status(),
    .cfg_obff_enable(),
    .cfg_pl_status_change(),
    .cfg_tph_requester_enable(),
    .cfg_tph_st_mode(),
    .cfg_vf_tph_requester_enable(),
    .cfg_vf_tph_st_mode(),

    .cfg_msg_received(),
    .cfg_msg_received_data(),
    .cfg_msg_received_type(),
    .cfg_msg_transmit(1'b0),
    .cfg_msg_transmit_type(3'd0),
    .cfg_msg_transmit_data(32'd0),
    .cfg_msg_transmit_done(),

    .cfg_fc_ph(cfg1_fc_ph),
    .cfg_fc_pd(cfg1_fc_pd),
    .cfg_fc_nph(cfg1_fc_nph),
    .cfg_fc_npd(cfg1_fc_npd),
    .cfg_fc_cplh(cfg1_fc_cplh),
    .cfg_fc_cpld(cfg1_fc_cpld),
    .cfg_fc_sel(cfg1_fc_sel),

    .cfg_dsn(64'd0),

    .cfg_power_state_change_ack(1'b1),
    .cfg_power_state_change_interrupt(),

    .cfg_err_cor_in(status1_error_cor),
    .cfg_err_uncor_in(status1_error_uncor),
    .cfg_flr_in_process(),
    .cfg_flr_done(4'd0),
    .cfg_vf_flr_in_process(),
    .cfg_vf_flr_func_num(8'd0),
    .cfg_vf_flr_done(8'd0),

    .cfg_link_training_enable(1'b1),

    .cfg_interrupt_int(4'd0),
    .cfg_interrupt_pending(4'd0),
    .cfg_interrupt_sent(),
    .cfg_interrupt_msi_enable(cfg1_interrupt_msi_enable),
    .cfg_interrupt_msi_mmenable(cfg1_interrupt_msi_mmenable),
    .cfg_interrupt_msi_mask_update(cfg1_interrupt_msi_mask_update),
    .cfg_interrupt_msi_data(cfg1_interrupt_msi_data),
    .cfg_interrupt_msi_select(cfg1_interrupt_msi_select),
    .cfg_interrupt_msi_int(cfg1_interrupt_msi_int),
    .cfg_interrupt_msi_pending_status(cfg1_interrupt_msi_pending_status),
    .cfg_interrupt_msi_pending_status_data_enable(cfg1_interrupt_msi_pending_status_data_enable),
    .cfg_interrupt_msi_pending_status_function_num(cfg1_interrupt_msi_pending_status_function_num),
    .cfg_interrupt_msi_sent(cfg1_interrupt_msi_sent),
    .cfg_interrupt_msi_fail(cfg1_interrupt_msi_fail),
    .cfg_interrupt_msi_attr(cfg1_interrupt_msi_attr),
    .cfg_interrupt_msi_tph_present(cfg1_interrupt_msi_tph_present),
    .cfg_interrupt_msi_tph_type(cfg1_interrupt_msi_tph_type),
    .cfg_interrupt_msi_tph_st_tag(cfg1_interrupt_msi_tph_st_tag),
    .cfg_interrupt_msi_function_number(cfg1_interrupt_msi_function_number),

    .cfg_pm_aspm_l1_entry_reject(1'b0),
    .cfg_pm_aspm_tx_l0s_entry_disable(1'b0),

    .cfg_hot_reset_out(),

    .cfg_config_space_enable(1'b1),
    .cfg_req_pm_transition_l23_ready(1'b0),
    .cfg_hot_reset_in(1'b0),

    .cfg_ds_port_number(8'd0),
    .cfg_ds_bus_number(8'd0),
    .cfg_ds_device_number(5'd0),

    .sys_clk(pcie1_sys_clk),
    .sys_clk_gt(pcie1_sys_clk_gt),
    .sys_reset(pcie1_reset_n),

    .phy_rdy_out()
);

// XGMII 10G PHY
wire        qsfp1_1_tx_clk_1_int;
wire        qsfp1_1_tx_rst_1_int;
wire [63:0] qsfp1_1_txd_1_int;
wire [7:0]  qsfp1_1_txc_1_int;
wire        qsfp1_1_tx_prbs31_enable_1_int;
wire        qsfp1_1_rx_clk_1_int;
wire        qsfp1_1_rx_rst_1_int;
wire [63:0] qsfp1_1_rxd_1_int;
wire [7:0]  qsfp1_1_rxc_1_int;
wire        qsfp1_1_rx_prbs31_enable_1_int;
wire [6:0]  qsfp1_1_rx_error_count_1_int;
wire        qsfp1_1_tx_clk_2_int;
wire        qsfp1_1_tx_rst_2_int;
wire [63:0] qsfp1_1_txd_2_int;
wire [7:0]  qsfp1_1_txc_2_int;
wire        qsfp1_1_tx_prbs31_enable_2_int;
wire        qsfp1_1_rx_clk_2_int;
wire        qsfp1_1_rx_rst_2_int;
wire [63:0] qsfp1_1_rxd_2_int;
wire [7:0]  qsfp1_1_rxc_2_int;
wire        qsfp1_1_rx_prbs31_enable_2_int;
wire [6:0]  qsfp1_1_rx_error_count_2_int;
wire        qsfp1_1_tx_clk_3_int;
wire        qsfp1_1_tx_rst_3_int;
wire [63:0] qsfp1_1_txd_3_int;
wire [7:0]  qsfp1_1_txc_3_int;
wire        qsfp1_1_tx_prbs31_enable_3_int;
wire        qsfp1_1_rx_clk_3_int;
wire        qsfp1_1_rx_rst_3_int;
wire [63:0] qsfp1_1_rxd_3_int;
wire [7:0]  qsfp1_1_rxc_3_int;
wire        qsfp1_1_rx_prbs31_enable_3_int;
wire [6:0]  qsfp1_1_rx_error_count_3_int;
wire        qsfp1_1_tx_clk_4_int;
wire        qsfp1_1_tx_rst_4_int;
wire [63:0] qsfp1_1_txd_4_int;
wire [7:0]  qsfp1_1_txc_4_int;
wire        qsfp1_1_tx_prbs31_enable_4_int;
wire        qsfp1_1_rx_clk_4_int;
wire        qsfp1_1_rx_rst_4_int;
wire [63:0] qsfp1_1_rxd_4_int;
wire [7:0]  qsfp1_1_rxc_4_int;
wire        qsfp1_1_rx_prbs31_enable_4_int;
wire [6:0]  qsfp1_1_rx_error_count_4_int;

wire        qsfp1_2_tx_clk_1_int;
wire        qsfp1_2_tx_rst_1_int;
wire [63:0] qsfp1_2_txd_1_int;
wire [7:0]  qsfp1_2_txc_1_int;
wire        qsfp1_2_tx_prbs31_enable_1_int;
wire        qsfp1_2_rx_clk_1_int;
wire        qsfp1_2_rx_rst_1_int;
wire [63:0] qsfp1_2_rxd_1_int;
wire [7:0]  qsfp1_2_rxc_1_int;
wire        qsfp1_2_rx_prbs31_enable_1_int;
wire [6:0]  qsfp1_2_rx_error_count_1_int;
wire        qsfp1_2_tx_clk_2_int;
wire        qsfp1_2_tx_rst_2_int;
wire [63:0] qsfp1_2_txd_2_int;
wire [7:0]  qsfp1_2_txc_2_int;
wire        qsfp1_2_tx_prbs31_enable_2_int;
wire        qsfp1_2_rx_clk_2_int;
wire        qsfp1_2_rx_rst_2_int;
wire [63:0] qsfp1_2_rxd_2_int;
wire [7:0]  qsfp1_2_rxc_2_int;
wire        qsfp1_2_rx_prbs31_enable_2_int;
wire [6:0]  qsfp1_2_rx_error_count_2_int;
wire        qsfp1_2_tx_clk_3_int;
wire        qsfp1_2_tx_rst_3_int;
wire [63:0] qsfp1_2_txd_3_int;
wire [7:0]  qsfp1_2_txc_3_int;
wire        qsfp1_2_tx_prbs31_enable_3_int;
wire        qsfp1_2_rx_clk_3_int;
wire        qsfp1_2_rx_rst_3_int;
wire [63:0] qsfp1_2_rxd_3_int;
wire [7:0]  qsfp1_2_rxc_3_int;
wire        qsfp1_2_rx_prbs31_enable_3_int;
wire [6:0]  qsfp1_2_rx_error_count_3_int;
wire        qsfp1_2_tx_clk_4_int;
wire        qsfp1_2_tx_rst_4_int;
wire [63:0] qsfp1_2_txd_4_int;
wire [7:0]  qsfp1_2_txc_4_int;
wire        qsfp1_2_tx_prbs31_enable_4_int;
wire        qsfp1_2_rx_clk_4_int;
wire        qsfp1_2_rx_rst_4_int;
wire [63:0] qsfp1_2_rxd_4_int;
wire [7:0]  qsfp1_2_rxc_4_int;
wire        qsfp1_2_rx_prbs31_enable_4_int;
wire [6:0]  qsfp1_2_rx_error_count_4_int;

wire qsfp1_1_rx_block_lock_1;
wire qsfp1_1_rx_block_lock_2;
wire qsfp1_1_rx_block_lock_3;
wire qsfp1_1_rx_block_lock_4;

wire qsfp1_2_rx_block_lock_1;
wire qsfp1_2_rx_block_lock_2;
wire qsfp1_2_rx_block_lock_3;
wire qsfp1_2_rx_block_lock_4;

wire qsfp1_1_mgt_refclk_0;

wire [7:0] gt1_txclkout;
wire gt1_txusrclk;

wire [7:0] gt1_rxclkout;
wire [7:0] gt1_rxusrclk;

wire gt1_reset_tx_done;
wire gt1_reset_rx_done;

wire [7:0] gt1_txprgdivresetdone;
wire [7:0] gt1_txpmaresetdone;
wire [7:0] gt1_rxprgdivresetdone;
wire [7:0] gt1_rxpmaresetdone;

wire gt1_tx_reset = ~((&gt1_txprgdivresetdone) & (&gt1_txpmaresetdone));
wire gt1_rx_reset = ~&gt1_rxpmaresetdone;

reg gt1_userclk_tx_active = 1'b0;
reg [7:0] gt1_userclk_rx_active = 1'b0;

IBUFDS_GTE4 ibufds_gte4_qsfp1_1_mgt_refclk_0_inst (
    .I             (qsfp1_1_mgt_refclk_0_p),
    .IB            (qsfp1_1_mgt_refclk_0_n),
    .CEB           (1'b0),
    .O             (qsfp1_1_mgt_refclk_0),
    .ODIV2         ()
);

// IBUFDS_GTE4 ibufds_gte4_qsfp1_2_mgt_refclk_0_inst (
//     .I             (qsfp1_2_mgt_refclk_0_p),
//     .IB            (qsfp1_2_mgt_refclk_0_n),
//     .CEB           (1'b0),
//     .O             (qsfp1_2_mgt_refclk_0),
//     .ODIV2         ()
// );

BUFG_GT bufg_gt1_tx_usrclk_inst (
    .CE      (1'b1),
    .CEMASK  (1'b0),
    .CLR     (gt1_tx_reset),
    .CLRMASK (1'b0),
    .DIV     (3'd0),
    .I       (gt1_txclkout[0]),
    .O       (gt1_txusrclk)
);

assign clk_156mhz_int1 = gt1_txusrclk;

always @(posedge gt1_txusrclk, posedge gt1_tx_reset) begin
    if (gt1_tx_reset) begin
        gt1_userclk_tx_active <= 1'b0;
    end else begin
        gt1_userclk_tx_active <= 1'b1;
    end
end

genvar n1;

generate

for (n1 = 0; n1 < 8; n1 = n1 + 1) begin

    BUFG_GT bufg_gt1_rx_usrclk_inst (
        .CE      (1'b1),
        .CEMASK  (1'b0),
        .CLR     (gt1_rx_reset),
        .CLRMASK (1'b0),
        .DIV     (3'd0),
        .I       (gt1_rxclkout[n1]),
        .O       (gt1_rxusrclk[n1])
    );

    always @(posedge gt1_rxusrclk[n1], posedge gt1_rx_reset) begin
        if (gt1_rx_reset) begin
            gt1_userclk_rx_active[n1] <= 1'b0;
        end else begin
            gt1_userclk_rx_active[n1] <= 1'b1;
        end
    end

end

endgenerate

sync_reset #(
    .N(4)
)
sync_reset_156mhz_inst1 (
    .clk(clk_156mhz_int1),
    .rst(~gt1_reset_tx_done),
    .out(rst_156mhz_int1)
);

wire [5:0] qsfp1_1_gt_txheader_1;
wire [63:0] qsfp1_1_gt_txdata_1;
wire qsfp1_1_gt_rxgearboxslip_1;
wire [5:0] qsfp1_1_gt_rxheader_1;
wire [1:0] qsfp1_1_gt_rxheadervalid_1;
wire [63:0] qsfp1_1_gt_rxdata_1;
wire [1:0] qsfp1_1_gt_rxdatavalid_1;

wire [5:0] qsfp1_1_gt_txheader_2;
wire [63:0] qsfp1_1_gt_txdata_2;
wire qsfp1_1_gt_rxgearboxslip_2;
wire [5:0] qsfp1_1_gt_rxheader_2;
wire [1:0] qsfp1_1_gt_rxheadervalid_2;
wire [63:0] qsfp1_1_gt_rxdata_2;
wire [1:0] qsfp1_1_gt_rxdatavalid_2;

wire [5:0] qsfp1_1_gt_txheader_3;
wire [63:0] qsfp1_1_gt_txdata_3;
wire qsfp1_1_gt_rxgearboxslip_3;
wire [5:0] qsfp1_1_gt_rxheader_3;
wire [1:0] qsfp1_1_gt_rxheadervalid_3;
wire [63:0] qsfp1_1_gt_rxdata_3;
wire [1:0] qsfp1_1_gt_rxdatavalid_3;

wire [5:0] qsfp1_1_gt_txheader_4;
wire [63:0] qsfp1_1_gt_txdata_4;
wire qsfp1_1_gt_rxgearboxslip_4;
wire [5:0] qsfp1_1_gt_rxheader_4;
wire [1:0] qsfp1_1_gt_rxheadervalid_4;
wire [63:0] qsfp1_1_gt_rxdata_4;
wire [1:0] qsfp1_1_gt_rxdatavalid_4;

wire [5:0] qsfp1_2_gt_txheader_1;
wire [63:0] qsfp1_2_gt_txdata_1;
wire qsfp1_2_gt_rxgearboxslip_1;
wire [5:0] qsfp1_2_gt_rxheader_1;
wire [1:0] qsfp1_2_gt_rxheadervalid_1;
wire [63:0] qsfp1_2_gt_rxdata_1;
wire [1:0] qsfp1_2_gt_rxdatavalid_1;

wire [5:0] qsfp1_2_gt_txheader_2;
wire [63:0] qsfp1_2_gt_txdata_2;
wire qsfp1_2_gt_rxgearboxslip_2;
wire [5:0] qsfp1_2_gt_rxheader_2;
wire [1:0] qsfp1_2_gt_rxheadervalid_2;
wire [63:0] qsfp1_2_gt_rxdata_2;
wire [1:0] qsfp1_2_gt_rxdatavalid_2;

wire [5:0] qsfp1_2_gt_txheader_3;
wire [63:0] qsfp1_2_gt_txdata_3;
wire qsfp1_2_gt_rxgearboxslip_3;
wire [5:0] qsfp1_2_gt_rxheader_3;
wire [1:0] qsfp1_2_gt_rxheadervalid_3;
wire [63:0] qsfp1_2_gt_rxdata_3;
wire [1:0] qsfp1_2_gt_rxdatavalid_3;

wire [5:0] qsfp1_2_gt_txheader_4;
wire [63:0] qsfp1_2_gt_txdata_4;
wire qsfp1_2_gt_rxgearboxslip_4;
wire [5:0] qsfp1_2_gt_rxheader_4;
wire [1:0] qsfp1_2_gt_rxheadervalid_4;
wire [63:0] qsfp1_2_gt_rxdata_4;
wire [1:0] qsfp1_2_gt_rxdatavalid_4;

gtwizard_ultrascale_1
qsfp_gty_inst1 (
    .gtwiz_userclk_tx_active_in(&gt1_userclk_tx_active),
    .gtwiz_userclk_rx_active_in(&gt1_userclk_rx_active),

    .gtwiz_reset_clk_freerun_in(clk_125mhz_int1),
    .gtwiz_reset_all_in(rst_125mhz_int1),

    .gtwiz_reset_tx_pll_and_datapath_in(1'b0),
    .gtwiz_reset_tx_datapath_in(1'b0),

    .gtwiz_reset_rx_pll_and_datapath_in(1'b0),
    .gtwiz_reset_rx_datapath_in(1'b0),

    .gtwiz_reset_rx_cdr_stable_out(),

    .gtwiz_reset_tx_done_out(gt1_reset_tx_done),
    .gtwiz_reset_rx_done_out(gt1_reset_rx_done),

    .gtrefclk00_in({2{qsfp1_1_mgt_refclk_0}}),

    .qpll0outclk_out(),
    .qpll0outrefclk_out(),

    .gtyrxn_in({qsfp1_1_rx4_n, qsfp1_1_rx2_n, qsfp1_1_rx3_n, qsfp1_1_rx1_n, qsfp1_2_rx2_n, qsfp1_2_rx1_n, qsfp1_2_rx4_n, qsfp1_2_rx3_n}),
    .gtyrxp_in({qsfp1_1_rx4_p, qsfp1_1_rx2_p, qsfp1_1_rx3_p, qsfp1_1_rx1_p, qsfp1_2_rx2_p, qsfp1_2_rx1_p, qsfp1_2_rx4_p, qsfp1_2_rx3_p}),

    .rxusrclk_in({gt1_rxusrclk[3], gt1_rxusrclk[1], gt1_rxusrclk[2], gt1_rxusrclk[0], gt1_rxusrclk[5:4], gt1_rxusrclk[7:6]}),
    .rxusrclk2_in({gt1_rxusrclk[3], gt1_rxusrclk[1], gt1_rxusrclk[2], gt1_rxusrclk[0], gt1_rxusrclk[5:4], gt1_rxusrclk[7:6]}),

    .gtwiz_userdata_tx_in({qsfp1_1_gt_txdata_4, qsfp1_1_gt_txdata_2, qsfp1_1_gt_txdata_3, qsfp1_1_gt_txdata_1, qsfp1_2_gt_txdata_2, qsfp1_2_gt_txdata_1, qsfp1_2_gt_txdata_4, qsfp1_2_gt_txdata_3}),
    .txheader_in({qsfp1_1_gt_txheader_4, qsfp1_1_gt_txheader_2, qsfp1_1_gt_txheader_3, qsfp1_1_gt_txheader_1, qsfp1_2_gt_txheader_2, qsfp1_2_gt_txheader_1, qsfp1_2_gt_txheader_4, qsfp1_2_gt_txheader_3}),
    .txsequence_in({8{1'b0}}),

    .txusrclk_in({8{gt1_txusrclk}}),
    .txusrclk2_in({8{gt1_txusrclk}}),

    .gtpowergood_out(),

    .gtytxn_out({qsfp1_1_tx4_n, qsfp1_1_tx2_n, qsfp1_1_tx3_n, qsfp1_1_tx1_n, qsfp1_2_tx2_n, qsfp1_2_tx1_n, qsfp1_2_tx4_n, qsfp1_2_tx3_n}),
    .gtytxp_out({qsfp1_1_tx4_p, qsfp1_1_tx2_p, qsfp1_1_tx3_p, qsfp1_1_tx1_p, qsfp1_2_tx2_p, qsfp1_2_tx1_p, qsfp1_2_tx4_p, qsfp1_2_tx3_p}),

    .rxgearboxslip_in({qsfp1_1_gt_rxgearboxslip_4, qsfp1_1_gt_rxgearboxslip_2, qsfp1_1_gt_rxgearboxslip_3, qsfp1_1_gt_rxgearboxslip_1, qsfp1_2_gt_rxgearboxslip_2, qsfp1_2_gt_rxgearboxslip_1, qsfp1_2_gt_rxgearboxslip_4, qsfp1_2_gt_rxgearboxslip_3}),
    .gtwiz_userdata_rx_out({qsfp1_1_gt_rxdata_4, qsfp1_1_gt_rxdata_2, qsfp1_1_gt_rxdata_3, qsfp1_1_gt_rxdata_1, qsfp1_2_gt_rxdata_2, qsfp1_2_gt_rxdata_1, qsfp1_2_gt_rxdata_4, qsfp1_2_gt_rxdata_3}),
    .rxdatavalid_out({qsfp1_1_gt_rxdatavalid_4, qsfp1_1_gt_rxdatavalid_2, qsfp1_1_gt_rxdatavalid_3, qsfp1_1_gt_rxdatavalid_1, qsfp1_2_gt_rxdatavalid_2, qsfp1_2_gt_rxdatavalid_1, qsfp1_2_gt_rxdatavalid_4, qsfp1_2_gt_rxdatavalid_3}),
    .rxheader_out({qsfp1_1_gt_rxheader_4, qsfp1_1_gt_rxheader_2, qsfp1_1_gt_rxheader_3, qsfp1_1_gt_rxheader_1, qsfp1_2_gt_rxheader_2, qsfp1_2_gt_rxheader_1, qsfp1_2_gt_rxheader_4, qsfp1_2_gt_rxheader_3}),
    .rxheadervalid_out({qsfp1_1_gt_rxheadervalid_4, qsfp1_1_gt_rxheadervalid_2, qsfp1_1_gt_rxheadervalid_3, qsfp1_1_gt_rxheadervalid_1, qsfp1_2_gt_rxheadervalid_2, qsfp1_2_gt_rxheadervalid_1, qsfp1_2_gt_rxheadervalid_4, qsfp1_2_gt_rxheadervalid_3}),
    .rxoutclk_out({gt1_rxclkout[3], gt1_rxclkout[1], gt1_rxclkout[2], gt1_rxclkout[0], gt1_rxclkout[5:4], gt1_rxclkout[7:6]}),
    .rxpmaresetdone_out(gt1_rxpmaresetdone),
    .rxprgdivresetdone_out(gt1_rxprgdivresetdone),
    .rxstartofseq_out(),

    .txoutclk_out(gt1_txclkout),
    .txpmaresetdone_out(gt1_txpmaresetdone),
    .txprgdivresetdone_out(gt1_txprgdivresetdone)
);

assign qsfp1_1_tx_clk_1_int = clk_156mhz_int1;
assign qsfp1_1_tx_rst_1_int = rst_156mhz_int1;

assign qsfp1_1_rx_clk_1_int = gt1_rxusrclk[0];

sync_reset #(
    .N(4)
)
qsfp1_1_rx_rst_1_reset_sync_inst (
    .clk(qsfp1_1_rx_clk_1_int),
    .rst(~gt1_reset_rx_done),
    .out(qsfp1_1_rx_rst_1_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp1_1_phy_1_inst (
    .tx_clk(qsfp1_1_tx_clk_1_int),
    .tx_rst(qsfp1_1_tx_rst_1_int),
    .rx_clk(qsfp1_1_rx_clk_1_int),
    .rx_rst(qsfp1_1_rx_rst_1_int),
    .xgmii_txd(qsfp1_1_txd_1_int),
    .xgmii_txc(qsfp1_1_txc_1_int),
    .xgmii_rxd(qsfp1_1_rxd_1_int),
    .xgmii_rxc(qsfp1_1_rxc_1_int),
    .serdes_tx_data(qsfp1_1_gt_txdata_1),
    .serdes_tx_hdr(qsfp1_1_gt_txheader_1),
    .serdes_rx_data(qsfp1_1_gt_rxdata_1),
    .serdes_rx_hdr(qsfp1_1_gt_rxheader_1),
    .serdes_rx_bitslip(qsfp1_1_gt_rxgearboxslip_1),
    .rx_error_count(qsfp1_1_rx_error_count_1_int),
    .rx_block_lock(qsfp1_1_rx_block_lock_1),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp1_1_tx_prbs31_enable_1_int),
    .rx_prbs31_enable(qsfp1_1_rx_prbs31_enable_1_int)
);

assign qsfp1_1_tx_clk_2_int = clk_156mhz_int1;
assign qsfp1_1_tx_rst_2_int = rst_156mhz_int1;

assign qsfp1_1_rx_clk_2_int = gt1_rxusrclk[1];

sync_reset #(
    .N(4)
)
qsfp1_1_rx_rst_2_reset_sync_inst (
    .clk(qsfp1_1_rx_clk_2_int),
    .rst(~gt1_reset_rx_done),
    .out(qsfp1_1_rx_rst_2_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp1_1_phy_2_inst (
    .tx_clk(qsfp1_1_tx_clk_2_int),
    .tx_rst(qsfp1_1_tx_rst_2_int),
    .rx_clk(qsfp1_1_rx_clk_2_int),
    .rx_rst(qsfp1_1_rx_rst_2_int),
    .xgmii_txd(qsfp1_1_txd_2_int),
    .xgmii_txc(qsfp1_1_txc_2_int),
    .xgmii_rxd(qsfp1_1_rxd_2_int),
    .xgmii_rxc(qsfp1_1_rxc_2_int),
    .serdes_tx_data(qsfp1_1_gt_txdata_2),
    .serdes_tx_hdr(qsfp1_1_gt_txheader_2),
    .serdes_rx_data(qsfp1_1_gt_rxdata_2),
    .serdes_rx_hdr(qsfp1_1_gt_rxheader_2),
    .serdes_rx_bitslip(qsfp1_1_gt_rxgearboxslip_2),
    .rx_error_count(qsfp1_1_rx_error_count_2_int),
    .rx_block_lock(qsfp1_1_rx_block_lock_2),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp1_1_tx_prbs31_enable_2_int),
    .rx_prbs31_enable(qsfp1_1_rx_prbs31_enable_2_int)
);

assign qsfp1_1_tx_clk_3_int = clk_156mhz_int1;
assign qsfp1_1_tx_rst_3_int = rst_156mhz_int1;

assign qsfp1_1_rx_clk_3_int = gt1_rxusrclk[2];

sync_reset #(
    .N(4)
)
qsfp1_1_rx_rst_3_reset_sync_inst (
    .clk(qsfp1_1_rx_clk_3_int),
    .rst(~gt1_reset_rx_done),
    .out(qsfp1_1_rx_rst_3_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp1_1_phy_3_inst (
    .tx_clk(qsfp1_1_tx_clk_3_int),
    .tx_rst(qsfp1_1_tx_rst_3_int),
    .rx_clk(qsfp1_1_rx_clk_3_int),
    .rx_rst(qsfp1_1_rx_rst_3_int),
    .xgmii_txd(qsfp1_1_txd_3_int),
    .xgmii_txc(qsfp1_1_txc_3_int),
    .xgmii_rxd(qsfp1_1_rxd_3_int),
    .xgmii_rxc(qsfp1_1_rxc_3_int),
    .serdes_tx_data(qsfp1_1_gt_txdata_3),
    .serdes_tx_hdr(qsfp1_1_gt_txheader_3),
    .serdes_rx_data(qsfp1_1_gt_rxdata_3),
    .serdes_rx_hdr(qsfp1_1_gt_rxheader_3),
    .serdes_rx_bitslip(qsfp1_1_gt_rxgearboxslip_3),
    .rx_error_count(qsfp1_1_rx_error_count_3_int),
    .rx_block_lock(qsfp1_1_rx_block_lock_3),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp1_1_tx_prbs31_enable_3_int),
    .rx_prbs31_enable(qsfp1_1_rx_prbs31_enable_3_int)
);

assign qsfp1_1_tx_clk_4_int = clk_156mhz_int1;
assign qsfp1_1_tx_rst_4_int = rst_156mhz_int1;

assign qsfp1_1_rx_clk_4_int = gt1_rxusrclk[3];

sync_reset #(
    .N(4)
)
qsfp1_1_rx_rst_4_reset_sync_inst (
    .clk(qsfp1_1_rx_clk_4_int),
    .rst(~gt1_reset_rx_done),
    .out(qsfp1_1_rx_rst_4_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp1_1_phy_4_inst (
    .tx_clk(qsfp1_1_tx_clk_4_int),
    .tx_rst(qsfp1_1_tx_rst_4_int),
    .rx_clk(qsfp1_1_rx_clk_4_int),
    .rx_rst(qsfp1_1_rx_rst_4_int),
    .xgmii_txd(qsfp1_1_txd_4_int),
    .xgmii_txc(qsfp1_1_txc_4_int),
    .xgmii_rxd(qsfp1_1_rxd_4_int),
    .xgmii_rxc(qsfp1_1_rxc_4_int),
    .serdes_tx_data(qsfp1_1_gt_txdata_4),
    .serdes_tx_hdr(qsfp1_1_gt_txheader_4),
    .serdes_rx_data(qsfp1_1_gt_rxdata_4),
    .serdes_rx_hdr(qsfp1_1_gt_rxheader_4),
    .serdes_rx_bitslip(qsfp1_1_gt_rxgearboxslip_4),
    .rx_error_count(qsfp1_1_rx_error_count_4_int),
    .rx_block_lock(qsfp1_1_rx_block_lock_4),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp1_1_tx_prbs31_enable_4_int),
    .rx_prbs31_enable(qsfp1_1_rx_prbs31_enable_4_int)
);

assign qsfp1_2_tx_clk_1_int = clk_156mhz_int1;
assign qsfp1_2_tx_rst_1_int = rst_156mhz_int1;

assign qsfp1_2_rx_clk_1_int = gt1_rxusrclk[4];

sync_reset #(
    .N(4)
)
qsfp1_2_rx_rst_1_reset_sync_inst (
    .clk(qsfp1_2_rx_clk_1_int),
    .rst(~gt1_reset_rx_done),
    .out(qsfp1_2_rx_rst_1_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp1_2_phy_1_inst (
    .tx_clk(qsfp1_2_tx_clk_1_int),
    .tx_rst(qsfp1_2_tx_rst_1_int),
    .rx_clk(qsfp1_2_rx_clk_1_int),
    .rx_rst(qsfp1_2_rx_rst_1_int),
    .xgmii_txd(qsfp1_2_txd_1_int),
    .xgmii_txc(qsfp1_2_txc_1_int),
    .xgmii_rxd(qsfp1_2_rxd_1_int),
    .xgmii_rxc(qsfp1_2_rxc_1_int),
    .serdes_tx_data(qsfp1_2_gt_txdata_1),
    .serdes_tx_hdr(qsfp1_2_gt_txheader_1),
    .serdes_rx_data(qsfp1_2_gt_rxdata_1),
    .serdes_rx_hdr(qsfp1_2_gt_rxheader_1),
    .serdes_rx_bitslip(qsfp1_2_gt_rxgearboxslip_1),
    .rx_error_count(qsfp1_2_rx_error_count_1_int),
    .rx_block_lock(qsfp1_2_rx_block_lock_1),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp1_2_tx_prbs31_enable_1_int),
    .rx_prbs31_enable(qsfp1_2_rx_prbs31_enable_1_int)
);

assign qsfp1_2_tx_clk_2_int = clk_156mhz_int1;
assign qsfp1_2_tx_rst_2_int = rst_156mhz_int1;

assign qsfp1_2_rx_clk_2_int = gt1_rxusrclk[5];

sync_reset #(
    .N(4)
)
qsfp1_2_rx_rst_2_reset_sync_inst (
    .clk(qsfp1_2_rx_clk_2_int),
    .rst(~gt1_reset_rx_done),
    .out(qsfp1_2_rx_rst_2_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp1_2_phy_2_inst (
    .tx_clk(qsfp1_2_tx_clk_2_int),
    .tx_rst(qsfp1_2_tx_rst_2_int),
    .rx_clk(qsfp1_2_rx_clk_2_int),
    .rx_rst(qsfp1_2_rx_rst_2_int),
    .xgmii_txd(qsfp1_2_txd_2_int),
    .xgmii_txc(qsfp1_2_txc_2_int),
    .xgmii_rxd(qsfp1_2_rxd_2_int),
    .xgmii_rxc(qsfp1_2_rxc_2_int),
    .serdes_tx_data(qsfp1_2_gt_txdata_2),
    .serdes_tx_hdr(qsfp1_2_gt_txheader_2),
    .serdes_rx_data(qsfp1_2_gt_rxdata_2),
    .serdes_rx_hdr(qsfp1_2_gt_rxheader_2),
    .serdes_rx_bitslip(qsfp1_2_gt_rxgearboxslip_2),
    .rx_error_count(qsfp1_2_rx_error_count_2_int),
    .rx_block_lock(qsfp1_2_rx_block_lock_2),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp1_2_tx_prbs31_enable_2_int),
    .rx_prbs31_enable(qsfp1_2_rx_prbs31_enable_2_int)
);

assign qsfp1_2_tx_clk_3_int = clk_156mhz_int1;
assign qsfp1_2_tx_rst_3_int = rst_156mhz_int1;

assign qsfp1_2_rx_clk_3_int = gt1_rxusrclk[6];

sync_reset #(
    .N(4)
)
qsfp1_2_rx_rst_3_reset_sync_inst (
    .clk(qsfp1_2_rx_clk_3_int),
    .rst(~gt1_reset_rx_done),
    .out(qsfp1_2_rx_rst_3_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp1_2_phy_3_inst (
    .tx_clk(qsfp1_2_tx_clk_3_int),
    .tx_rst(qsfp1_2_tx_rst_3_int),
    .rx_clk(qsfp1_2_rx_clk_3_int),
    .rx_rst(qsfp1_2_rx_rst_3_int),
    .xgmii_txd(qsfp1_2_txd_3_int),
    .xgmii_txc(qsfp1_2_txc_3_int),
    .xgmii_rxd(qsfp1_2_rxd_3_int),
    .xgmii_rxc(qsfp1_2_rxc_3_int),
    .serdes_tx_data(qsfp1_2_gt_txdata_3),
    .serdes_tx_hdr(qsfp1_2_gt_txheader_3),
    .serdes_rx_data(qsfp1_2_gt_rxdata_3),
    .serdes_rx_hdr(qsfp1_2_gt_rxheader_3),
    .serdes_rx_bitslip(qsfp1_2_gt_rxgearboxslip_3),
    .rx_error_count(qsfp1_2_rx_error_count_3_int),
    .rx_block_lock(qsfp1_2_rx_block_lock_3),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp1_2_tx_prbs31_enable_3_int),
    .rx_prbs31_enable(qsfp1_2_rx_prbs31_enable_3_int)
);

assign qsfp1_2_tx_clk_4_int = clk_156mhz_int1;
assign qsfp1_2_tx_rst_4_int = rst_156mhz_int1;

assign qsfp1_2_rx_clk_4_int = gt1_rxusrclk[7];

sync_reset #(
    .N(4)
)
qsfp1_2_rx_rst_4_reset_sync_inst (
    .clk(qsfp1_2_rx_clk_4_int),
    .rst(~gt1_reset_rx_done),
    .out(qsfp1_2_rx_rst_4_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp1_2_phy_4_inst (
    .tx_clk(qsfp1_2_tx_clk_4_int),
    .tx_rst(qsfp1_2_tx_rst_4_int),
    .rx_clk(qsfp1_2_rx_clk_4_int),
    .rx_rst(qsfp1_2_rx_rst_4_int),
    .xgmii_txd(qsfp1_2_txd_4_int),
    .xgmii_txc(qsfp1_2_txc_4_int),
    .xgmii_rxd(qsfp1_2_rxd_4_int),
    .xgmii_rxc(qsfp1_2_rxc_4_int),
    .serdes_tx_data(qsfp1_2_gt_txdata_4),
    .serdes_tx_hdr(qsfp1_2_gt_txheader_4),
    .serdes_rx_data(qsfp1_2_gt_rxdata_4),
    .serdes_rx_hdr(qsfp1_2_gt_rxheader_4),
    .serdes_rx_bitslip(qsfp1_2_gt_rxgearboxslip_4),
    .rx_error_count(qsfp1_2_rx_error_count_4_int),
    .rx_block_lock(qsfp1_2_rx_block_lock_4),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp1_2_tx_prbs31_enable_4_int),
    .rx_prbs31_enable(qsfp1_2_rx_prbs31_enable_4_int)
);

fpga_core #(
    .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
    .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
    .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
    .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
    .AXIS_PCIE_CQ_USER_WIDTH(AXIS_PCIE_CQ_USER_WIDTH),
    .AXIS_PCIE_CC_USER_WIDTH(AXIS_PCIE_CC_USER_WIDTH),
    .RQ_SEQ_NUM_WIDTH(RQ_SEQ_NUM_WIDTH),
    .BAR0_APERTURE(BAR0_APERTURE)
)
core_inst1 (
    /*
     * Clock: 250 MHz
     * Synchronous reset
     */
    .clk_250mhz(pcie1_user_clk),
    .rst_250mhz(pcie1_user_reset),

    /*
     * GPIO
     */
    .led(user_led),

    /*
     * PCIe
     */
    .m_axis_rq_tdata(axis1_rq_tdata),
    .m_axis_rq_tkeep(axis1_rq_tkeep),
    .m_axis_rq_tlast(axis1_rq_tlast),
    .m_axis_rq_tready(axis1_rq_tready),
    .m_axis_rq_tuser(axis1_rq_tuser),
    .m_axis_rq_tvalid(axis1_rq_tvalid),

    .s_axis_rc_tdata(axis1_rc_tdata),
    .s_axis_rc_tkeep(axis1_rc_tkeep),
    .s_axis_rc_tlast(axis1_rc_tlast),
    .s_axis_rc_tready(axis1_rc_tready),
    .s_axis_rc_tuser(axis1_rc_tuser),
    .s_axis_rc_tvalid(axis1_rc_tvalid),

    .s_axis_cq_tdata(axis1_cq_tdata),
    .s_axis_cq_tkeep(axis1_cq_tkeep),
    .s_axis_cq_tlast(axis1_cq_tlast),
    .s_axis_cq_tready(axis1_cq_tready),
    .s_axis_cq_tuser(axis1_cq_tuser),
    .s_axis_cq_tvalid(axis1_cq_tvalid),

    .m_axis_cc_tdata(axis1_cc_tdata),
    .m_axis_cc_tkeep(axis1_cc_tkeep),
    .m_axis_cc_tlast(axis1_cc_tlast),
    .m_axis_cc_tready(axis1_cc_tready),
    .m_axis_cc_tuser(axis1_cc_tuser),
    .m_axis_cc_tvalid(axis1_cc_tvalid),

    .s_axis_rq_seq_num_0(pcie1_rq_seq_num0),
    .s_axis_rq_seq_num_valid_0(pcie1_rq_seq_num_vld0),
    .s_axis_rq_seq_num_1(pcie1_rq_seq_num1),
    .s_axis_rq_seq_num_valid_1(pcie1_rq_seq_num_vld1),

    .pcie_tfc_nph_av(pcie1_tfc_nph_av),
    .pcie_tfc_npd_av(pcie1_tfc_npd_av),

    .cfg_max_payload(cfg1_max_payload),
    .cfg_max_read_req(cfg1_max_read_req),

    .cfg_mgmt_addr(cfg1_mgmt_addr),
    .cfg_mgmt_function_number(cfg1_mgmt_function_number),
    .cfg_mgmt_write(cfg1_mgmt_write),
    .cfg_mgmt_write_data(cfg1_mgmt_write_data),
    .cfg_mgmt_byte_enable(cfg1_mgmt_byte_enable),
    .cfg_mgmt_read(cfg1_mgmt_read),
    .cfg_mgmt_read_data(cfg1_mgmt_read_data),
    .cfg_mgmt_read_write_done(cfg1_mgmt_read_write_done),

    .cfg_fc_ph(cfg1_fc_ph),
    .cfg_fc_pd(cfg1_fc_pd),
    .cfg_fc_nph(cfg1_fc_nph),
    .cfg_fc_npd(cfg1_fc_npd),
    .cfg_fc_cplh(cfg1_fc_cplh),
    .cfg_fc_cpld(cfg1_fc_cpld),
    .cfg_fc_sel(cfg1_fc_sel),

    .cfg_interrupt_msi_enable(cfg1_interrupt_msi_enable),
    .cfg_interrupt_msi_mmenable(cfg1_interrupt_msi_mmenable),
    .cfg_interrupt_msi_mask_update(cfg1_interrupt_msi_mask_update),
    .cfg_interrupt_msi_data(cfg1_interrupt_msi_data),
    .cfg_interrupt_msi_select(cfg1_interrupt_msi_select),
    .cfg_interrupt_msi_int(cfg1_interrupt_msi_int),
    .cfg_interrupt_msi_pending_status(cfg1_interrupt_msi_pending_status),
    .cfg_interrupt_msi_pending_status_data_enable(cfg1_interrupt_msi_pending_status_data_enable),
    .cfg_interrupt_msi_pending_status_function_num(cfg1_interrupt_msi_pending_status_function_num),
    .cfg_interrupt_msi_sent(cfg1_interrupt_msi_sent),
    .cfg_interrupt_msi_fail(cfg1_interrupt_msi_fail),
    .cfg_interrupt_msi_attr(cfg1_interrupt_msi_attr),
    .cfg_interrupt_msi_tph_present(cfg1_interrupt_msi_tph_present),
    .cfg_interrupt_msi_tph_type(cfg1_interrupt_msi_tph_type),
    .cfg_interrupt_msi_tph_st_tag(cfg1_interrupt_msi_tph_st_tag),
    .cfg_interrupt_msi_function_number(cfg1_interrupt_msi_function_number),

    .status_error_cor(status1_error_cor),
    .status_error_uncor(status1_error_uncor),

    /*
     * Ethernet: QSFP28
     */
    .qsfp1_tx_clk_1(qsfp1_1_tx_clk_1_int),
    .qsfp1_tx_rst_1(qsfp1_1_tx_rst_1_int),
    .qsfp1_txd_1(qsfp1_1_txd_1_int),
    .qsfp1_txc_1(qsfp1_1_txc_1_int),
    .qsfp1_tx_prbs31_enable_1(qsfp1_1_tx_prbs31_enable_1_int),
    .qsfp1_rx_clk_1(qsfp1_1_rx_clk_1_int),
    .qsfp1_rx_rst_1(qsfp1_1_rx_rst_1_int),
    .qsfp1_rxd_1(qsfp1_1_rxd_1_int),
    .qsfp1_rxc_1(qsfp1_1_rxc_1_int),
    .qsfp1_rx_prbs31_enable_1(qsfp1_1_rx_prbs31_enable_1_int),
    .qsfp1_rx_error_count_1(qsfp1_1_rx_error_count_1_int),
    .qsfp1_tx_clk_2(qsfp1_1_tx_clk_2_int),
    .qsfp1_tx_rst_2(qsfp1_1_tx_rst_2_int),
    .qsfp1_txd_2(qsfp1_1_txd_2_int),
    .qsfp1_txc_2(qsfp1_1_txc_2_int),
    .qsfp1_tx_prbs31_enable_2(qsfp1_1_tx_prbs31_enable_2_int),
    .qsfp1_rx_clk_2(qsfp1_1_rx_clk_2_int),
    .qsfp1_rx_rst_2(qsfp1_1_rx_rst_2_int),
    .qsfp1_rxd_2(qsfp1_1_rxd_2_int),
    .qsfp1_rxc_2(qsfp1_1_rxc_2_int),
    .qsfp1_rx_prbs31_enable_2(qsfp1_1_rx_prbs31_enable_2_int),
    .qsfp1_rx_error_count_2(qsfp1_1_rx_error_count_2_int),
    .qsfp1_tx_clk_3(qsfp1_1_tx_clk_3_int),
    .qsfp1_tx_rst_3(qsfp1_1_tx_rst_3_int),
    .qsfp1_txd_3(qsfp1_1_txd_3_int),
    .qsfp1_txc_3(qsfp1_1_txc_3_int),
    .qsfp1_tx_prbs31_enable_3(qsfp1_1_tx_prbs31_enable_3_int),
    .qsfp1_rx_clk_3(qsfp1_1_rx_clk_3_int),
    .qsfp1_rx_rst_3(qsfp1_1_rx_rst_3_int),
    .qsfp1_rxd_3(qsfp1_1_rxd_3_int),
    .qsfp1_rxc_3(qsfp1_1_rxc_3_int),
    .qsfp1_rx_prbs31_enable_3(qsfp1_1_rx_prbs31_enable_3_int),
    .qsfp1_rx_error_count_3(qsfp1_1_rx_error_count_3_int),
    .qsfp1_tx_clk_4(qsfp1_1_tx_clk_4_int),
    .qsfp1_tx_rst_4(qsfp1_1_tx_rst_4_int),
    .qsfp1_txd_4(qsfp1_1_txd_4_int),
    .qsfp1_txc_4(qsfp1_1_txc_4_int),
    .qsfp1_tx_prbs31_enable_4(qsfp1_1_tx_prbs31_enable_4_int),
    .qsfp1_rx_clk_4(qsfp1_1_rx_clk_4_int),
    .qsfp1_rx_rst_4(qsfp1_1_rx_rst_4_int),
    .qsfp1_rxd_4(qsfp1_1_rxd_4_int),
    .qsfp1_rxc_4(qsfp1_1_rxc_4_int),
    .qsfp1_rx_prbs31_enable_4(qsfp1_1_rx_prbs31_enable_4_int),
    .qsfp1_rx_error_count_4(qsfp1_1_rx_error_count_4_int),

    .qsfp2_tx_clk_1(qsfp1_2_tx_clk_1_int),
    .qsfp2_tx_rst_1(qsfp1_2_tx_rst_1_int),
    .qsfp2_txd_1(qsfp1_2_txd_1_int),
    .qsfp2_txc_1(qsfp1_2_txc_1_int),
    .qsfp2_tx_prbs31_enable_1(qsfp1_2_tx_prbs31_enable_1_int),
    .qsfp2_rx_clk_1(qsfp1_2_rx_clk_1_int),
    .qsfp2_rx_rst_1(qsfp1_2_rx_rst_1_int),
    .qsfp2_rxd_1(qsfp1_2_rxd_1_int),
    .qsfp2_rxc_1(qsfp1_2_rxc_1_int),
    .qsfp2_rx_prbs31_enable_1(qsfp1_2_rx_prbs31_enable_1_int),
    .qsfp2_rx_error_count_1(qsfp1_2_rx_error_count_1_int),
    .qsfp2_tx_clk_2(qsfp1_2_tx_clk_2_int),
    .qsfp2_tx_rst_2(qsfp1_2_tx_rst_2_int),
    .qsfp2_txd_2(qsfp1_2_txd_2_int),
    .qsfp2_txc_2(qsfp1_2_txc_2_int),
    .qsfp2_tx_prbs31_enable_2(qsfp1_2_tx_prbs31_enable_2_int),
    .qsfp2_rx_clk_2(qsfp1_2_rx_clk_2_int),
    .qsfp2_rx_rst_2(qsfp1_2_rx_rst_2_int),
    .qsfp2_rxd_2(qsfp1_2_rxd_2_int),
    .qsfp2_rxc_2(qsfp1_2_rxc_2_int),
    .qsfp2_rx_prbs31_enable_2(qsfp1_2_rx_prbs31_enable_2_int),
    .qsfp2_rx_error_count_2(qsfp1_2_rx_error_count_2_int),
    .qsfp2_tx_clk_3(qsfp1_2_tx_clk_3_int),
    .qsfp2_tx_rst_3(qsfp1_2_tx_rst_3_int),
    .qsfp2_txd_3(qsfp1_2_txd_3_int),
    .qsfp2_txc_3(qsfp1_2_txc_3_int),
    .qsfp2_tx_prbs31_enable_3(qsfp1_2_tx_prbs31_enable_3_int),
    .qsfp2_rx_clk_3(qsfp1_2_rx_clk_3_int),
    .qsfp2_rx_rst_3(qsfp1_2_rx_rst_3_int),
    .qsfp2_rxd_3(qsfp1_2_rxd_3_int),
    .qsfp2_rxc_3(qsfp1_2_rxc_3_int),
    .qsfp2_rx_prbs31_enable_3(qsfp1_2_rx_prbs31_enable_3_int),
    .qsfp2_rx_error_count_3(qsfp1_2_rx_error_count_3_int),
    .qsfp2_tx_clk_4(qsfp1_2_tx_clk_4_int),
    .qsfp2_tx_rst_4(qsfp1_2_tx_rst_4_int),
    .qsfp2_txd_4(qsfp1_2_txd_4_int),
    .qsfp2_txc_4(qsfp1_2_txc_4_int),
    .qsfp2_tx_prbs31_enable_4(qsfp1_2_tx_prbs31_enable_4_int),
    .qsfp2_rx_clk_4(qsfp1_2_rx_clk_4_int),
    .qsfp2_rx_rst_4(qsfp1_2_rx_rst_4_int),
    .qsfp2_rxd_4(qsfp1_2_rxd_4_int),
    .qsfp2_rxc_4(qsfp1_2_rxc_4_int),
    .qsfp2_rx_prbs31_enable_4(qsfp1_2_rx_prbs31_enable_4_int),
    .qsfp2_rx_error_count_4(qsfp1_2_rx_error_count_4_int),

    /*
     * QSPI flash
     */
    .fpga_boot(fpga_boot),
    .qspi_clk(qspi_clk_int),
    .qspi_0_dq_i(qspi_0_dq_i_int),
    .qspi_0_dq_o(qspi_0_dq_o_int),
    .qspi_0_dq_oe(qspi_0_dq_oe_int),
    .qspi_0_cs(qspi_0_cs_int)
);

/*
 ****************************** FPGA0 D2 ********************************
 */

// PCIe
wire pcie2_sys_clk;
wire pcie2_sys_clk_gt;

IBUFDS_GTE4 #(
    .REFCLK_HROW_CK_SEL(2'b00)
)
ibufds_gte4_pcie_mgt_refclk_inst2 (
    .I             (pcie2_refclk_0_p),
    .IB            (pcie2_refclk_0_n),
    .CEB           (1'b0),
    .O             (pcie2_sys_clk_gt),
    .ODIV2         (pcie2_sys_clk)
);

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis2_rq_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis2_rq_tkeep;
wire                               axis2_rq_tlast;
wire                               axis2_rq_tready;
wire [AXIS_PCIE_RQ_USER_WIDTH-1:0] axis2_rq_tuser;
wire                               axis2_rq_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis2_rc_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis2_rc_tkeep;
wire                               axis2_rc_tlast;
wire                               axis2_rc_tready;
wire [AXIS_PCIE_RC_USER_WIDTH-1:0] axis2_rc_tuser;
wire                               axis2_rc_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis2_cq_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis2_cq_tkeep;
wire                               axis2_cq_tlast;
wire                               axis2_cq_tready;
wire [AXIS_PCIE_CQ_USER_WIDTH-1:0] axis2_cq_tuser;
wire                               axis2_cq_tvalid;

wire [AXIS_PCIE_DATA_WIDTH-1:0]    axis2_cc_tdata;
wire [AXIS_PCIE_KEEP_WIDTH-1:0]    axis2_cc_tkeep;
wire                               axis2_cc_tlast;
wire                               axis2_cc_tready;
wire [AXIS_PCIE_CC_USER_WIDTH-1:0] axis2_cc_tuser;
wire                               axis2_cc_tvalid;

wire [RQ_SEQ_NUM_WIDTH-1:0]        pcie2_rq_seq_num0;
wire                               pcie2_rq_seq_num_vld0;
wire [RQ_SEQ_NUM_WIDTH-1:0]        pcie2_rq_seq_num1;
wire                               pcie2_rq_seq_num_vld1;

wire [3:0] pcie2_tfc_nph_av;
wire [3:0] pcie2_tfc_npd_av;

wire [2:0] cfg2_max_payload;
wire [2:0] cfg2_max_read_req;

wire [9:0]  cfg2_mgmt_addr;
wire [7:0]  cfg2_mgmt_function_number;
wire        cfg2_mgmt_write;
wire [31:0] cfg2_mgmt_write_data;
wire [3:0]  cfg2_mgmt_byte_enable;
wire        cfg2_mgmt_read;
wire [31:0] cfg2_mgmt_read_data;
wire        cfg2_mgmt_read_write_done;

wire [7:0]  cfg2_fc_ph;
wire [11:0] cfg2_fc_pd;
wire [7:0]  cfg2_fc_nph;
wire [11:0] cfg2_fc_npd;
wire [7:0]  cfg2_fc_cplh;
wire [11:0] cfg2_fc_cpld;
wire [2:0]  cfg2_fc_sel;

wire [3:0]  cfg2_interrupt_msi_enable;
wire [11:0] cfg2_interrupt_msi_mmenable;
wire        cfg2_interrupt_msi_mask_update;
wire [31:0] cfg2_interrupt_msi_data;
wire [3:0]  cfg2_interrupt_msi_select;
wire [31:0] cfg2_interrupt_msi_int;
wire [31:0] cfg2_interrupt_msi_pending_status;
wire        cfg2_interrupt_msi_pending_status_data_enable;
wire [3:0]  cfg2_interrupt_msi_pending_status_function_num;
wire        cfg2_interrupt_msi_sent;
wire        cfg2_interrupt_msi_fail;
wire [2:0]  cfg2_interrupt_msi_attr;
wire        cfg2_interrupt_msi_tph_present;
wire [1:0]  cfg2_interrupt_msi_tph_type;
wire [8:0]  cfg2_interrupt_msi_tph_st_tag;
wire [3:0]  cfg2_interrupt_msi_function_number;

wire status2_error_cor;
wire status2_error_uncor;

// extra register for pcie2_user_reset signal
wire pcie2_user_reset_int;
(* shreg_extract = "no" *)
reg pcie2_user_reset_reg_1 = 1'b1;
(* shreg_extract = "no" *)
reg pcie2_user_reset_reg_2 = 1'b1;

always @(posedge pcie2_user_clk) begin
    pcie2_user_reset_reg_1 <= pcie2_user_reset_int;
    pcie2_user_reset_reg_2 <= pcie2_user_reset_reg_1;
end

assign pcie2_user_reset = pcie2_user_reset_reg_2;

pcie4_uscale_plus_2
pcie4_uscale_plus_inst2 (
    .pci_exp_txn(pcie2_tx_n),
    .pci_exp_txp(pcie2_tx_p),
    .pci_exp_rxn(pcie2_rx_n),
    .pci_exp_rxp(pcie2_rx_p),
    .user_clk(pcie2_user_clk),
    .user_reset(pcie2_user_reset_int),
    .user_lnk_up(),

    .s_axis_rq_tdata(axis2_rq_tdata),
    .s_axis_rq_tkeep(axis2_rq_tkeep),
    .s_axis_rq_tlast(axis2_rq_tlast),
    .s_axis_rq_tready(axis2_rq_tready),
    .s_axis_rq_tuser(axis2_rq_tuser),
    .s_axis_rq_tvalid(axis2_rq_tvalid),

    .m_axis_rc_tdata(axis2_rc_tdata),
    .m_axis_rc_tkeep(axis2_rc_tkeep),
    .m_axis_rc_tlast(axis2_rc_tlast),
    .m_axis_rc_tready(axis2_rc_tready),
    .m_axis_rc_tuser(axis2_rc_tuser),
    .m_axis_rc_tvalid(axis2_rc_tvalid),

    .m_axis_cq_tdata(axis2_cq_tdata),
    .m_axis_cq_tkeep(axis2_cq_tkeep),
    .m_axis_cq_tlast(axis2_cq_tlast),
    .m_axis_cq_tready(axis2_cq_tready),
    .m_axis_cq_tuser(axis2_cq_tuser),
    .m_axis_cq_tvalid(axis2_cq_tvalid),

    .s_axis_cc_tdata(axis2_cc_tdata),
    .s_axis_cc_tkeep(axis2_cc_tkeep),
    .s_axis_cc_tlast(axis2_cc_tlast),
    .s_axis_cc_tready(axis2_cc_tready),
    .s_axis_cc_tuser(axis2_cc_tuser),
    .s_axis_cc_tvalid(axis2_cc_tvalid),

    .pcie_rq_seq_num0(pcie2_rq_seq_num0),
    .pcie_rq_seq_num_vld0(pcie2_rq_seq_num_vld0),
    .pcie_rq_seq_num1(pcie2_rq_seq_num1),
    .pcie_rq_seq_num_vld1(pcie2_rq_seq_num_vld1),
    .pcie_rq_tag0(),
    .pcie_rq_tag1(),
    .pcie_rq_tag_av(),
    .pcie_rq_tag_vld0(),
    .pcie_rq_tag_vld1(),

    .pcie_tfc_nph_av(pcie2_tfc_nph_av),
    .pcie_tfc_npd_av(pcie2_tfc_npd_av),

    .pcie_cq_np_req(1'b1),
    .pcie_cq_np_req_count(),

    .cfg_phy_link_down(),
    .cfg_phy_link_status(),
    .cfg_negotiated_width(),
    .cfg_current_speed(),
    .cfg_max_payload(cfg2_max_payload),
    .cfg_max_read_req(cfg2_max_read_req),
    .cfg_function_status(),
    .cfg_function_power_state(),
    .cfg_vf_status(),
    .cfg_vf_power_state(),
    .cfg_link_power_state(),

    .cfg_mgmt_addr(cfg2_mgmt_addr),
    .cfg_mgmt_function_number(cfg2_mgmt_function_number),
    .cfg_mgmt_write(cfg2_mgmt_write),
    .cfg_mgmt_write_data(cfg2_mgmt_write_data),
    .cfg_mgmt_byte_enable(cfg2_mgmt_byte_enable),
    .cfg_mgmt_read(cfg2_mgmt_read),
    .cfg_mgmt_read_data(cfg2_mgmt_read_data),
    .cfg_mgmt_read_write_done(cfg2_mgmt_read_write_done),
    .cfg_mgmt_debug_access(1'b0),

    .cfg_err_cor_out(),
    .cfg_err_nonfatal_out(),
    .cfg_err_fatal_out(),
    .cfg_local_error_valid(),
    .cfg_local_error_out(),
    .cfg_ltssm_state(),
    .cfg_rx_pm_state(),
    .cfg_tx_pm_state(),
    .cfg_rcb_status(),
    .cfg_obff_enable(),
    .cfg_pl_status_change(),
    .cfg_tph_requester_enable(),
    .cfg_tph_st_mode(),
    .cfg_vf_tph_requester_enable(),
    .cfg_vf_tph_st_mode(),

    .cfg_msg_received(),
    .cfg_msg_received_data(),
    .cfg_msg_received_type(),
    .cfg_msg_transmit(1'b0),
    .cfg_msg_transmit_type(3'd0),
    .cfg_msg_transmit_data(32'd0),
    .cfg_msg_transmit_done(),

    .cfg_fc_ph(cfg2_fc_ph),
    .cfg_fc_pd(cfg2_fc_pd),
    .cfg_fc_nph(cfg2_fc_nph),
    .cfg_fc_npd(cfg2_fc_npd),
    .cfg_fc_cplh(cfg2_fc_cplh),
    .cfg_fc_cpld(cfg2_fc_cpld),
    .cfg_fc_sel(cfg2_fc_sel),

    .cfg_dsn(64'd0),

    .cfg_power_state_change_ack(1'b1),
    .cfg_power_state_change_interrupt(),

    .cfg_err_cor_in(status2_error_cor),
    .cfg_err_uncor_in(status2_error_uncor),
    .cfg_flr_in_process(),
    .cfg_flr_done(4'd0),
    .cfg_vf_flr_in_process(),
    .cfg_vf_flr_func_num(8'd0),
    .cfg_vf_flr_done(8'd0),

    .cfg_link_training_enable(1'b1),

    .cfg_interrupt_int(4'd0),
    .cfg_interrupt_pending(4'd0),
    .cfg_interrupt_sent(),
    .cfg_interrupt_msi_enable(cfg2_interrupt_msi_enable),
    .cfg_interrupt_msi_mmenable(cfg2_interrupt_msi_mmenable),
    .cfg_interrupt_msi_mask_update(cfg2_interrupt_msi_mask_update),
    .cfg_interrupt_msi_data(cfg2_interrupt_msi_data),
    .cfg_interrupt_msi_select(cfg2_interrupt_msi_select),
    .cfg_interrupt_msi_int(cfg2_interrupt_msi_int),
    .cfg_interrupt_msi_pending_status(cfg2_interrupt_msi_pending_status),
    .cfg_interrupt_msi_pending_status_data_enable(cfg2_interrupt_msi_pending_status_data_enable),
    .cfg_interrupt_msi_pending_status_function_num(cfg2_interrupt_msi_pending_status_function_num),
    .cfg_interrupt_msi_sent(cfg2_interrupt_msi_sent),
    .cfg_interrupt_msi_fail(cfg2_interrupt_msi_fail),
    .cfg_interrupt_msi_attr(cfg2_interrupt_msi_attr),
    .cfg_interrupt_msi_tph_present(cfg2_interrupt_msi_tph_present),
    .cfg_interrupt_msi_tph_type(cfg2_interrupt_msi_tph_type),
    .cfg_interrupt_msi_tph_st_tag(cfg2_interrupt_msi_tph_st_tag),
    .cfg_interrupt_msi_function_number(cfg2_interrupt_msi_function_number),

    .cfg_pm_aspm_l1_entry_reject(1'b0),
    .cfg_pm_aspm_tx_l0s_entry_disable(1'b0),

    .cfg_hot_reset_out(),

    .cfg_config_space_enable(1'b1),
    .cfg_req_pm_transition_l23_ready(1'b0),
    .cfg_hot_reset_in(1'b0),

    .cfg_ds_port_number(8'd0),
    .cfg_ds_bus_number(8'd0),
    .cfg_ds_device_number(5'd0),

    .sys_clk(pcie2_sys_clk),
    .sys_clk_gt(pcie2_sys_clk_gt),
    .sys_reset(pcie2_reset_n),

    .phy_rdy_out()
);

// XGMII 10G PHY
wire        qsfp2_1_tx_clk_1_int;
wire        qsfp2_1_tx_rst_1_int;
wire [63:0] qsfp2_1_txd_1_int;
wire [7:0]  qsfp2_1_txc_1_int;
wire        qsfp2_1_tx_prbs31_enable_1_int;
wire        qsfp2_1_rx_clk_1_int;
wire        qsfp2_1_rx_rst_1_int;
wire [63:0] qsfp2_1_rxd_1_int;
wire [7:0]  qsfp2_1_rxc_1_int;
wire        qsfp2_1_rx_prbs31_enable_1_int;
wire [6:0]  qsfp2_1_rx_error_count_1_int;
wire        qsfp2_1_tx_clk_2_int;
wire        qsfp2_1_tx_rst_2_int;
wire [63:0] qsfp2_1_txd_2_int;
wire [7:0]  qsfp2_1_txc_2_int;
wire        qsfp2_1_tx_prbs31_enable_2_int;
wire        qsfp2_1_rx_clk_2_int;
wire        qsfp2_1_rx_rst_2_int;
wire [63:0] qsfp2_1_rxd_2_int;
wire [7:0]  qsfp2_1_rxc_2_int;
wire        qsfp2_1_rx_prbs31_enable_2_int;
wire [6:0]  qsfp2_1_rx_error_count_2_int;
wire        qsfp2_1_tx_clk_3_int;
wire        qsfp2_1_tx_rst_3_int;
wire [63:0] qsfp2_1_txd_3_int;
wire [7:0]  qsfp2_1_txc_3_int;
wire        qsfp2_1_tx_prbs31_enable_3_int;
wire        qsfp2_1_rx_clk_3_int;
wire        qsfp2_1_rx_rst_3_int;
wire [63:0] qsfp2_1_rxd_3_int;
wire [7:0]  qsfp2_1_rxc_3_int;
wire        qsfp2_1_rx_prbs31_enable_3_int;
wire [6:0]  qsfp2_1_rx_error_count_3_int;
wire        qsfp2_1_tx_clk_4_int;
wire        qsfp2_1_tx_rst_4_int;
wire [63:0] qsfp2_1_txd_4_int;
wire [7:0]  qsfp2_1_txc_4_int;
wire        qsfp2_1_tx_prbs31_enable_4_int;
wire        qsfp2_1_rx_clk_4_int;
wire        qsfp2_1_rx_rst_4_int;
wire [63:0] qsfp2_1_rxd_4_int;
wire [7:0]  qsfp2_1_rxc_4_int;
wire        qsfp2_1_rx_prbs31_enable_4_int;
wire [6:0]  qsfp2_1_rx_error_count_4_int;

wire        qsfp2_2_tx_clk_1_int;
wire        qsfp2_2_tx_rst_1_int;
wire [63:0] qsfp2_2_txd_1_int;
wire [7:0]  qsfp2_2_txc_1_int;
wire        qsfp2_2_tx_prbs31_enable_1_int;
wire        qsfp2_2_rx_clk_1_int;
wire        qsfp2_2_rx_rst_1_int;
wire [63:0] qsfp2_2_rxd_1_int;
wire [7:0]  qsfp2_2_rxc_1_int;
wire        qsfp2_2_rx_prbs31_enable_1_int;
wire [6:0]  qsfp2_2_rx_error_count_1_int;
wire        qsfp2_2_tx_clk_2_int;
wire        qsfp2_2_tx_rst_2_int;
wire [63:0] qsfp2_2_txd_2_int;
wire [7:0]  qsfp2_2_txc_2_int;
wire        qsfp2_2_tx_prbs31_enable_2_int;
wire        qsfp2_2_rx_clk_2_int;
wire        qsfp2_2_rx_rst_2_int;
wire [63:0] qsfp2_2_rxd_2_int;
wire [7:0]  qsfp2_2_rxc_2_int;
wire        qsfp2_2_rx_prbs31_enable_2_int;
wire [6:0]  qsfp2_2_rx_error_count_2_int;
wire        qsfp2_2_tx_clk_3_int;
wire        qsfp2_2_tx_rst_3_int;
wire [63:0] qsfp2_2_txd_3_int;
wire [7:0]  qsfp2_2_txc_3_int;
wire        qsfp2_2_tx_prbs31_enable_3_int;
wire        qsfp2_2_rx_clk_3_int;
wire        qsfp2_2_rx_rst_3_int;
wire [63:0] qsfp2_2_rxd_3_int;
wire [7:0]  qsfp2_2_rxc_3_int;
wire        qsfp2_2_rx_prbs31_enable_3_int;
wire [6:0]  qsfp2_2_rx_error_count_3_int;
wire        qsfp2_2_tx_clk_4_int;
wire        qsfp2_2_tx_rst_4_int;
wire [63:0] qsfp2_2_txd_4_int;
wire [7:0]  qsfp2_2_txc_4_int;
wire        qsfp2_2_tx_prbs31_enable_4_int;
wire        qsfp2_2_rx_clk_4_int;
wire        qsfp2_2_rx_rst_4_int;
wire [63:0] qsfp2_2_rxd_4_int;
wire [7:0]  qsfp2_2_rxc_4_int;
wire        qsfp2_2_rx_prbs31_enable_4_int;
wire [6:0]  qsfp2_2_rx_error_count_4_int;

wire qsfp2_1_rx_block_lock_1;
wire qsfp2_1_rx_block_lock_2;
wire qsfp2_1_rx_block_lock_3;
wire qsfp2_1_rx_block_lock_4;

wire qsfp2_2_rx_block_lock_1;
wire qsfp2_2_rx_block_lock_2;
wire qsfp2_2_rx_block_lock_3;
wire qsfp2_2_rx_block_lock_4;

wire qsfp2_1_mgt_refclk_0;

wire [7:0] gt2_txclkout;
wire gt2_txusrclk;

wire [7:0] gt2_rxclkout;
wire [7:0] gt2_rxusrclk;

wire gt2_reset_tx_done;
wire gt2_reset_rx_done;

wire [7:0] gt2_txprgdivresetdone;
wire [7:0] gt2_txpmaresetdone;
wire [7:0] gt2_rxprgdivresetdone;
wire [7:0] gt2_rxpmaresetdone;

wire gt2_tx_reset = ~((&gt2_txprgdivresetdone) & (&gt2_txpmaresetdone));
wire gt2_rx_reset = ~&gt2_rxpmaresetdone;

reg gt2_userclk_tx_active = 1'b0;
reg [7:0] gt2_userclk_rx_active = 1'b0;

IBUFDS_GTE4 ibufds_gte4_qsfp2_1_mgt_refclk_0_inst (
    .I             (qsfp2_1_mgt_refclk_0_p),
    .IB            (qsfp2_1_mgt_refclk_0_n),
    .CEB           (1'b0),
    .O             (qsfp2_1_mgt_refclk_0),
    .ODIV2         ()
);

// IBUFDS_GTE4 ibufds_gte4_qsfp2_2_mgt_refclk_0_inst (
//     .I             (qsfp2_2_mgt_refclk_0_p),
//     .IB            (qsfp2_2_mgt_refclk_0_n),
//     .CEB           (1'b0),
//     .O             (qsfp2_2_mgt_refclk_0),
//     .ODIV2         ()
// );

BUFG_GT bufg_gt2_tx_usrclk_inst (
    .CE      (1'b1),
    .CEMASK  (1'b0),
    .CLR     (gt2_tx_reset),
    .CLRMASK (1'b0),
    .DIV     (3'd0),
    .I       (gt2_txclkout[0]),
    .O       (gt2_txusrclk)
);

assign clk_156mhz_int2 = gt2_txusrclk;

always @(posedge gt2_txusrclk, posedge gt2_tx_reset) begin
    if (gt2_tx_reset) begin
        gt2_userclk_tx_active <= 1'b0;
    end else begin
        gt2_userclk_tx_active <= 1'b1;
    end
end

genvar n2;

generate

for (n2 = 0; n2 < 8; n2 = n2 + 1) begin

    BUFG_GT bufg_gt2_rx_usrclk_inst (
        .CE      (1'b1),
        .CEMASK  (1'b0),
        .CLR     (gt2_rx_reset),
        .CLRMASK (1'b0),
        .DIV     (3'd0),
        .I       (gt2_rxclkout[n2]),
        .O       (gt2_rxusrclk[n2])
    );

    always @(posedge gt2_rxusrclk[n2], posedge gt2_rx_reset) begin
        if (gt2_rx_reset) begin
            gt2_userclk_rx_active[n2] <= 1'b0;
        end else begin
            gt2_userclk_rx_active[n2] <= 1'b1;
        end
    end

end

endgenerate

sync_reset #(
    .N(4)
)
sync_reset_156mhz_inst2 (
    .clk(clk_156mhz_int2),
    .rst(~gt2_reset_tx_done),
    .out(rst_156mhz_int2)
);

wire [5:0] qsfp2_1_gt_txheader_1;
wire [63:0] qsfp2_1_gt_txdata_1;
wire qsfp2_1_gt_rxgearboxslip_1;
wire [5:0] qsfp2_1_gt_rxheader_1;
wire [1:0] qsfp2_1_gt_rxheadervalid_1;
wire [63:0] qsfp2_1_gt_rxdata_1;
wire [1:0] qsfp2_1_gt_rxdatavalid_1;

wire [5:0] qsfp2_1_gt_txheader_2;
wire [63:0] qsfp2_1_gt_txdata_2;
wire qsfp2_1_gt_rxgearboxslip_2;
wire [5:0] qsfp2_1_gt_rxheader_2;
wire [1:0] qsfp2_1_gt_rxheadervalid_2;
wire [63:0] qsfp2_1_gt_rxdata_2;
wire [1:0] qsfp2_1_gt_rxdatavalid_2;

wire [5:0] qsfp2_1_gt_txheader_3;
wire [63:0] qsfp2_1_gt_txdata_3;
wire qsfp2_1_gt_rxgearboxslip_3;
wire [5:0] qsfp2_1_gt_rxheader_3;
wire [1:0] qsfp2_1_gt_rxheadervalid_3;
wire [63:0] qsfp2_1_gt_rxdata_3;
wire [1:0] qsfp2_1_gt_rxdatavalid_3;

wire [5:0] qsfp2_1_gt_txheader_4;
wire [63:0] qsfp2_1_gt_txdata_4;
wire qsfp2_1_gt_rxgearboxslip_4;
wire [5:0] qsfp2_1_gt_rxheader_4;
wire [1:0] qsfp2_1_gt_rxheadervalid_4;
wire [63:0] qsfp2_1_gt_rxdata_4;
wire [1:0] qsfp2_1_gt_rxdatavalid_4;

wire [5:0] qsfp2_2_gt_txheader_1;
wire [63:0] qsfp2_2_gt_txdata_1;
wire qsfp2_2_gt_rxgearboxslip_1;
wire [5:0] qsfp2_2_gt_rxheader_1;
wire [1:0] qsfp2_2_gt_rxheadervalid_1;
wire [63:0] qsfp2_2_gt_rxdata_1;
wire [1:0] qsfp2_2_gt_rxdatavalid_1;

wire [5:0] qsfp2_2_gt_txheader_2;
wire [63:0] qsfp2_2_gt_txdata_2;
wire qsfp2_2_gt_rxgearboxslip_2;
wire [5:0] qsfp2_2_gt_rxheader_2;
wire [1:0] qsfp2_2_gt_rxheadervalid_2;
wire [63:0] qsfp2_2_gt_rxdata_2;
wire [1:0] qsfp2_2_gt_rxdatavalid_2;

wire [5:0] qsfp2_2_gt_txheader_3;
wire [63:0] qsfp2_2_gt_txdata_3;
wire qsfp2_2_gt_rxgearboxslip_3;
wire [5:0] qsfp2_2_gt_rxheader_3;
wire [1:0] qsfp2_2_gt_rxheadervalid_3;
wire [63:0] qsfp2_2_gt_rxdata_3;
wire [1:0] qsfp2_2_gt_rxdatavalid_3;

wire [5:0] qsfp2_2_gt_txheader_4;
wire [63:0] qsfp2_2_gt_txdata_4;
wire qsfp2_2_gt_rxgearboxslip_4;
wire [5:0] qsfp2_2_gt_rxheader_4;
wire [1:0] qsfp2_2_gt_rxheadervalid_4;
wire [63:0] qsfp2_2_gt_rxdata_4;
wire [1:0] qsfp2_2_gt_rxdatavalid_4;

gtwizard_ultrascale_2
qsfp_gty_inst2 (
    .gtwiz_userclk_tx_active_in(&gt2_userclk_tx_active),
    .gtwiz_userclk_rx_active_in(&gt2_userclk_rx_active),

    .gtwiz_reset_clk_freerun_in(clk_125mhz_int2),
    .gtwiz_reset_all_in(rst_125mhz_int2),

    .gtwiz_reset_tx_pll_and_datapath_in(1'b0),
    .gtwiz_reset_tx_datapath_in(1'b0),

    .gtwiz_reset_rx_pll_and_datapath_in(1'b0),
    .gtwiz_reset_rx_datapath_in(1'b0),

    .gtwiz_reset_rx_cdr_stable_out(),

    .gtwiz_reset_tx_done_out(gt2_reset_tx_done),
    .gtwiz_reset_rx_done_out(gt2_reset_rx_done),

    .gtrefclk00_in({2{qsfp2_1_mgt_refclk_0}}),

    .qpll0outclk_out(),
    .qpll0outrefclk_out(),

    .gtyrxn_in({qsfp2_1_rx4_n, qsfp2_1_rx2_n, qsfp2_1_rx3_n, qsfp2_1_rx1_n, qsfp2_2_rx4_n, qsfp2_2_rx3_n, qsfp2_2_rx2_n, qsfp2_2_rx1_n}),
    .gtyrxp_in({qsfp2_1_rx4_p, qsfp2_1_rx2_p, qsfp2_1_rx3_p, qsfp2_1_rx1_p, qsfp2_2_rx4_p, qsfp2_2_rx3_p, qsfp2_2_rx2_p, qsfp2_2_rx1_p}),

    .rxusrclk_in({gt2_rxusrclk[3], gt2_rxusrclk[1], gt2_rxusrclk[2], gt2_rxusrclk[0], gt2_rxusrclk[7:4]}),
    .rxusrclk2_in({gt2_rxusrclk[3], gt2_rxusrclk[1], gt2_rxusrclk[2], gt2_rxusrclk[0], gt2_rxusrclk[7:4]}),

    .gtwiz_userdata_tx_in({qsfp2_1_gt_txdata_4, qsfp2_1_gt_txdata_2, qsfp2_1_gt_txdata_3, qsfp2_1_gt_txdata_1, qsfp2_2_gt_txdata_4, qsfp2_2_gt_txdata_3, qsfp2_2_gt_txdata_2, qsfp2_2_gt_txdata_1}),
    .txheader_in({qsfp2_1_gt_txheader_4, qsfp2_1_gt_txheader_2, qsfp2_1_gt_txheader_3, qsfp2_1_gt_txheader_1, qsfp2_2_gt_txheader_4, qsfp2_2_gt_txheader_3, qsfp2_2_gt_txheader_2, qsfp2_2_gt_txheader_1}),
    .txsequence_in({8{1'b0}}),

    .txusrclk_in({8{gt2_txusrclk}}),
    .txusrclk2_in({8{gt2_txusrclk}}),

    .gtpowergood_out(),

    .gtytxn_out({qsfp2_1_tx4_n, qsfp2_1_tx2_n, qsfp2_1_tx3_n, qsfp2_1_tx1_n, qsfp2_2_tx4_n, qsfp2_2_tx3_n, qsfp2_2_tx2_n, qsfp2_2_tx1_n}),
    .gtytxp_out({qsfp2_1_tx4_p, qsfp2_1_tx2_p, qsfp2_1_tx3_p, qsfp2_1_tx1_p, qsfp2_2_tx4_p, qsfp2_2_tx3_p, qsfp2_2_tx2_p, qsfp2_2_tx1_p}),

    .rxgearboxslip_in({qsfp2_1_gt_rxgearboxslip_4, qsfp2_1_gt_rxgearboxslip_2, qsfp2_1_gt_rxgearboxslip_3, qsfp2_1_gt_rxgearboxslip_1, qsfp2_2_gt_rxgearboxslip_4, qsfp2_2_gt_rxgearboxslip_3, qsfp2_2_gt_rxgearboxslip_2, qsfp2_2_gt_rxgearboxslip_1}),
    .gtwiz_userdata_rx_out({qsfp2_1_gt_rxdata_4, qsfp2_1_gt_rxdata_2, qsfp2_1_gt_rxdata_3, qsfp2_1_gt_rxdata_1, qsfp2_2_gt_rxdata_4, qsfp2_2_gt_rxdata_3, qsfp2_2_gt_rxdata_2, qsfp2_2_gt_rxdata_1}),
    .rxdatavalid_out({qsfp2_1_gt_rxdatavalid_4, qsfp2_1_gt_rxdatavalid_2, qsfp2_1_gt_rxdatavalid_3, qsfp2_1_gt_rxdatavalid_1, qsfp2_2_gt_rxdatavalid_4, qsfp2_2_gt_rxdatavalid_3, qsfp2_2_gt_rxdatavalid_2, qsfp2_2_gt_rxdatavalid_1}),
    .rxheader_out({qsfp2_1_gt_rxheader_4, qsfp2_1_gt_rxheader_2, qsfp2_1_gt_rxheader_3, qsfp2_1_gt_rxheader_1, qsfp2_2_gt_rxheader_4, qsfp2_2_gt_rxheader_3, qsfp2_2_gt_rxheader_2, qsfp2_2_gt_rxheader_1}),
    .rxheadervalid_out({qsfp2_1_gt_rxheadervalid_4, qsfp2_1_gt_rxheadervalid_2, qsfp2_1_gt_rxheadervalid_3, qsfp2_1_gt_rxheadervalid_1, qsfp2_2_gt_rxheadervalid_4, qsfp2_2_gt_rxheadervalid_3, qsfp2_2_gt_rxheadervalid_2, qsfp2_2_gt_rxheadervalid_1}),
    .rxoutclk_out({gt2_rxclkout[3], gt2_rxclkout[1], gt2_rxclkout[2], gt2_rxclkout[0], gt2_rxclkout[7:4]}),
    .rxpmaresetdone_out(gt2_rxpmaresetdone),
    .rxprgdivresetdone_out(gt2_rxprgdivresetdone),
    .rxstartofseq_out(),

    .txoutclk_out(gt2_txclkout),
    .txpmaresetdone_out(gt2_txpmaresetdone),
    .txprgdivresetdone_out(gt2_txprgdivresetdone)
);

assign qsfp2_1_tx_clk_1_int = clk_156mhz_int2;
assign qsfp2_1_tx_rst_1_int = rst_156mhz_int2;

assign qsfp2_1_rx_clk_1_int = gt2_rxusrclk[0];

sync_reset #(
    .N(4)
)
qsfp2_1_rx_rst_1_reset_sync_inst (
    .clk(qsfp2_1_rx_clk_1_int),
    .rst(~gt2_reset_rx_done),
    .out(qsfp2_1_rx_rst_1_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp2_1_phy_1_inst (
    .tx_clk(qsfp2_1_tx_clk_1_int),
    .tx_rst(qsfp2_1_tx_rst_1_int),
    .rx_clk(qsfp2_1_rx_clk_1_int),
    .rx_rst(qsfp2_1_rx_rst_1_int),
    .xgmii_txd(qsfp2_1_txd_1_int),
    .xgmii_txc(qsfp2_1_txc_1_int),
    .xgmii_rxd(qsfp2_1_rxd_1_int),
    .xgmii_rxc(qsfp2_1_rxc_1_int),
    .serdes_tx_data(qsfp2_1_gt_txdata_1),
    .serdes_tx_hdr(qsfp2_1_gt_txheader_1),
    .serdes_rx_data(qsfp2_1_gt_rxdata_1),
    .serdes_rx_hdr(qsfp2_1_gt_rxheader_1),
    .serdes_rx_bitslip(qsfp2_1_gt_rxgearboxslip_1),
    .rx_error_count(qsfp2_1_rx_error_count_1_int),
    .rx_block_lock(qsfp2_1_rx_block_lock_1),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp2_1_tx_prbs31_enable_1_int),
    .rx_prbs31_enable(qsfp2_1_rx_prbs31_enable_1_int)
);

assign qsfp2_1_tx_clk_2_int = clk_156mhz_int2;
assign qsfp2_1_tx_rst_2_int = rst_156mhz_int2;

assign qsfp2_1_rx_clk_2_int = gt2_rxusrclk[1];

sync_reset #(
    .N(4)
)
qsfp2_1_rx_rst_2_reset_sync_inst (
    .clk(qsfp2_1_rx_clk_2_int),
    .rst(~gt2_reset_rx_done),
    .out(qsfp2_1_rx_rst_2_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp2_1_phy_2_inst (
    .tx_clk(qsfp2_1_tx_clk_2_int),
    .tx_rst(qsfp2_1_tx_rst_2_int),
    .rx_clk(qsfp2_1_rx_clk_2_int),
    .rx_rst(qsfp2_1_rx_rst_2_int),
    .xgmii_txd(qsfp2_1_txd_2_int),
    .xgmii_txc(qsfp2_1_txc_2_int),
    .xgmii_rxd(qsfp2_1_rxd_2_int),
    .xgmii_rxc(qsfp2_1_rxc_2_int),
    .serdes_tx_data(qsfp2_1_gt_txdata_2),
    .serdes_tx_hdr(qsfp2_1_gt_txheader_2),
    .serdes_rx_data(qsfp2_1_gt_rxdata_2),
    .serdes_rx_hdr(qsfp2_1_gt_rxheader_2),
    .serdes_rx_bitslip(qsfp2_1_gt_rxgearboxslip_2),
    .rx_error_count(qsfp2_1_rx_error_count_2_int),
    .rx_block_lock(qsfp2_1_rx_block_lock_2),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp2_1_tx_prbs31_enable_2_int),
    .rx_prbs31_enable(qsfp2_1_rx_prbs31_enable_2_int)
);

assign qsfp2_1_tx_clk_3_int = clk_156mhz_int2;
assign qsfp2_1_tx_rst_3_int = rst_156mhz_int2;

assign qsfp2_1_rx_clk_3_int = gt2_rxusrclk[2];

sync_reset #(
    .N(4)
)
qsfp2_1_rx_rst_3_reset_sync_inst (
    .clk(qsfp2_1_rx_clk_3_int),
    .rst(~gt2_reset_rx_done),
    .out(qsfp2_1_rx_rst_3_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp2_1_phy_3_inst (
    .tx_clk(qsfp2_1_tx_clk_3_int),
    .tx_rst(qsfp2_1_tx_rst_3_int),
    .rx_clk(qsfp2_1_rx_clk_3_int),
    .rx_rst(qsfp2_1_rx_rst_3_int),
    .xgmii_txd(qsfp2_1_txd_3_int),
    .xgmii_txc(qsfp2_1_txc_3_int),
    .xgmii_rxd(qsfp2_1_rxd_3_int),
    .xgmii_rxc(qsfp2_1_rxc_3_int),
    .serdes_tx_data(qsfp2_1_gt_txdata_3),
    .serdes_tx_hdr(qsfp2_1_gt_txheader_3),
    .serdes_rx_data(qsfp2_1_gt_rxdata_3),
    .serdes_rx_hdr(qsfp2_1_gt_rxheader_3),
    .serdes_rx_bitslip(qsfp2_1_gt_rxgearboxslip_3),
    .rx_error_count(qsfp2_1_rx_error_count_3_int),
    .rx_block_lock(qsfp2_1_rx_block_lock_3),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp2_1_tx_prbs31_enable_3_int),
    .rx_prbs31_enable(qsfp2_1_rx_prbs31_enable_3_int)
);

assign qsfp2_1_tx_clk_4_int = clk_156mhz_int2;
assign qsfp2_1_tx_rst_4_int = rst_156mhz_int2;

assign qsfp2_1_rx_clk_4_int = gt2_rxusrclk[3];

sync_reset #(
    .N(4)
)
qsfp2_1_rx_rst_4_reset_sync_inst (
    .clk(qsfp2_1_rx_clk_4_int),
    .rst(~gt2_reset_rx_done),
    .out(qsfp2_1_rx_rst_4_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp2_1_phy_4_inst (
    .tx_clk(qsfp2_1_tx_clk_4_int),
    .tx_rst(qsfp2_1_tx_rst_4_int),
    .rx_clk(qsfp2_1_rx_clk_4_int),
    .rx_rst(qsfp2_1_rx_rst_4_int),
    .xgmii_txd(qsfp2_1_txd_4_int),
    .xgmii_txc(qsfp2_1_txc_4_int),
    .xgmii_rxd(qsfp2_1_rxd_4_int),
    .xgmii_rxc(qsfp2_1_rxc_4_int),
    .serdes_tx_data(qsfp2_1_gt_txdata_4),
    .serdes_tx_hdr(qsfp2_1_gt_txheader_4),
    .serdes_rx_data(qsfp2_1_gt_rxdata_4),
    .serdes_rx_hdr(qsfp2_1_gt_rxheader_4),
    .serdes_rx_bitslip(qsfp2_1_gt_rxgearboxslip_4),
    .rx_error_count(qsfp2_1_rx_error_count_4_int),
    .rx_block_lock(qsfp2_1_rx_block_lock_4),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp2_1_tx_prbs31_enable_4_int),
    .rx_prbs31_enable(qsfp2_1_rx_prbs31_enable_4_int)
);

assign qsfp2_2_tx_clk_1_int = clk_156mhz_int2;
assign qsfp2_2_tx_rst_1_int = rst_156mhz_int2;

assign qsfp2_2_rx_clk_1_int = gt2_rxusrclk[4];

sync_reset #(
    .N(4)
)
qsfp2_2_rx_rst_1_reset_sync_inst (
    .clk(qsfp2_2_rx_clk_1_int),
    .rst(~gt2_reset_rx_done),
    .out(qsfp2_2_rx_rst_1_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp2_2_phy_1_inst (
    .tx_clk(qsfp2_2_tx_clk_1_int),
    .tx_rst(qsfp2_2_tx_rst_1_int),
    .rx_clk(qsfp2_2_rx_clk_1_int),
    .rx_rst(qsfp2_2_rx_rst_1_int),
    .xgmii_txd(qsfp2_2_txd_1_int),
    .xgmii_txc(qsfp2_2_txc_1_int),
    .xgmii_rxd(qsfp2_2_rxd_1_int),
    .xgmii_rxc(qsfp2_2_rxc_1_int),
    .serdes_tx_data(qsfp2_2_gt_txdata_1),
    .serdes_tx_hdr(qsfp2_2_gt_txheader_1),
    .serdes_rx_data(qsfp2_2_gt_rxdata_1),
    .serdes_rx_hdr(qsfp2_2_gt_rxheader_1),
    .serdes_rx_bitslip(qsfp2_2_gt_rxgearboxslip_1),
    .rx_error_count(qsfp2_2_rx_error_count_1_int),
    .rx_block_lock(qsfp2_2_rx_block_lock_1),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp2_2_tx_prbs31_enable_1_int),
    .rx_prbs31_enable(qsfp2_2_rx_prbs31_enable_1_int)
);

assign qsfp2_2_tx_clk_2_int = clk_156mhz_int2;
assign qsfp2_2_tx_rst_2_int = rst_156mhz_int2;

assign qsfp2_2_rx_clk_2_int = gt2_rxusrclk[5];

sync_reset #(
    .N(4)
)
qsfp2_2_rx_rst_2_reset_sync_inst (
    .clk(qsfp2_2_rx_clk_2_int),
    .rst(~gt2_reset_rx_done),
    .out(qsfp2_2_rx_rst_2_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp2_2_phy_2_inst (
    .tx_clk(qsfp2_2_tx_clk_2_int),
    .tx_rst(qsfp2_2_tx_rst_2_int),
    .rx_clk(qsfp2_2_rx_clk_2_int),
    .rx_rst(qsfp2_2_rx_rst_2_int),
    .xgmii_txd(qsfp2_2_txd_2_int),
    .xgmii_txc(qsfp2_2_txc_2_int),
    .xgmii_rxd(qsfp2_2_rxd_2_int),
    .xgmii_rxc(qsfp2_2_rxc_2_int),
    .serdes_tx_data(qsfp2_2_gt_txdata_2),
    .serdes_tx_hdr(qsfp2_2_gt_txheader_2),
    .serdes_rx_data(qsfp2_2_gt_rxdata_2),
    .serdes_rx_hdr(qsfp2_2_gt_rxheader_2),
    .serdes_rx_bitslip(qsfp2_2_gt_rxgearboxslip_2),
    .rx_error_count(qsfp2_2_rx_error_count_2_int),
    .rx_block_lock(qsfp2_2_rx_block_lock_2),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp2_2_tx_prbs31_enable_2_int),
    .rx_prbs31_enable(qsfp2_2_rx_prbs31_enable_2_int)
);

assign qsfp2_2_tx_clk_3_int = clk_156mhz_int2;
assign qsfp2_2_tx_rst_3_int = rst_156mhz_int2;

assign qsfp2_2_rx_clk_3_int = gt2_rxusrclk[6];

sync_reset #(
    .N(4)
)
qsfp2_2_rx_rst_3_reset_sync_inst (
    .clk(qsfp2_2_rx_clk_3_int),
    .rst(~gt2_reset_rx_done),
    .out(qsfp2_2_rx_rst_3_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp2_2_phy_3_inst (
    .tx_clk(qsfp2_2_tx_clk_3_int),
    .tx_rst(qsfp2_2_tx_rst_3_int),
    .rx_clk(qsfp2_2_rx_clk_3_int),
    .rx_rst(qsfp2_2_rx_rst_3_int),
    .xgmii_txd(qsfp2_2_txd_3_int),
    .xgmii_txc(qsfp2_2_txc_3_int),
    .xgmii_rxd(qsfp2_2_rxd_3_int),
    .xgmii_rxc(qsfp2_2_rxc_3_int),
    .serdes_tx_data(qsfp2_2_gt_txdata_3),
    .serdes_tx_hdr(qsfp2_2_gt_txheader_3),
    .serdes_rx_data(qsfp2_2_gt_rxdata_3),
    .serdes_rx_hdr(qsfp2_2_gt_rxheader_3),
    .serdes_rx_bitslip(qsfp2_2_gt_rxgearboxslip_3),
    .rx_error_count(qsfp2_2_rx_error_count_3_int),
    .rx_block_lock(qsfp2_2_rx_block_lock_3),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp2_2_tx_prbs31_enable_3_int),
    .rx_prbs31_enable(qsfp2_2_rx_prbs31_enable_3_int)
);

assign qsfp2_2_tx_clk_4_int = clk_156mhz_int2;
assign qsfp2_2_tx_rst_4_int = rst_156mhz_int2;

assign qsfp2_2_rx_clk_4_int = gt2_rxusrclk[7];

sync_reset #(
    .N(4)
)
qsfp2_2_rx_rst_4_reset_sync_inst (
    .clk(qsfp2_2_rx_clk_4_int),
    .rst(~gt2_reset_rx_done),
    .out(qsfp2_2_rx_rst_4_int)
);

eth_phy_10g #(
    .BIT_REVERSE(1),
    .PRBS31_ENABLE(1)
)
qsfp2_2_phy_4_inst (
    .tx_clk(qsfp2_2_tx_clk_4_int),
    .tx_rst(qsfp2_2_tx_rst_4_int),
    .rx_clk(qsfp2_2_rx_clk_4_int),
    .rx_rst(qsfp2_2_rx_rst_4_int),
    .xgmii_txd(qsfp2_2_txd_4_int),
    .xgmii_txc(qsfp2_2_txc_4_int),
    .xgmii_rxd(qsfp2_2_rxd_4_int),
    .xgmii_rxc(qsfp2_2_rxc_4_int),
    .serdes_tx_data(qsfp2_2_gt_txdata_4),
    .serdes_tx_hdr(qsfp2_2_gt_txheader_4),
    .serdes_rx_data(qsfp2_2_gt_rxdata_4),
    .serdes_rx_hdr(qsfp2_2_gt_rxheader_4),
    .serdes_rx_bitslip(qsfp2_2_gt_rxgearboxslip_4),
    .rx_error_count(qsfp2_2_rx_error_count_4_int),
    .rx_block_lock(qsfp2_2_rx_block_lock_4),
    .rx_high_ber(),
    .tx_prbs31_enable(qsfp2_2_tx_prbs31_enable_4_int),
    .rx_prbs31_enable(qsfp2_2_rx_prbs31_enable_4_int)
);

fpga_core #(
    .AXIS_PCIE_DATA_WIDTH(AXIS_PCIE_DATA_WIDTH),
    .AXIS_PCIE_KEEP_WIDTH(AXIS_PCIE_KEEP_WIDTH),
    .AXIS_PCIE_RC_USER_WIDTH(AXIS_PCIE_RC_USER_WIDTH),
    .AXIS_PCIE_RQ_USER_WIDTH(AXIS_PCIE_RQ_USER_WIDTH),
    .AXIS_PCIE_CQ_USER_WIDTH(AXIS_PCIE_CQ_USER_WIDTH),
    .AXIS_PCIE_CC_USER_WIDTH(AXIS_PCIE_CC_USER_WIDTH),
    .RQ_SEQ_NUM_WIDTH(RQ_SEQ_NUM_WIDTH),
    .BAR0_APERTURE(BAR0_APERTURE)
)
core_inst2 (
    /*
     * Clock: 250 MHz
     * Synchronous reset
     */
    .clk_250mhz(pcie2_user_clk),
    .rst_250mhz(pcie2_user_reset),

    /*
     * GPIO
     */
    .led(),

    /*
     * PCIe
     */
    .m_axis_rq_tdata(axis2_rq_tdata),
    .m_axis_rq_tkeep(axis2_rq_tkeep),
    .m_axis_rq_tlast(axis2_rq_tlast),
    .m_axis_rq_tready(axis2_rq_tready),
    .m_axis_rq_tuser(axis2_rq_tuser),
    .m_axis_rq_tvalid(axis2_rq_tvalid),

    .s_axis_rc_tdata(axis2_rc_tdata),
    .s_axis_rc_tkeep(axis2_rc_tkeep),
    .s_axis_rc_tlast(axis2_rc_tlast),
    .s_axis_rc_tready(axis2_rc_tready),
    .s_axis_rc_tuser(axis2_rc_tuser),
    .s_axis_rc_tvalid(axis2_rc_tvalid),

    .s_axis_cq_tdata(axis2_cq_tdata),
    .s_axis_cq_tkeep(axis2_cq_tkeep),
    .s_axis_cq_tlast(axis2_cq_tlast),
    .s_axis_cq_tready(axis2_cq_tready),
    .s_axis_cq_tuser(axis2_cq_tuser),
    .s_axis_cq_tvalid(axis2_cq_tvalid),

    .m_axis_cc_tdata(axis2_cc_tdata),
    .m_axis_cc_tkeep(axis2_cc_tkeep),
    .m_axis_cc_tlast(axis2_cc_tlast),
    .m_axis_cc_tready(axis2_cc_tready),
    .m_axis_cc_tuser(axis2_cc_tuser),
    .m_axis_cc_tvalid(axis2_cc_tvalid),

    .s_axis_rq_seq_num_0(pcie2_rq_seq_num0),
    .s_axis_rq_seq_num_valid_0(pcie2_rq_seq_num_vld0),
    .s_axis_rq_seq_num_1(pcie2_rq_seq_num1),
    .s_axis_rq_seq_num_valid_1(pcie2_rq_seq_num_vld1),

    .pcie_tfc_nph_av(pcie2_tfc_nph_av),
    .pcie_tfc_npd_av(pcie2_tfc_npd_av),

    .cfg_max_payload(cfg2_max_payload),
    .cfg_max_read_req(cfg2_max_read_req),

    .cfg_mgmt_addr(cfg2_mgmt_addr),
    .cfg_mgmt_function_number(cfg2_mgmt_function_number),
    .cfg_mgmt_write(cfg2_mgmt_write),
    .cfg_mgmt_write_data(cfg2_mgmt_write_data),
    .cfg_mgmt_byte_enable(cfg2_mgmt_byte_enable),
    .cfg_mgmt_read(cfg2_mgmt_read),
    .cfg_mgmt_read_data(cfg2_mgmt_read_data),
    .cfg_mgmt_read_write_done(cfg2_mgmt_read_write_done),

    .cfg_fc_ph(cfg2_fc_ph),
    .cfg_fc_pd(cfg2_fc_pd),
    .cfg_fc_nph(cfg2_fc_nph),
    .cfg_fc_npd(cfg2_fc_npd),
    .cfg_fc_cplh(cfg2_fc_cplh),
    .cfg_fc_cpld(cfg2_fc_cpld),
    .cfg_fc_sel(cfg2_fc_sel),

    .cfg_interrupt_msi_enable(cfg2_interrupt_msi_enable),
    .cfg_interrupt_msi_mmenable(cfg2_interrupt_msi_mmenable),
    .cfg_interrupt_msi_mask_update(cfg2_interrupt_msi_mask_update),
    .cfg_interrupt_msi_data(cfg2_interrupt_msi_data),
    .cfg_interrupt_msi_select(cfg2_interrupt_msi_select),
    .cfg_interrupt_msi_int(cfg2_interrupt_msi_int),
    .cfg_interrupt_msi_pending_status(cfg2_interrupt_msi_pending_status),
    .cfg_interrupt_msi_pending_status_data_enable(cfg2_interrupt_msi_pending_status_data_enable),
    .cfg_interrupt_msi_pending_status_function_num(cfg2_interrupt_msi_pending_status_function_num),
    .cfg_interrupt_msi_sent(cfg2_interrupt_msi_sent),
    .cfg_interrupt_msi_fail(cfg2_interrupt_msi_fail),
    .cfg_interrupt_msi_attr(cfg2_interrupt_msi_attr),
    .cfg_interrupt_msi_tph_present(cfg2_interrupt_msi_tph_present),
    .cfg_interrupt_msi_tph_type(cfg2_interrupt_msi_tph_type),
    .cfg_interrupt_msi_tph_st_tag(cfg2_interrupt_msi_tph_st_tag),
    .cfg_interrupt_msi_function_number(cfg2_interrupt_msi_function_number),

    .status_error_cor(status2_error_cor),
    .status_error_uncor(status2_error_uncor),

    /*
     * Ethernet: QSFP28
     */
    .qsfp1_tx_clk_1(qsfp2_1_tx_clk_1_int),
    .qsfp1_tx_rst_1(qsfp2_1_tx_rst_1_int),
    .qsfp1_txd_1(qsfp2_1_txd_1_int),
    .qsfp1_txc_1(qsfp2_1_txc_1_int),
    .qsfp1_tx_prbs31_enable_1(qsfp2_1_tx_prbs31_enable_1_int),
    .qsfp1_rx_clk_1(qsfp2_1_rx_clk_1_int),
    .qsfp1_rx_rst_1(qsfp2_1_rx_rst_1_int),
    .qsfp1_rxd_1(qsfp2_1_rxd_1_int),
    .qsfp1_rxc_1(qsfp2_1_rxc_1_int),
    .qsfp1_rx_prbs31_enable_1(qsfp2_1_rx_prbs31_enable_1_int),
    .qsfp1_rx_error_count_1(qsfp2_1_rx_error_count_1_int),
    .qsfp1_tx_clk_2(qsfp2_1_tx_clk_2_int),
    .qsfp1_tx_rst_2(qsfp2_1_tx_rst_2_int),
    .qsfp1_txd_2(qsfp2_1_txd_2_int),
    .qsfp1_txc_2(qsfp2_1_txc_2_int),
    .qsfp1_tx_prbs31_enable_2(qsfp2_1_tx_prbs31_enable_2_int),
    .qsfp1_rx_clk_2(qsfp2_1_rx_clk_2_int),
    .qsfp1_rx_rst_2(qsfp2_1_rx_rst_2_int),
    .qsfp1_rxd_2(qsfp2_1_rxd_2_int),
    .qsfp1_rxc_2(qsfp2_1_rxc_2_int),
    .qsfp1_rx_prbs31_enable_2(qsfp2_1_rx_prbs31_enable_2_int),
    .qsfp1_rx_error_count_2(qsfp2_1_rx_error_count_2_int),
    .qsfp1_tx_clk_3(qsfp2_1_tx_clk_3_int),
    .qsfp1_tx_rst_3(qsfp2_1_tx_rst_3_int),
    .qsfp1_txd_3(qsfp2_1_txd_3_int),
    .qsfp1_txc_3(qsfp2_1_txc_3_int),
    .qsfp1_tx_prbs31_enable_3(qsfp2_1_tx_prbs31_enable_3_int),
    .qsfp1_rx_clk_3(qsfp2_1_rx_clk_3_int),
    .qsfp1_rx_rst_3(qsfp2_1_rx_rst_3_int),
    .qsfp1_rxd_3(qsfp2_1_rxd_3_int),
    .qsfp1_rxc_3(qsfp2_1_rxc_3_int),
    .qsfp1_rx_prbs31_enable_3(qsfp2_1_rx_prbs31_enable_3_int),
    .qsfp1_rx_error_count_3(qsfp2_1_rx_error_count_3_int),
    .qsfp1_tx_clk_4(qsfp2_1_tx_clk_4_int),
    .qsfp1_tx_rst_4(qsfp2_1_tx_rst_4_int),
    .qsfp1_txd_4(qsfp2_1_txd_4_int),
    .qsfp1_txc_4(qsfp2_1_txc_4_int),
    .qsfp1_tx_prbs31_enable_4(qsfp2_1_tx_prbs31_enable_4_int),
    .qsfp1_rx_clk_4(qsfp2_1_rx_clk_4_int),
    .qsfp1_rx_rst_4(qsfp2_1_rx_rst_4_int),
    .qsfp1_rxd_4(qsfp2_1_rxd_4_int),
    .qsfp1_rxc_4(qsfp2_1_rxc_4_int),
    .qsfp1_rx_prbs31_enable_4(qsfp2_1_rx_prbs31_enable_4_int),
    .qsfp1_rx_error_count_4(qsfp2_1_rx_error_count_4_int),

    .qsfp2_tx_clk_1(qsfp2_2_tx_clk_1_int),
    .qsfp2_tx_rst_1(qsfp2_2_tx_rst_1_int),
    .qsfp2_txd_1(qsfp2_2_txd_1_int),
    .qsfp2_txc_1(qsfp2_2_txc_1_int),
    .qsfp2_tx_prbs31_enable_1(qsfp2_2_tx_prbs31_enable_1_int),
    .qsfp2_rx_clk_1(qsfp2_2_rx_clk_1_int),
    .qsfp2_rx_rst_1(qsfp2_2_rx_rst_1_int),
    .qsfp2_rxd_1(qsfp2_2_rxd_1_int),
    .qsfp2_rxc_1(qsfp2_2_rxc_1_int),
    .qsfp2_rx_prbs31_enable_1(qsfp2_2_rx_prbs31_enable_1_int),
    .qsfp2_rx_error_count_1(qsfp2_2_rx_error_count_1_int),
    .qsfp2_tx_clk_2(qsfp2_2_tx_clk_2_int),
    .qsfp2_tx_rst_2(qsfp2_2_tx_rst_2_int),
    .qsfp2_txd_2(qsfp2_2_txd_2_int),
    .qsfp2_txc_2(qsfp2_2_txc_2_int),
    .qsfp2_tx_prbs31_enable_2(qsfp2_2_tx_prbs31_enable_2_int),
    .qsfp2_rx_clk_2(qsfp2_2_rx_clk_2_int),
    .qsfp2_rx_rst_2(qsfp2_2_rx_rst_2_int),
    .qsfp2_rxd_2(qsfp2_2_rxd_2_int),
    .qsfp2_rxc_2(qsfp2_2_rxc_2_int),
    .qsfp2_rx_prbs31_enable_2(qsfp2_2_rx_prbs31_enable_2_int),
    .qsfp2_rx_error_count_2(qsfp2_2_rx_error_count_2_int),
    .qsfp2_tx_clk_3(qsfp2_2_tx_clk_3_int),
    .qsfp2_tx_rst_3(qsfp2_2_tx_rst_3_int),
    .qsfp2_txd_3(qsfp2_2_txd_3_int),
    .qsfp2_txc_3(qsfp2_2_txc_3_int),
    .qsfp2_tx_prbs31_enable_3(qsfp2_2_tx_prbs31_enable_3_int),
    .qsfp2_rx_clk_3(qsfp2_2_rx_clk_3_int),
    .qsfp2_rx_rst_3(qsfp2_2_rx_rst_3_int),
    .qsfp2_rxd_3(qsfp2_2_rxd_3_int),
    .qsfp2_rxc_3(qsfp2_2_rxc_3_int),
    .qsfp2_rx_prbs31_enable_3(qsfp2_2_rx_prbs31_enable_3_int),
    .qsfp2_rx_error_count_3(qsfp2_2_rx_error_count_3_int),
    .qsfp2_tx_clk_4(qsfp2_2_tx_clk_4_int),
    .qsfp2_tx_rst_4(qsfp2_2_tx_rst_4_int),
    .qsfp2_txd_4(qsfp2_2_txd_4_int),
    .qsfp2_txc_4(qsfp2_2_txc_4_int),
    .qsfp2_tx_prbs31_enable_4(qsfp2_2_tx_prbs31_enable_4_int),
    .qsfp2_rx_clk_4(qsfp2_2_rx_clk_4_int),
    .qsfp2_rx_rst_4(qsfp2_2_rx_rst_4_int),
    .qsfp2_rxd_4(qsfp2_2_rxd_4_int),
    .qsfp2_rxc_4(qsfp2_2_rxc_4_int),
    .qsfp2_rx_prbs31_enable_4(qsfp2_2_rx_prbs31_enable_4_int),
    .qsfp2_rx_error_count_4(qsfp2_2_rx_error_count_4_int),

    /*
     * QSPI flash
     */
    .fpga_boot(),
    .qspi_clk(),
    .qspi_0_dq_i(4'h0),
    .qspi_0_dq_o(),
    .qspi_0_dq_oe(),
    .qspi_0_cs()
);

endmodule

