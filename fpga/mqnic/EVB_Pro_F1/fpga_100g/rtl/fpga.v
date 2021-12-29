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
    input  wire         qsfp0_2_mgt_refclk_0_p,
    input  wire         qsfp0_2_mgt_refclk_0_n,

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
    input  wire         qsfp1_2_mgt_refclk_0_p,
    input  wire         qsfp1_2_mgt_refclk_0_n,

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
    input  wire         qsfp2_2_rx4_n,
    input  wire         qsfp2_2_mgt_refclk_0_p,
    input  wire         qsfp2_2_mgt_refclk_0_n
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

parameter AXIS_ETH_DATA_WIDTH = 512;
parameter AXIS_ETH_KEEP_WIDTH = AXIS_ETH_DATA_WIDTH/8;

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

// CMAC
wire                           qsfp0_1_tx_clk_int;
wire                           qsfp0_1_tx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp0_1_tx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp0_1_tx_axis_tkeep_int;
wire                           qsfp0_1_tx_axis_tvalid_int;
wire                           qsfp0_1_tx_axis_tready_int;
wire                           qsfp0_1_tx_axis_tlast_int;
wire                           qsfp0_1_tx_axis_tuser_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp0_1_mac_tx_axis_tdata;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp0_1_mac_tx_axis_tkeep;
wire                           qsfp0_1_mac_tx_axis_tvalid;
wire                           qsfp0_1_mac_tx_axis_tready;
wire                           qsfp0_1_mac_tx_axis_tlast;
wire                           qsfp0_1_mac_tx_axis_tuser;

wire [79:0]                    qsfp0_1_tx_ptp_time_int;
wire [79:0]                    qsfp0_1_tx_ptp_ts_int;
wire                           qsfp0_1_tx_ptp_ts_valid_int;

wire                           qsfp0_1_rx_clk_int;
wire                           qsfp0_1_rx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp0_1_rx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp0_1_rx_axis_tkeep_int;
wire                           qsfp0_1_rx_axis_tvalid_int;
wire                           qsfp0_1_rx_axis_tlast_int;
wire [80+1-1:0]                qsfp0_1_rx_axis_tuser_int;

wire [79:0]                    qsfp0_1_rx_ptp_time_int;

wire                           qsfp0_2_tx_clk_int;
wire                           qsfp0_2_tx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp0_2_tx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp0_2_tx_axis_tkeep_int;
wire                           qsfp0_2_tx_axis_tvalid_int;
wire                           qsfp0_2_tx_axis_tready_int;
wire                           qsfp0_2_tx_axis_tlast_int;
wire                           qsfp0_2_tx_axis_tuser_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp0_2_mac_tx_axis_tdata;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp0_2_mac_tx_axis_tkeep;
wire                           qsfp0_2_mac_tx_axis_tvalid;
wire                           qsfp0_2_mac_tx_axis_tready;
wire                           qsfp0_2_mac_tx_axis_tlast;
wire                           qsfp0_2_mac_tx_axis_tuser;

wire [79:0]                    qsfp0_2_tx_ptp_time_int;
wire [79:0]                    qsfp0_2_tx_ptp_ts_int;
wire                           qsfp0_2_tx_ptp_ts_valid_int;

wire                           qsfp0_2_rx_clk_int;
wire                           qsfp0_2_rx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp0_2_rx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp0_2_rx_axis_tkeep_int;
wire                           qsfp0_2_rx_axis_tvalid_int;
wire                           qsfp0_2_rx_axis_tlast_int;
wire [80+1-1:0]                qsfp0_2_rx_axis_tuser_int;

wire [79:0]                    qsfp0_2_rx_ptp_time_int;

wire qsfp0_1_rx_status;
wire qsfp0_2_rx_status;

wire qsfp0_1_txuserclk2;

assign qsfp0_1_tx_clk_int = qsfp0_1_txuserclk2;
assign qsfp0_1_rx_clk_int = qsfp0_1_txuserclk2;

wire qsfp0_2_txuserclk2;

assign qsfp0_2_tx_clk_int = qsfp0_2_txuserclk2;
assign qsfp0_2_rx_clk_int = qsfp0_2_txuserclk2;

cmac_pad #(
    .DATA_WIDTH(AXIS_ETH_DATA_WIDTH),
    .KEEP_WIDTH(AXIS_ETH_KEEP_WIDTH),
    .USER_WIDTH(1)
)
qsfp0_1_cmac_pad_inst (
    .clk(qsfp0_1_tx_clk_int),
    .rst(qsfp0_1_tx_rst_int),

    .s_axis_tdata(qsfp0_1_tx_axis_tdata_int),
    .s_axis_tkeep(qsfp0_1_tx_axis_tkeep_int),
    .s_axis_tvalid(qsfp0_1_tx_axis_tvalid_int),
    .s_axis_tready(qsfp0_1_tx_axis_tready_int),
    .s_axis_tlast(qsfp0_1_tx_axis_tlast_int),
    .s_axis_tuser(qsfp0_1_tx_axis_tuser_int),

    .m_axis_tdata(qsfp0_1_mac_tx_axis_tdata),
    .m_axis_tkeep(qsfp0_1_mac_tx_axis_tkeep),
    .m_axis_tvalid(qsfp0_1_mac_tx_axis_tvalid),
    .m_axis_tready(qsfp0_1_mac_tx_axis_tready),
    .m_axis_tlast(qsfp0_1_mac_tx_axis_tlast),
    .m_axis_tuser(qsfp0_1_mac_tx_axis_tuser)
);

cmac_usplus_0_0
qsfp0_1_cmac_inst (
    .gt_rxp_in({qsfp0_1_rx4_p, qsfp0_1_rx3_p, qsfp0_1_rx2_p, qsfp0_1_rx1_p}), // input
    .gt_rxn_in({qsfp0_1_rx4_n, qsfp0_1_rx3_n, qsfp0_1_rx2_n, qsfp0_1_rx1_n}), // input
    .gt_txp_out({qsfp0_1_tx4_p, qsfp0_1_tx3_p, qsfp0_1_tx2_p, qsfp0_1_tx1_p}), // output
    .gt_txn_out({qsfp0_1_tx4_n, qsfp0_1_tx3_n, qsfp0_1_tx2_n, qsfp0_1_tx1_n}), // output
    .gt_txusrclk2(qsfp0_1_txuserclk2), // output
    .gt_loopback_in(12'd0), // input [11:0]
    .gt_rxrecclkout(), // output [3:0]
    .gt_powergoodout(), // output [3:0]
    .gt_ref_clk_out(), // output
    .gtwiz_reset_tx_datapath(1'b0), // input
    .gtwiz_reset_rx_datapath(1'b0), // input
    .sys_reset(rst_125mhz_int0), // input
    .gt_ref_clk_p(qsfp0_1_mgt_refclk_0_p), // input
    .gt_ref_clk_n(qsfp0_1_mgt_refclk_0_n), // input
    .init_clk(clk_125mhz_int0), // input

    .rx_axis_tvalid(qsfp0_1_rx_axis_tvalid_int), // output
    .rx_axis_tdata(qsfp0_1_rx_axis_tdata_int), // output [511:0]
    .rx_axis_tlast(qsfp0_1_rx_axis_tlast_int), // output
    .rx_axis_tkeep(qsfp0_1_rx_axis_tkeep_int), // output [63:0]
    .rx_axis_tuser(qsfp0_1_rx_axis_tuser_int[0]), // output

    .rx_otn_bip8_0(), // output [7:0]
    .rx_otn_bip8_1(), // output [7:0]
    .rx_otn_bip8_2(), // output [7:0]
    .rx_otn_bip8_3(), // output [7:0]
    .rx_otn_bip8_4(), // output [7:0]
    .rx_otn_data_0(), // output [65:0]
    .rx_otn_data_1(), // output [65:0]
    .rx_otn_data_2(), // output [65:0]
    .rx_otn_data_3(), // output [65:0]
    .rx_otn_data_4(), // output [65:0]
    .rx_otn_ena(), // output
    .rx_otn_lane0(), // output
    .rx_otn_vlmarker(), // output
    .rx_preambleout(), // output [55:0]
    .usr_rx_reset(qsfp0_1_rx_rst_int), // output
    .gt_rxusrclk2(), // output

    .rx_lane_aligner_fill_0(), // output [6:0]
    .rx_lane_aligner_fill_1(), // output [6:0]
    .rx_lane_aligner_fill_10(), // output [6:0]
    .rx_lane_aligner_fill_11(), // output [6:0]
    .rx_lane_aligner_fill_12(), // output [6:0]
    .rx_lane_aligner_fill_13(), // output [6:0]
    .rx_lane_aligner_fill_14(), // output [6:0]
    .rx_lane_aligner_fill_15(), // output [6:0]
    .rx_lane_aligner_fill_16(), // output [6:0]
    .rx_lane_aligner_fill_17(), // output [6:0]
    .rx_lane_aligner_fill_18(), // output [6:0]
    .rx_lane_aligner_fill_19(), // output [6:0]
    .rx_lane_aligner_fill_2(), // output [6:0]
    .rx_lane_aligner_fill_3(), // output [6:0]
    .rx_lane_aligner_fill_4(), // output [6:0]
    .rx_lane_aligner_fill_5(), // output [6:0]
    .rx_lane_aligner_fill_6(), // output [6:0]
    .rx_lane_aligner_fill_7(), // output [6:0]
    .rx_lane_aligner_fill_8(), // output [6:0]
    .rx_lane_aligner_fill_9(), // output [6:0]
    .rx_ptp_tstamp_out(qsfp0_1_rx_axis_tuser_int[80:1]), // output [79:0]
    .rx_ptp_pcslane_out(), // output [4:0]
    .ctl_rx_systemtimerin(qsfp0_1_rx_ptp_time_int), // input [79:0]
    .stat_rx_aligned(), // output
    .stat_rx_aligned_err(), // output
    .stat_rx_bad_code(), // output [2:0]
    .stat_rx_bad_fcs(), // output [2:0]
    .stat_rx_bad_preamble(), // output
    .stat_rx_bad_sfd(), // output
    .stat_rx_bip_err_0(), // output
    .stat_rx_bip_err_1(), // output
    .stat_rx_bip_err_10(), // output
    .stat_rx_bip_err_11(), // output
    .stat_rx_bip_err_12(), // output
    .stat_rx_bip_err_13(), // output
    .stat_rx_bip_err_14(), // output
    .stat_rx_bip_err_15(), // output
    .stat_rx_bip_err_16(), // output
    .stat_rx_bip_err_17(), // output
    .stat_rx_bip_err_18(), // output
    .stat_rx_bip_err_19(), // output
    .stat_rx_bip_err_2(), // output
    .stat_rx_bip_err_3(), // output
    .stat_rx_bip_err_4(), // output
    .stat_rx_bip_err_5(), // output
    .stat_rx_bip_err_6(), // output
    .stat_rx_bip_err_7(), // output
    .stat_rx_bip_err_8(), // output
    .stat_rx_bip_err_9(), // output
    .stat_rx_block_lock(), // output [19:0]
    .stat_rx_broadcast(), // output
    .stat_rx_fragment(), // output [2:0]
    .stat_rx_framing_err_0(), // output [1:0]
    .stat_rx_framing_err_1(), // output [1:0]
    .stat_rx_framing_err_10(), // output [1:0]
    .stat_rx_framing_err_11(), // output [1:0]
    .stat_rx_framing_err_12(), // output [1:0]
    .stat_rx_framing_err_13(), // output [1:0]
    .stat_rx_framing_err_14(), // output [1:0]
    .stat_rx_framing_err_15(), // output [1:0]
    .stat_rx_framing_err_16(), // output [1:0]
    .stat_rx_framing_err_17(), // output [1:0]
    .stat_rx_framing_err_18(), // output [1:0]
    .stat_rx_framing_err_19(), // output [1:0]
    .stat_rx_framing_err_2(), // output [1:0]
    .stat_rx_framing_err_3(), // output [1:0]
    .stat_rx_framing_err_4(), // output [1:0]
    .stat_rx_framing_err_5(), // output [1:0]
    .stat_rx_framing_err_6(), // output [1:0]
    .stat_rx_framing_err_7(), // output [1:0]
    .stat_rx_framing_err_8(), // output [1:0]
    .stat_rx_framing_err_9(), // output [1:0]
    .stat_rx_framing_err_valid_0(), // output
    .stat_rx_framing_err_valid_1(), // output
    .stat_rx_framing_err_valid_10(), // output
    .stat_rx_framing_err_valid_11(), // output
    .stat_rx_framing_err_valid_12(), // output
    .stat_rx_framing_err_valid_13(), // output
    .stat_rx_framing_err_valid_14(), // output
    .stat_rx_framing_err_valid_15(), // output
    .stat_rx_framing_err_valid_16(), // output
    .stat_rx_framing_err_valid_17(), // output
    .stat_rx_framing_err_valid_18(), // output
    .stat_rx_framing_err_valid_19(), // output
    .stat_rx_framing_err_valid_2(), // output
    .stat_rx_framing_err_valid_3(), // output
    .stat_rx_framing_err_valid_4(), // output
    .stat_rx_framing_err_valid_5(), // output
    .stat_rx_framing_err_valid_6(), // output
    .stat_rx_framing_err_valid_7(), // output
    .stat_rx_framing_err_valid_8(), // output
    .stat_rx_framing_err_valid_9(), // output
    .stat_rx_got_signal_os(), // output
    .stat_rx_hi_ber(), // output
    .stat_rx_inrangeerr(), // output
    .stat_rx_internal_local_fault(), // output
    .stat_rx_jabber(), // output
    .stat_rx_local_fault(), // output
    .stat_rx_mf_err(), // output [19:0]
    .stat_rx_mf_len_err(), // output [19:0]
    .stat_rx_mf_repeat_err(), // output [19:0]
    .stat_rx_misaligned(), // output
    .stat_rx_multicast(), // output
    .stat_rx_oversize(), // output
    .stat_rx_packet_1024_1518_bytes(), // output
    .stat_rx_packet_128_255_bytes(), // output
    .stat_rx_packet_1519_1522_bytes(), // output
    .stat_rx_packet_1523_1548_bytes(), // output
    .stat_rx_packet_1549_2047_bytes(), // output
    .stat_rx_packet_2048_4095_bytes(), // output
    .stat_rx_packet_256_511_bytes(), // output
    .stat_rx_packet_4096_8191_bytes(), // output
    .stat_rx_packet_512_1023_bytes(), // output
    .stat_rx_packet_64_bytes(), // output
    .stat_rx_packet_65_127_bytes(), // output
    .stat_rx_packet_8192_9215_bytes(), // output
    .stat_rx_packet_bad_fcs(), // output
    .stat_rx_packet_large(), // output
    .stat_rx_packet_small(), // output [2:0]

    .ctl_rx_enable(1'b1), // input
    .ctl_rx_force_resync(1'b0), // input
    .ctl_rx_test_pattern(1'b0), // input
    .ctl_rsfec_ieee_error_indication_mode(1'b0), // input
    .ctl_rx_rsfec_enable(1'b1), // input
    .ctl_rx_rsfec_enable_correction(1'b1), // input
    .ctl_rx_rsfec_enable_indication(1'b1), // input
    .core_rx_reset(1'b0), // input
    .rx_clk(qsfp0_1_rx_clk_int), // input

    .stat_rx_received_local_fault(), // output
    .stat_rx_remote_fault(), // output
    .stat_rx_status(qsfp0_1_rx_status), // output
    .stat_rx_stomped_fcs(), // output [2:0]
    .stat_rx_synced(), // output [19:0]
    .stat_rx_synced_err(), // output [19:0]
    .stat_rx_test_pattern_mismatch(), // output [2:0]
    .stat_rx_toolong(), // output
    .stat_rx_total_bytes(), // output [6:0]
    .stat_rx_total_good_bytes(), // output [13:0]
    .stat_rx_total_good_packets(), // output
    .stat_rx_total_packets(), // output [2:0]
    .stat_rx_truncated(), // output
    .stat_rx_undersize(), // output [2:0]
    .stat_rx_unicast(), // output
    .stat_rx_vlan(), // output
    .stat_rx_pcsl_demuxed(), // output [19:0]
    .stat_rx_pcsl_number_0(), // output [4:0]
    .stat_rx_pcsl_number_1(), // output [4:0]
    .stat_rx_pcsl_number_10(), // output [4:0]
    .stat_rx_pcsl_number_11(), // output [4:0]
    .stat_rx_pcsl_number_12(), // output [4:0]
    .stat_rx_pcsl_number_13(), // output [4:0]
    .stat_rx_pcsl_number_14(), // output [4:0]
    .stat_rx_pcsl_number_15(), // output [4:0]
    .stat_rx_pcsl_number_16(), // output [4:0]
    .stat_rx_pcsl_number_17(), // output [4:0]
    .stat_rx_pcsl_number_18(), // output [4:0]
    .stat_rx_pcsl_number_19(), // output [4:0]
    .stat_rx_pcsl_number_2(), // output [4:0]
    .stat_rx_pcsl_number_3(), // output [4:0]
    .stat_rx_pcsl_number_4(), // output [4:0]
    .stat_rx_pcsl_number_5(), // output [4:0]
    .stat_rx_pcsl_number_6(), // output [4:0]
    .stat_rx_pcsl_number_7(), // output [4:0]
    .stat_rx_pcsl_number_8(), // output [4:0]
    .stat_rx_pcsl_number_9(), // output [4:0]
    .stat_rx_rsfec_am_lock0(), // output
    .stat_rx_rsfec_am_lock1(), // output
    .stat_rx_rsfec_am_lock2(), // output
    .stat_rx_rsfec_am_lock3(), // output
    .stat_rx_rsfec_corrected_cw_inc(), // output
    .stat_rx_rsfec_cw_inc(), // output
    .stat_rx_rsfec_err_count0_inc(), // output [2:0]
    .stat_rx_rsfec_err_count1_inc(), // output [2:0]
    .stat_rx_rsfec_err_count2_inc(), // output [2:0]
    .stat_rx_rsfec_err_count3_inc(), // output [2:0]
    .stat_rx_rsfec_hi_ser(), // output
    .stat_rx_rsfec_lane_alignment_status(), // output
    .stat_rx_rsfec_lane_fill_0(), // output [13:0]
    .stat_rx_rsfec_lane_fill_1(), // output [13:0]
    .stat_rx_rsfec_lane_fill_2(), // output [13:0]
    .stat_rx_rsfec_lane_fill_3(), // output [13:0]
    .stat_rx_rsfec_lane_mapping(), // output [7:0]
    .stat_rx_rsfec_uncorrected_cw_inc(), // output

    .ctl_tx_systemtimerin(qsfp0_1_tx_ptp_time_int), // input [79:0]

    .stat_tx_ptp_fifo_read_error(), // output
    .stat_tx_ptp_fifo_write_error(), // output

    .tx_ptp_tstamp_valid_out(qsfp0_1_tx_ptp_ts_valid_int), // output
    .tx_ptp_pcslane_out(), // output [4:0]
    .tx_ptp_tstamp_tag_out(), // output [15:0]
    .tx_ptp_tstamp_out(qsfp0_1_tx_ptp_ts_int), // output [79:0]
    .tx_ptp_1588op_in(2'b10), // input [1:0]
    .tx_ptp_tag_field_in(16'd0), // input [15:0]

    .stat_tx_bad_fcs(), // output
    .stat_tx_broadcast(), // output
    .stat_tx_frame_error(), // output
    .stat_tx_local_fault(), // output
    .stat_tx_multicast(), // output
    .stat_tx_packet_1024_1518_bytes(), // output
    .stat_tx_packet_128_255_bytes(), // output
    .stat_tx_packet_1519_1522_bytes(), // output
    .stat_tx_packet_1523_1548_bytes(), // output
    .stat_tx_packet_1549_2047_bytes(), // output
    .stat_tx_packet_2048_4095_bytes(), // output
    .stat_tx_packet_256_511_bytes(), // output
    .stat_tx_packet_4096_8191_bytes(), // output
    .stat_tx_packet_512_1023_bytes(), // output
    .stat_tx_packet_64_bytes(), // output
    .stat_tx_packet_65_127_bytes(), // output
    .stat_tx_packet_8192_9215_bytes(), // output
    .stat_tx_packet_large(), // output
    .stat_tx_packet_small(), // output
    .stat_tx_total_bytes(), // output [5:0]
    .stat_tx_total_good_bytes(), // output [13:0]
    .stat_tx_total_good_packets(), // output
    .stat_tx_total_packets(), // output
    .stat_tx_unicast(), // output
    .stat_tx_vlan(), // output

    .ctl_tx_enable(1'b1), // input
    .ctl_tx_test_pattern(1'b0), // input
    .ctl_tx_rsfec_enable(1'b1), // input
    .ctl_tx_send_idle(1'b0), // input
    .ctl_tx_send_rfi(1'b0), // input
    .ctl_tx_send_lfi(1'b0), // input
    .core_tx_reset(1'b0), // input

    .tx_axis_tready(qsfp0_1_mac_tx_axis_tready), // output
    .tx_axis_tvalid(qsfp0_1_mac_tx_axis_tvalid), // input
    .tx_axis_tdata(qsfp0_1_mac_tx_axis_tdata), // input [511:0]
    .tx_axis_tlast(qsfp0_1_mac_tx_axis_tlast), // input
    .tx_axis_tkeep(qsfp0_1_mac_tx_axis_tkeep), // input [63:0]
    .tx_axis_tuser(qsfp0_1_mac_tx_axis_tuser), // input

    .tx_ovfout(), // output
    .tx_unfout(), // output
    .tx_preamblein(56'd0), // input [55:0]
    .usr_tx_reset(qsfp0_1_tx_rst_int), // output

    .core_drp_reset(1'b0), // input
    .drp_clk(1'b0), // input
    .drp_addr(10'd0), // input [9:0]
    .drp_di(16'd0), // input [15:0]
    .drp_en(1'b0), // input
    .drp_do(), // output [15:0]
    .drp_rdy(), // output
    .drp_we(1'b0) // input
);

cmac_pad #(
    .DATA_WIDTH(AXIS_ETH_DATA_WIDTH),
    .KEEP_WIDTH(AXIS_ETH_KEEP_WIDTH),
    .USER_WIDTH(1)
)
qsfp0_2_cmac_pad_inst (
    .clk(qsfp0_2_tx_clk_int),
    .rst(qsfp0_2_tx_rst_int),

    .s_axis_tdata(qsfp0_2_tx_axis_tdata_int),
    .s_axis_tkeep(qsfp0_2_tx_axis_tkeep_int),
    .s_axis_tvalid(qsfp0_2_tx_axis_tvalid_int),
    .s_axis_tready(qsfp0_2_tx_axis_tready_int),
    .s_axis_tlast(qsfp0_2_tx_axis_tlast_int),
    .s_axis_tuser(qsfp0_2_tx_axis_tuser_int),

    .m_axis_tdata(qsfp0_2_mac_tx_axis_tdata),
    .m_axis_tkeep(qsfp0_2_mac_tx_axis_tkeep),
    .m_axis_tvalid(qsfp0_2_mac_tx_axis_tvalid),
    .m_axis_tready(qsfp0_2_mac_tx_axis_tready),
    .m_axis_tlast(qsfp0_2_mac_tx_axis_tlast),
    .m_axis_tuser(qsfp0_2_mac_tx_axis_tuser)
);

cmac_usplus_0_1
qsfp0_2_cmac_inst (
    .gt_rxp_in({qsfp0_2_rx4_p, qsfp0_2_rx3_p, qsfp0_2_rx2_p, qsfp0_2_rx1_p}), // input
    .gt_rxn_in({qsfp0_2_rx4_n, qsfp0_2_rx3_n, qsfp0_2_rx2_n, qsfp0_2_rx1_n}), // input
    .gt_txp_out({qsfp0_2_tx4_p, qsfp0_2_tx3_p, qsfp0_2_tx2_p, qsfp0_2_tx1_p}), // output
    .gt_txn_out({qsfp0_2_tx4_n, qsfp0_2_tx3_n, qsfp0_2_tx2_n, qsfp0_2_tx1_n}), // output
    .gt_txusrclk2(qsfp0_2_txuserclk2), // output
    .gt_loopback_in(12'd0), // input [11:0]
    .gt_rxrecclkout(), // output [3:0]
    .gt_powergoodout(), // output [3:0]
    .gt_ref_clk_out(), // output
    .gtwiz_reset_tx_datapath(1'b0), // input
    .gtwiz_reset_rx_datapath(1'b0), // input
    .sys_reset(rst_125mhz_int0), // input
    .gt_ref_clk_p(qsfp0_2_mgt_refclk_0_p), // input
    .gt_ref_clk_n(qsfp0_2_mgt_refclk_0_n), // input
    .init_clk(clk_125mhz_int0), // input

    .rx_axis_tvalid(qsfp0_2_rx_axis_tvalid_int), // output
    .rx_axis_tdata(qsfp0_2_rx_axis_tdata_int), // output [511:0]
    .rx_axis_tlast(qsfp0_2_rx_axis_tlast_int), // output
    .rx_axis_tkeep(qsfp0_2_rx_axis_tkeep_int), // output [63:0]
    .rx_axis_tuser(qsfp0_2_rx_axis_tuser_int[0]), // output

    .rx_otn_bip8_0(), // output [7:0]
    .rx_otn_bip8_1(), // output [7:0]
    .rx_otn_bip8_2(), // output [7:0]
    .rx_otn_bip8_3(), // output [7:0]
    .rx_otn_bip8_4(), // output [7:0]
    .rx_otn_data_0(), // output [65:0]
    .rx_otn_data_1(), // output [65:0]
    .rx_otn_data_2(), // output [65:0]
    .rx_otn_data_3(), // output [65:0]
    .rx_otn_data_4(), // output [65:0]
    .rx_otn_ena(), // output
    .rx_otn_lane0(), // output
    .rx_otn_vlmarker(), // output
    .rx_preambleout(), // output [55:0]
    .usr_rx_reset(qsfp0_2_rx_rst_int), // output
    .gt_rxusrclk2(), // output

    .rx_lane_aligner_fill_0(), // output [6:0]
    .rx_lane_aligner_fill_1(), // output [6:0]
    .rx_lane_aligner_fill_10(), // output [6:0]
    .rx_lane_aligner_fill_11(), // output [6:0]
    .rx_lane_aligner_fill_12(), // output [6:0]
    .rx_lane_aligner_fill_13(), // output [6:0]
    .rx_lane_aligner_fill_14(), // output [6:0]
    .rx_lane_aligner_fill_15(), // output [6:0]
    .rx_lane_aligner_fill_16(), // output [6:0]
    .rx_lane_aligner_fill_17(), // output [6:0]
    .rx_lane_aligner_fill_18(), // output [6:0]
    .rx_lane_aligner_fill_19(), // output [6:0]
    .rx_lane_aligner_fill_2(), // output [6:0]
    .rx_lane_aligner_fill_3(), // output [6:0]
    .rx_lane_aligner_fill_4(), // output [6:0]
    .rx_lane_aligner_fill_5(), // output [6:0]
    .rx_lane_aligner_fill_6(), // output [6:0]
    .rx_lane_aligner_fill_7(), // output [6:0]
    .rx_lane_aligner_fill_8(), // output [6:0]
    .rx_lane_aligner_fill_9(), // output [6:0]
    .rx_ptp_tstamp_out(qsfp0_2_rx_axis_tuser_int[80:1]), // output [79:0]
    .rx_ptp_pcslane_out(), // output [4:0]
    .ctl_rx_systemtimerin(qsfp0_2_rx_ptp_time_int), // input [79:0]

    .stat_rx_aligned(), // output
    .stat_rx_aligned_err(), // output
    .stat_rx_bad_code(), // output [2:0]
    .stat_rx_bad_fcs(), // output [2:0]
    .stat_rx_bad_preamble(), // output
    .stat_rx_bad_sfd(), // output
    .stat_rx_bip_err_0(), // output
    .stat_rx_bip_err_1(), // output
    .stat_rx_bip_err_10(), // output
    .stat_rx_bip_err_11(), // output
    .stat_rx_bip_err_12(), // output
    .stat_rx_bip_err_13(), // output
    .stat_rx_bip_err_14(), // output
    .stat_rx_bip_err_15(), // output
    .stat_rx_bip_err_16(), // output
    .stat_rx_bip_err_17(), // output
    .stat_rx_bip_err_18(), // output
    .stat_rx_bip_err_19(), // output
    .stat_rx_bip_err_2(), // output
    .stat_rx_bip_err_3(), // output
    .stat_rx_bip_err_4(), // output
    .stat_rx_bip_err_5(), // output
    .stat_rx_bip_err_6(), // output
    .stat_rx_bip_err_7(), // output
    .stat_rx_bip_err_8(), // output
    .stat_rx_bip_err_9(), // output
    .stat_rx_block_lock(), // output [19:0]
    .stat_rx_broadcast(), // output
    .stat_rx_fragment(), // output [2:0]
    .stat_rx_framing_err_0(), // output [1:0]
    .stat_rx_framing_err_1(), // output [1:0]
    .stat_rx_framing_err_10(), // output [1:0]
    .stat_rx_framing_err_11(), // output [1:0]
    .stat_rx_framing_err_12(), // output [1:0]
    .stat_rx_framing_err_13(), // output [1:0]
    .stat_rx_framing_err_14(), // output [1:0]
    .stat_rx_framing_err_15(), // output [1:0]
    .stat_rx_framing_err_16(), // output [1:0]
    .stat_rx_framing_err_17(), // output [1:0]
    .stat_rx_framing_err_18(), // output [1:0]
    .stat_rx_framing_err_19(), // output [1:0]
    .stat_rx_framing_err_2(), // output [1:0]
    .stat_rx_framing_err_3(), // output [1:0]
    .stat_rx_framing_err_4(), // output [1:0]
    .stat_rx_framing_err_5(), // output [1:0]
    .stat_rx_framing_err_6(), // output [1:0]
    .stat_rx_framing_err_7(), // output [1:0]
    .stat_rx_framing_err_8(), // output [1:0]
    .stat_rx_framing_err_9(), // output [1:0]
    .stat_rx_framing_err_valid_0(), // output
    .stat_rx_framing_err_valid_1(), // output
    .stat_rx_framing_err_valid_10(), // output
    .stat_rx_framing_err_valid_11(), // output
    .stat_rx_framing_err_valid_12(), // output
    .stat_rx_framing_err_valid_13(), // output
    .stat_rx_framing_err_valid_14(), // output
    .stat_rx_framing_err_valid_15(), // output
    .stat_rx_framing_err_valid_16(), // output
    .stat_rx_framing_err_valid_17(), // output
    .stat_rx_framing_err_valid_18(), // output
    .stat_rx_framing_err_valid_19(), // output
    .stat_rx_framing_err_valid_2(), // output
    .stat_rx_framing_err_valid_3(), // output
    .stat_rx_framing_err_valid_4(), // output
    .stat_rx_framing_err_valid_5(), // output
    .stat_rx_framing_err_valid_6(), // output
    .stat_rx_framing_err_valid_7(), // output
    .stat_rx_framing_err_valid_8(), // output
    .stat_rx_framing_err_valid_9(), // output
    .stat_rx_got_signal_os(), // output
    .stat_rx_hi_ber(), // output
    .stat_rx_inrangeerr(), // output
    .stat_rx_internal_local_fault(), // output
    .stat_rx_jabber(), // output
    .stat_rx_local_fault(), // output
    .stat_rx_mf_err(), // output [19:0]
    .stat_rx_mf_len_err(), // output [19:0]
    .stat_rx_mf_repeat_err(), // output [19:0]
    .stat_rx_misaligned(), // output
    .stat_rx_multicast(), // output
    .stat_rx_oversize(), // output
    .stat_rx_packet_1024_1518_bytes(), // output
    .stat_rx_packet_128_255_bytes(), // output
    .stat_rx_packet_1519_1522_bytes(), // output
    .stat_rx_packet_1523_1548_bytes(), // output
    .stat_rx_packet_1549_2047_bytes(), // output
    .stat_rx_packet_2048_4095_bytes(), // output
    .stat_rx_packet_256_511_bytes(), // output
    .stat_rx_packet_4096_8191_bytes(), // output
    .stat_rx_packet_512_1023_bytes(), // output
    .stat_rx_packet_64_bytes(), // output
    .stat_rx_packet_65_127_bytes(), // output
    .stat_rx_packet_8192_9215_bytes(), // output
    .stat_rx_packet_bad_fcs(), // output
    .stat_rx_packet_large(), // output
    .stat_rx_packet_small(), // output [2:0]

    .ctl_rx_enable(1'b1), // input
    .ctl_rx_force_resync(1'b0), // input
    .ctl_rx_test_pattern(1'b0), // input
    .ctl_rsfec_ieee_error_indication_mode(1'b0), // input
    .ctl_rx_rsfec_enable(1'b1), // input
    .ctl_rx_rsfec_enable_correction(1'b1), // input
    .ctl_rx_rsfec_enable_indication(1'b1), // input
    .core_rx_reset(1'b0), // input
    .rx_clk(qsfp0_2_rx_clk_int), // input

    .stat_rx_received_local_fault(), // output
    .stat_rx_remote_fault(), // output
    .stat_rx_status(qsfp0_2_rx_status), // output
    .stat_rx_stomped_fcs(), // output [2:0]
    .stat_rx_synced(), // output [19:0]
    .stat_rx_synced_err(), // output [19:0]
    .stat_rx_test_pattern_mismatch(), // output [2:0]
    .stat_rx_toolong(), // output
    .stat_rx_total_bytes(), // output [6:0]
    .stat_rx_total_good_bytes(), // output [13:0]
    .stat_rx_total_good_packets(), // output
    .stat_rx_total_packets(), // output [2:0]
    .stat_rx_truncated(), // output
    .stat_rx_undersize(), // output [2:0]
    .stat_rx_unicast(), // output
    .stat_rx_vlan(), // output
    .stat_rx_pcsl_demuxed(), // output [19:0]
    .stat_rx_pcsl_number_0(), // output [4:0]
    .stat_rx_pcsl_number_1(), // output [4:0]
    .stat_rx_pcsl_number_10(), // output [4:0]
    .stat_rx_pcsl_number_11(), // output [4:0]
    .stat_rx_pcsl_number_12(), // output [4:0]
    .stat_rx_pcsl_number_13(), // output [4:0]
    .stat_rx_pcsl_number_14(), // output [4:0]
    .stat_rx_pcsl_number_15(), // output [4:0]
    .stat_rx_pcsl_number_16(), // output [4:0]
    .stat_rx_pcsl_number_17(), // output [4:0]
    .stat_rx_pcsl_number_18(), // output [4:0]
    .stat_rx_pcsl_number_19(), // output [4:0]
    .stat_rx_pcsl_number_2(), // output [4:0]
    .stat_rx_pcsl_number_3(), // output [4:0]
    .stat_rx_pcsl_number_4(), // output [4:0]
    .stat_rx_pcsl_number_5(), // output [4:0]
    .stat_rx_pcsl_number_6(), // output [4:0]
    .stat_rx_pcsl_number_7(), // output [4:0]
    .stat_rx_pcsl_number_8(), // output [4:0]
    .stat_rx_pcsl_number_9(), // output [4:0]
    .stat_rx_rsfec_am_lock0(), // output
    .stat_rx_rsfec_am_lock1(), // output
    .stat_rx_rsfec_am_lock2(), // output
    .stat_rx_rsfec_am_lock3(), // output
    .stat_rx_rsfec_corrected_cw_inc(), // output
    .stat_rx_rsfec_cw_inc(), // output
    .stat_rx_rsfec_err_count0_inc(), // output [2:0]
    .stat_rx_rsfec_err_count1_inc(), // output [2:0]
    .stat_rx_rsfec_err_count2_inc(), // output [2:0]
    .stat_rx_rsfec_err_count3_inc(), // output [2:0]
    .stat_rx_rsfec_hi_ser(), // output
    .stat_rx_rsfec_lane_alignment_status(), // output
    .stat_rx_rsfec_lane_fill_0(), // output [13:0]
    .stat_rx_rsfec_lane_fill_1(), // output [13:0]
    .stat_rx_rsfec_lane_fill_2(), // output [13:0]
    .stat_rx_rsfec_lane_fill_3(), // output [13:0]
    .stat_rx_rsfec_lane_mapping(), // output [7:0]
    .stat_rx_rsfec_uncorrected_cw_inc(), // output

    .ctl_tx_systemtimerin(qsfp0_2_tx_ptp_time_int), // input [79:0]

    .stat_tx_ptp_fifo_read_error(), // output
    .stat_tx_ptp_fifo_write_error(), // output

    .tx_ptp_tstamp_valid_out(qsfp0_2_tx_ptp_ts_valid_int), // output
    .tx_ptp_pcslane_out(), // output [4:0]
    .tx_ptp_tstamp_tag_out(), // output [15:0]
    .tx_ptp_tstamp_out(qsfp0_2_tx_ptp_ts_int), // output [79:0]
    .tx_ptp_1588op_in(2'b10), // input [1:0]
    .tx_ptp_tag_field_in(16'd0), // input [15:0]
    .stat_tx_bad_fcs(), // output
    .stat_tx_broadcast(), // output
    .stat_tx_frame_error(), // output
    .stat_tx_local_fault(), // output
    .stat_tx_multicast(), // output
    .stat_tx_packet_1024_1518_bytes(), // output
    .stat_tx_packet_128_255_bytes(), // output
    .stat_tx_packet_1519_1522_bytes(), // output
    .stat_tx_packet_1523_1548_bytes(), // output
    .stat_tx_packet_1549_2047_bytes(), // output
    .stat_tx_packet_2048_4095_bytes(), // output
    .stat_tx_packet_256_511_bytes(), // output
    .stat_tx_packet_4096_8191_bytes(), // output
    .stat_tx_packet_512_1023_bytes(), // output
    .stat_tx_packet_64_bytes(), // output
    .stat_tx_packet_65_127_bytes(), // output
    .stat_tx_packet_8192_9215_bytes(), // output
    .stat_tx_packet_large(), // output
    .stat_tx_packet_small(), // output
    .stat_tx_total_bytes(), // output [5:0]
    .stat_tx_total_good_bytes(), // output [13:0]
    .stat_tx_total_good_packets(), // output
    .stat_tx_total_packets(), // output
    .stat_tx_unicast(), // output
    .stat_tx_vlan(), // output

    .ctl_tx_enable(1'b1), // input
    .ctl_tx_test_pattern(1'b0), // input
    .ctl_tx_rsfec_enable(1'b1), // input
    .ctl_tx_send_idle(1'b0), // input
    .ctl_tx_send_rfi(1'b0), // input
    .ctl_tx_send_lfi(1'b0), // input
    .core_tx_reset(1'b0), // input

    .tx_axis_tready(qsfp0_2_mac_tx_axis_tready), // output
    .tx_axis_tvalid(qsfp0_2_mac_tx_axis_tvalid), // input
    .tx_axis_tdata(qsfp0_2_mac_tx_axis_tdata), // input [511:0]
    .tx_axis_tlast(qsfp0_2_mac_tx_axis_tlast), // input
    .tx_axis_tkeep(qsfp0_2_mac_tx_axis_tkeep), // input [63:0]
    .tx_axis_tuser(qsfp0_2_mac_tx_axis_tuser), // input

    .tx_ovfout(), // output
    .tx_unfout(), // output
    .tx_preamblein(56'd0), // input [55:0]
    .usr_tx_reset(qsfp0_2_tx_rst_int), // output

    .core_drp_reset(1'b0), // input
    .drp_clk(1'b0), // input
    .drp_addr(10'd0), // input [9:0]
    .drp_di(16'd0), // input [15:0]
    .drp_en(1'b0), // input
    .drp_do(), // output [15:0]
    .drp_rdy(), // output
    .drp_we(1'b0) // input
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
     * GPIO NC
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
    .qsfp1_tx_clk(qsfp0_1_tx_clk_int),
    .qsfp1_tx_rst(qsfp0_1_tx_rst_int),
    .qsfp1_tx_axis_tdata(qsfp0_1_tx_axis_tdata_int),
    .qsfp1_tx_axis_tkeep(qsfp0_1_tx_axis_tkeep_int),
    .qsfp1_tx_axis_tvalid(qsfp0_1_tx_axis_tvalid_int),
    .qsfp1_tx_axis_tready(qsfp0_1_tx_axis_tready_int),
    .qsfp1_tx_axis_tlast(qsfp0_1_tx_axis_tlast_int),
    .qsfp1_tx_axis_tuser(qsfp0_1_tx_axis_tuser_int),
    .qsfp1_tx_ptp_time(qsfp0_1_tx_ptp_time_int),
    .qsfp1_tx_ptp_ts(qsfp0_1_tx_ptp_ts_int),
    .qsfp1_tx_ptp_ts_valid(qsfp0_1_tx_ptp_ts_valid_int),
    .qsfp1_rx_clk(qsfp0_1_rx_clk_int),
    .qsfp1_rx_rst(qsfp0_1_rx_rst_int),
    .qsfp1_rx_axis_tdata(qsfp0_1_rx_axis_tdata_int),
    .qsfp1_rx_axis_tkeep(qsfp0_1_rx_axis_tkeep_int),
    .qsfp1_rx_axis_tvalid(qsfp0_1_rx_axis_tvalid_int),
    .qsfp1_rx_axis_tlast(qsfp0_1_rx_axis_tlast_int),
    .qsfp1_rx_axis_tuser(qsfp0_1_rx_axis_tuser_int),
    .qsfp1_rx_ptp_time(qsfp0_1_rx_ptp_time_int),

    .qsfp2_tx_clk(qsfp0_2_tx_clk_int),
    .qsfp2_tx_rst(qsfp0_2_tx_rst_int),
    .qsfp2_tx_axis_tdata(qsfp0_2_tx_axis_tdata_int),
    .qsfp2_tx_axis_tkeep(qsfp0_2_tx_axis_tkeep_int),
    .qsfp2_tx_axis_tvalid(qsfp0_2_tx_axis_tvalid_int),
    .qsfp2_tx_axis_tready(qsfp0_2_tx_axis_tready_int),
    .qsfp2_tx_axis_tlast(qsfp0_2_tx_axis_tlast_int),
    .qsfp2_tx_axis_tuser(qsfp0_2_tx_axis_tuser_int),
    .qsfp2_tx_ptp_time(qsfp0_2_tx_ptp_time_int),
    .qsfp2_tx_ptp_ts(qsfp0_2_tx_ptp_ts_int),
    .qsfp2_tx_ptp_ts_valid(qsfp0_2_tx_ptp_ts_valid_int),
    .qsfp2_rx_clk(qsfp0_2_rx_clk_int),
    .qsfp2_rx_rst(qsfp0_2_rx_rst_int),
    .qsfp2_rx_axis_tdata(qsfp0_2_rx_axis_tdata_int),
    .qsfp2_rx_axis_tkeep(qsfp0_2_rx_axis_tkeep_int),
    .qsfp2_rx_axis_tvalid(qsfp0_2_rx_axis_tvalid_int),
    .qsfp2_rx_axis_tlast(qsfp0_2_rx_axis_tlast_int),
    .qsfp2_rx_axis_tuser(qsfp0_2_rx_axis_tuser_int),
    .qsfp2_rx_ptp_time(qsfp0_2_rx_ptp_time_int),


    /*
     * QSPI flash NC
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

// CMAC
wire                           qsfp1_1_tx_clk_int;
wire                           qsfp1_1_tx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp1_1_tx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp1_1_tx_axis_tkeep_int;
wire                           qsfp1_1_tx_axis_tvalid_int;
wire                           qsfp1_1_tx_axis_tready_int;
wire                           qsfp1_1_tx_axis_tlast_int;
wire                           qsfp1_1_tx_axis_tuser_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp1_1_mac_tx_axis_tdata;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp1_1_mac_tx_axis_tkeep;
wire                           qsfp1_1_mac_tx_axis_tvalid;
wire                           qsfp1_1_mac_tx_axis_tready;
wire                           qsfp1_1_mac_tx_axis_tlast;
wire                           qsfp1_1_mac_tx_axis_tuser;

wire [79:0]                    qsfp1_1_tx_ptp_time_int;
wire [79:0]                    qsfp1_1_tx_ptp_ts_int;
wire                           qsfp1_1_tx_ptp_ts_valid_int;

wire                           qsfp1_1_rx_clk_int;
wire                           qsfp1_1_rx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp1_1_rx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp1_1_rx_axis_tkeep_int;
wire                           qsfp1_1_rx_axis_tvalid_int;
wire                           qsfp1_1_rx_axis_tlast_int;
wire [80+1-1:0]                qsfp1_1_rx_axis_tuser_int;

wire [79:0]                    qsfp1_1_rx_ptp_time_int;

wire                           qsfp1_2_tx_clk_int;
wire                           qsfp1_2_tx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp1_2_tx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp1_2_tx_axis_tkeep_int;
wire                           qsfp1_2_tx_axis_tvalid_int;
wire                           qsfp1_2_tx_axis_tready_int;
wire                           qsfp1_2_tx_axis_tlast_int;
wire                           qsfp1_2_tx_axis_tuser_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp1_2_mac_tx_axis_tdata;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp1_2_mac_tx_axis_tkeep;
wire                           qsfp1_2_mac_tx_axis_tvalid;
wire                           qsfp1_2_mac_tx_axis_tready;
wire                           qsfp1_2_mac_tx_axis_tlast;
wire                           qsfp1_2_mac_tx_axis_tuser;

wire [79:0]                    qsfp1_2_tx_ptp_time_int;
wire [79:0]                    qsfp1_2_tx_ptp_ts_int;
wire                           qsfp1_2_tx_ptp_ts_valid_int;

wire                           qsfp1_2_rx_clk_int;
wire                           qsfp1_2_rx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp1_2_rx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp1_2_rx_axis_tkeep_int;
wire                           qsfp1_2_rx_axis_tvalid_int;
wire                           qsfp1_2_rx_axis_tlast_int;
wire [80+1-1:0]                qsfp1_2_rx_axis_tuser_int;

wire [79:0]                    qsfp1_2_rx_ptp_time_int;

wire qsfp1_1_rx_status;
wire qsfp1_2_rx_status;

wire qsfp1_1_txuserclk2;

assign qsfp1_1_tx_clk_int = qsfp1_1_txuserclk2;
assign qsfp1_1_rx_clk_int = qsfp1_1_txuserclk2;

wire qsfp1_2_txuserclk2;

assign qsfp1_2_tx_clk_int = qsfp1_2_txuserclk2;
assign qsfp1_2_rx_clk_int = qsfp1_2_txuserclk2;

cmac_pad #(
    .DATA_WIDTH(AXIS_ETH_DATA_WIDTH),
    .KEEP_WIDTH(AXIS_ETH_KEEP_WIDTH),
    .USER_WIDTH(1)
)
qsfp1_1_cmac_pad_inst (
    .clk(qsfp1_1_tx_clk_int),
    .rst(qsfp1_1_tx_rst_int),

    .s_axis_tdata(qsfp1_1_tx_axis_tdata_int),
    .s_axis_tkeep(qsfp1_1_tx_axis_tkeep_int),
    .s_axis_tvalid(qsfp1_1_tx_axis_tvalid_int),
    .s_axis_tready(qsfp1_1_tx_axis_tready_int),
    .s_axis_tlast(qsfp1_1_tx_axis_tlast_int),
    .s_axis_tuser(qsfp1_1_tx_axis_tuser_int),

    .m_axis_tdata(qsfp1_1_mac_tx_axis_tdata),
    .m_axis_tkeep(qsfp1_1_mac_tx_axis_tkeep),
    .m_axis_tvalid(qsfp1_1_mac_tx_axis_tvalid),
    .m_axis_tready(qsfp1_1_mac_tx_axis_tready),
    .m_axis_tlast(qsfp1_1_mac_tx_axis_tlast),
    .m_axis_tuser(qsfp1_1_mac_tx_axis_tuser)
);

cmac_usplus_1_0
qsfp1_1_cmac_inst (
    .gt_rxp_in({qsfp1_1_rx4_p, qsfp1_1_rx3_p, qsfp1_1_rx2_p, qsfp1_1_rx1_p}), // input
    .gt_rxn_in({qsfp1_1_rx4_n, qsfp1_1_rx3_n, qsfp1_1_rx2_n, qsfp1_1_rx1_n}), // input
    .gt_txp_out({qsfp1_1_tx4_p, qsfp1_1_tx3_p, qsfp1_1_tx2_p, qsfp1_1_tx1_p}), // output
    .gt_txn_out({qsfp1_1_tx4_n, qsfp1_1_tx3_n, qsfp1_1_tx2_n, qsfp1_1_tx1_n}), // output
    .gt_txusrclk2(qsfp1_1_txuserclk2), // output
    .gt_loopback_in(12'd0), // input [11:0]
    .gt_rxrecclkout(), // output [3:0]
    .gt_powergoodout(), // output [3:0]
    .gt_ref_clk_out(), // output
    .gtwiz_reset_tx_datapath(1'b0), // input
    .gtwiz_reset_rx_datapath(1'b0), // input
    .sys_reset(rst_125mhz_int1), // input
    .gt_ref_clk_p(qsfp1_1_mgt_refclk_0_p), // input
    .gt_ref_clk_n(qsfp1_1_mgt_refclk_0_n), // input
    .init_clk(clk_125mhz_int1), // input

    .rx_axis_tvalid(qsfp1_1_rx_axis_tvalid_int), // output
    .rx_axis_tdata(qsfp1_1_rx_axis_tdata_int), // output [511:0]
    .rx_axis_tlast(qsfp1_1_rx_axis_tlast_int), // output
    .rx_axis_tkeep(qsfp1_1_rx_axis_tkeep_int), // output [63:0]
    .rx_axis_tuser(qsfp1_1_rx_axis_tuser_int[0]), // output

    .rx_otn_bip8_0(), // output [7:0]
    .rx_otn_bip8_1(), // output [7:0]
    .rx_otn_bip8_2(), // output [7:0]
    .rx_otn_bip8_3(), // output [7:0]
    .rx_otn_bip8_4(), // output [7:0]
    .rx_otn_data_0(), // output [65:0]
    .rx_otn_data_1(), // output [65:0]
    .rx_otn_data_2(), // output [65:0]
    .rx_otn_data_3(), // output [65:0]
    .rx_otn_data_4(), // output [65:0]
    .rx_otn_ena(), // output
    .rx_otn_lane0(), // output
    .rx_otn_vlmarker(), // output
    .rx_preambleout(), // output [55:0]
    .usr_rx_reset(qsfp1_1_rx_rst_int), // output
    .gt_rxusrclk2(), // output

    .rx_lane_aligner_fill_0(), // output [6:0]
    .rx_lane_aligner_fill_1(), // output [6:0]
    .rx_lane_aligner_fill_10(), // output [6:0]
    .rx_lane_aligner_fill_11(), // output [6:0]
    .rx_lane_aligner_fill_12(), // output [6:0]
    .rx_lane_aligner_fill_13(), // output [6:0]
    .rx_lane_aligner_fill_14(), // output [6:0]
    .rx_lane_aligner_fill_15(), // output [6:0]
    .rx_lane_aligner_fill_16(), // output [6:0]
    .rx_lane_aligner_fill_17(), // output [6:0]
    .rx_lane_aligner_fill_18(), // output [6:0]
    .rx_lane_aligner_fill_19(), // output [6:0]
    .rx_lane_aligner_fill_2(), // output [6:0]
    .rx_lane_aligner_fill_3(), // output [6:0]
    .rx_lane_aligner_fill_4(), // output [6:0]
    .rx_lane_aligner_fill_5(), // output [6:0]
    .rx_lane_aligner_fill_6(), // output [6:0]
    .rx_lane_aligner_fill_7(), // output [6:0]
    .rx_lane_aligner_fill_8(), // output [6:0]
    .rx_lane_aligner_fill_9(), // output [6:0]
    .rx_ptp_tstamp_out(qsfp1_1_rx_axis_tuser_int[80:1]), // output [79:0]
    .rx_ptp_pcslane_out(), // output [4:0]
    .ctl_rx_systemtimerin(qsfp1_1_rx_ptp_time_int), // input [79:0]
    .stat_rx_aligned(), // output
    .stat_rx_aligned_err(), // output
    .stat_rx_bad_code(), // output [2:0]
    .stat_rx_bad_fcs(), // output [2:0]
    .stat_rx_bad_preamble(), // output
    .stat_rx_bad_sfd(), // output
    .stat_rx_bip_err_0(), // output
    .stat_rx_bip_err_1(), // output
    .stat_rx_bip_err_10(), // output
    .stat_rx_bip_err_11(), // output
    .stat_rx_bip_err_12(), // output
    .stat_rx_bip_err_13(), // output
    .stat_rx_bip_err_14(), // output
    .stat_rx_bip_err_15(), // output
    .stat_rx_bip_err_16(), // output
    .stat_rx_bip_err_17(), // output
    .stat_rx_bip_err_18(), // output
    .stat_rx_bip_err_19(), // output
    .stat_rx_bip_err_2(), // output
    .stat_rx_bip_err_3(), // output
    .stat_rx_bip_err_4(), // output
    .stat_rx_bip_err_5(), // output
    .stat_rx_bip_err_6(), // output
    .stat_rx_bip_err_7(), // output
    .stat_rx_bip_err_8(), // output
    .stat_rx_bip_err_9(), // output
    .stat_rx_block_lock(), // output [19:0]
    .stat_rx_broadcast(), // output
    .stat_rx_fragment(), // output [2:0]
    .stat_rx_framing_err_0(), // output [1:0]
    .stat_rx_framing_err_1(), // output [1:0]
    .stat_rx_framing_err_10(), // output [1:0]
    .stat_rx_framing_err_11(), // output [1:0]
    .stat_rx_framing_err_12(), // output [1:0]
    .stat_rx_framing_err_13(), // output [1:0]
    .stat_rx_framing_err_14(), // output [1:0]
    .stat_rx_framing_err_15(), // output [1:0]
    .stat_rx_framing_err_16(), // output [1:0]
    .stat_rx_framing_err_17(), // output [1:0]
    .stat_rx_framing_err_18(), // output [1:0]
    .stat_rx_framing_err_19(), // output [1:0]
    .stat_rx_framing_err_2(), // output [1:0]
    .stat_rx_framing_err_3(), // output [1:0]
    .stat_rx_framing_err_4(), // output [1:0]
    .stat_rx_framing_err_5(), // output [1:0]
    .stat_rx_framing_err_6(), // output [1:0]
    .stat_rx_framing_err_7(), // output [1:0]
    .stat_rx_framing_err_8(), // output [1:0]
    .stat_rx_framing_err_9(), // output [1:0]
    .stat_rx_framing_err_valid_0(), // output
    .stat_rx_framing_err_valid_1(), // output
    .stat_rx_framing_err_valid_10(), // output
    .stat_rx_framing_err_valid_11(), // output
    .stat_rx_framing_err_valid_12(), // output
    .stat_rx_framing_err_valid_13(), // output
    .stat_rx_framing_err_valid_14(), // output
    .stat_rx_framing_err_valid_15(), // output
    .stat_rx_framing_err_valid_16(), // output
    .stat_rx_framing_err_valid_17(), // output
    .stat_rx_framing_err_valid_18(), // output
    .stat_rx_framing_err_valid_19(), // output
    .stat_rx_framing_err_valid_2(), // output
    .stat_rx_framing_err_valid_3(), // output
    .stat_rx_framing_err_valid_4(), // output
    .stat_rx_framing_err_valid_5(), // output
    .stat_rx_framing_err_valid_6(), // output
    .stat_rx_framing_err_valid_7(), // output
    .stat_rx_framing_err_valid_8(), // output
    .stat_rx_framing_err_valid_9(), // output
    .stat_rx_got_signal_os(), // output
    .stat_rx_hi_ber(), // output
    .stat_rx_inrangeerr(), // output
    .stat_rx_internal_local_fault(), // output
    .stat_rx_jabber(), // output
    .stat_rx_local_fault(), // output
    .stat_rx_mf_err(), // output [19:0]
    .stat_rx_mf_len_err(), // output [19:0]
    .stat_rx_mf_repeat_err(), // output [19:0]
    .stat_rx_misaligned(), // output
    .stat_rx_multicast(), // output
    .stat_rx_oversize(), // output
    .stat_rx_packet_1024_1518_bytes(), // output
    .stat_rx_packet_128_255_bytes(), // output
    .stat_rx_packet_1519_1522_bytes(), // output
    .stat_rx_packet_1523_1548_bytes(), // output
    .stat_rx_packet_1549_2047_bytes(), // output
    .stat_rx_packet_2048_4095_bytes(), // output
    .stat_rx_packet_256_511_bytes(), // output
    .stat_rx_packet_4096_8191_bytes(), // output
    .stat_rx_packet_512_1023_bytes(), // output
    .stat_rx_packet_64_bytes(), // output
    .stat_rx_packet_65_127_bytes(), // output
    .stat_rx_packet_8192_9215_bytes(), // output
    .stat_rx_packet_bad_fcs(), // output
    .stat_rx_packet_large(), // output
    .stat_rx_packet_small(), // output [2:0]

    .ctl_rx_enable(1'b1), // input
    .ctl_rx_force_resync(1'b0), // input
    .ctl_rx_test_pattern(1'b0), // input
    .ctl_rsfec_ieee_error_indication_mode(1'b0), // input
    .ctl_rx_rsfec_enable(1'b1), // input
    .ctl_rx_rsfec_enable_correction(1'b1), // input
    .ctl_rx_rsfec_enable_indication(1'b1), // input
    .core_rx_reset(1'b0), // input
    .rx_clk(qsfp1_1_rx_clk_int), // input

    .stat_rx_received_local_fault(), // output
    .stat_rx_remote_fault(), // output
    .stat_rx_status(qsfp1_1_rx_status), // output
    .stat_rx_stomped_fcs(), // output [2:0]
    .stat_rx_synced(), // output [19:0]
    .stat_rx_synced_err(), // output [19:0]
    .stat_rx_test_pattern_mismatch(), // output [2:0]
    .stat_rx_toolong(), // output
    .stat_rx_total_bytes(), // output [6:0]
    .stat_rx_total_good_bytes(), // output [13:0]
    .stat_rx_total_good_packets(), // output
    .stat_rx_total_packets(), // output [2:0]
    .stat_rx_truncated(), // output
    .stat_rx_undersize(), // output [2:0]
    .stat_rx_unicast(), // output
    .stat_rx_vlan(), // output
    .stat_rx_pcsl_demuxed(), // output [19:0]
    .stat_rx_pcsl_number_0(), // output [4:0]
    .stat_rx_pcsl_number_1(), // output [4:0]
    .stat_rx_pcsl_number_10(), // output [4:0]
    .stat_rx_pcsl_number_11(), // output [4:0]
    .stat_rx_pcsl_number_12(), // output [4:0]
    .stat_rx_pcsl_number_13(), // output [4:0]
    .stat_rx_pcsl_number_14(), // output [4:0]
    .stat_rx_pcsl_number_15(), // output [4:0]
    .stat_rx_pcsl_number_16(), // output [4:0]
    .stat_rx_pcsl_number_17(), // output [4:0]
    .stat_rx_pcsl_number_18(), // output [4:0]
    .stat_rx_pcsl_number_19(), // output [4:0]
    .stat_rx_pcsl_number_2(), // output [4:0]
    .stat_rx_pcsl_number_3(), // output [4:0]
    .stat_rx_pcsl_number_4(), // output [4:0]
    .stat_rx_pcsl_number_5(), // output [4:0]
    .stat_rx_pcsl_number_6(), // output [4:0]
    .stat_rx_pcsl_number_7(), // output [4:0]
    .stat_rx_pcsl_number_8(), // output [4:0]
    .stat_rx_pcsl_number_9(), // output [4:0]
    .stat_rx_rsfec_am_lock0(), // output
    .stat_rx_rsfec_am_lock1(), // output
    .stat_rx_rsfec_am_lock2(), // output
    .stat_rx_rsfec_am_lock3(), // output
    .stat_rx_rsfec_corrected_cw_inc(), // output
    .stat_rx_rsfec_cw_inc(), // output
    .stat_rx_rsfec_err_count0_inc(), // output [2:0]
    .stat_rx_rsfec_err_count1_inc(), // output [2:0]
    .stat_rx_rsfec_err_count2_inc(), // output [2:0]
    .stat_rx_rsfec_err_count3_inc(), // output [2:0]
    .stat_rx_rsfec_hi_ser(), // output
    .stat_rx_rsfec_lane_alignment_status(), // output
    .stat_rx_rsfec_lane_fill_0(), // output [13:0]
    .stat_rx_rsfec_lane_fill_1(), // output [13:0]
    .stat_rx_rsfec_lane_fill_2(), // output [13:0]
    .stat_rx_rsfec_lane_fill_3(), // output [13:0]
    .stat_rx_rsfec_lane_mapping(), // output [7:0]
    .stat_rx_rsfec_uncorrected_cw_inc(), // output

    .ctl_tx_systemtimerin(qsfp1_1_tx_ptp_time_int), // input [79:0]

    .stat_tx_ptp_fifo_read_error(), // output
    .stat_tx_ptp_fifo_write_error(), // output

    .tx_ptp_tstamp_valid_out(qsfp1_1_tx_ptp_ts_valid_int), // output
    .tx_ptp_pcslane_out(), // output [4:0]
    .tx_ptp_tstamp_tag_out(), // output [15:0]
    .tx_ptp_tstamp_out(qsfp1_1_tx_ptp_ts_int), // output [79:0]
    .tx_ptp_1588op_in(2'b10), // input [1:0]
    .tx_ptp_tag_field_in(16'd0), // input [15:0]

    .stat_tx_bad_fcs(), // output
    .stat_tx_broadcast(), // output
    .stat_tx_frame_error(), // output
    .stat_tx_local_fault(), // output
    .stat_tx_multicast(), // output
    .stat_tx_packet_1024_1518_bytes(), // output
    .stat_tx_packet_128_255_bytes(), // output
    .stat_tx_packet_1519_1522_bytes(), // output
    .stat_tx_packet_1523_1548_bytes(), // output
    .stat_tx_packet_1549_2047_bytes(), // output
    .stat_tx_packet_2048_4095_bytes(), // output
    .stat_tx_packet_256_511_bytes(), // output
    .stat_tx_packet_4096_8191_bytes(), // output
    .stat_tx_packet_512_1023_bytes(), // output
    .stat_tx_packet_64_bytes(), // output
    .stat_tx_packet_65_127_bytes(), // output
    .stat_tx_packet_8192_9215_bytes(), // output
    .stat_tx_packet_large(), // output
    .stat_tx_packet_small(), // output
    .stat_tx_total_bytes(), // output [5:0]
    .stat_tx_total_good_bytes(), // output [13:0]
    .stat_tx_total_good_packets(), // output
    .stat_tx_total_packets(), // output
    .stat_tx_unicast(), // output
    .stat_tx_vlan(), // output

    .ctl_tx_enable(1'b1), // input
    .ctl_tx_test_pattern(1'b0), // input
    .ctl_tx_rsfec_enable(1'b1), // input
    .ctl_tx_send_idle(1'b0), // input
    .ctl_tx_send_rfi(1'b0), // input
    .ctl_tx_send_lfi(1'b0), // input
    .core_tx_reset(1'b0), // input

    .tx_axis_tready(qsfp1_1_mac_tx_axis_tready), // output
    .tx_axis_tvalid(qsfp1_1_mac_tx_axis_tvalid), // input
    .tx_axis_tdata(qsfp1_1_mac_tx_axis_tdata), // input [511:0]
    .tx_axis_tlast(qsfp1_1_mac_tx_axis_tlast), // input
    .tx_axis_tkeep(qsfp1_1_mac_tx_axis_tkeep), // input [63:0]
    .tx_axis_tuser(qsfp1_1_mac_tx_axis_tuser), // input

    .tx_ovfout(), // output
    .tx_unfout(), // output
    .tx_preamblein(56'd0), // input [55:0]
    .usr_tx_reset(qsfp1_1_tx_rst_int), // output

    .core_drp_reset(1'b0), // input
    .drp_clk(1'b0), // input
    .drp_addr(10'd0), // input [9:0]
    .drp_di(16'd0), // input [15:0]
    .drp_en(1'b0), // input
    .drp_do(), // output [15:0]
    .drp_rdy(), // output
    .drp_we(1'b0) // input
);

cmac_pad #(
    .DATA_WIDTH(AXIS_ETH_DATA_WIDTH),
    .KEEP_WIDTH(AXIS_ETH_KEEP_WIDTH),
    .USER_WIDTH(1)
)
qsfp1_2_cmac_pad_inst (
    .clk(qsfp1_2_tx_clk_int),
    .rst(qsfp1_2_tx_rst_int),

    .s_axis_tdata(qsfp1_2_tx_axis_tdata_int),
    .s_axis_tkeep(qsfp1_2_tx_axis_tkeep_int),
    .s_axis_tvalid(qsfp1_2_tx_axis_tvalid_int),
    .s_axis_tready(qsfp1_2_tx_axis_tready_int),
    .s_axis_tlast(qsfp1_2_tx_axis_tlast_int),
    .s_axis_tuser(qsfp1_2_tx_axis_tuser_int),

    .m_axis_tdata(qsfp1_2_mac_tx_axis_tdata),
    .m_axis_tkeep(qsfp1_2_mac_tx_axis_tkeep),
    .m_axis_tvalid(qsfp1_2_mac_tx_axis_tvalid),
    .m_axis_tready(qsfp1_2_mac_tx_axis_tready),
    .m_axis_tlast(qsfp1_2_mac_tx_axis_tlast),
    .m_axis_tuser(qsfp1_2_mac_tx_axis_tuser)
);

cmac_usplus_1_1
qsfp1_2_cmac_inst (
    .gt_rxp_in({qsfp1_2_rx4_p, qsfp1_2_rx3_p, qsfp1_2_rx2_p, qsfp1_2_rx1_p}), // input
    .gt_rxn_in({qsfp1_2_rx4_n, qsfp1_2_rx3_n, qsfp1_2_rx2_n, qsfp1_2_rx1_n}), // input
    .gt_txp_out({qsfp1_2_tx4_p, qsfp1_2_tx3_p, qsfp1_2_tx2_p, qsfp1_2_tx1_p}), // output
    .gt_txn_out({qsfp1_2_tx4_n, qsfp1_2_tx3_n, qsfp1_2_tx2_n, qsfp1_2_tx1_n}), // output
    .gt_txusrclk2(qsfp1_2_txuserclk2), // output
    .gt_loopback_in(12'd0), // input [11:0]
    .gt_rxrecclkout(), // output [3:0]
    .gt_powergoodout(), // output [3:0]
    .gt_ref_clk_out(), // output
    .gtwiz_reset_tx_datapath(1'b0), // input
    .gtwiz_reset_rx_datapath(1'b0), // input
    .sys_reset(rst_125mhz_int1), // input
    .gt_ref_clk_p(qsfp1_2_mgt_refclk_0_p), // input
    .gt_ref_clk_n(qsfp1_2_mgt_refclk_0_n), // input
    .init_clk(clk_125mhz_int1), // input

    .rx_axis_tvalid(qsfp1_2_rx_axis_tvalid_int), // output
    .rx_axis_tdata(qsfp1_2_rx_axis_tdata_int), // output [511:0]
    .rx_axis_tlast(qsfp1_2_rx_axis_tlast_int), // output
    .rx_axis_tkeep(qsfp1_2_rx_axis_tkeep_int), // output [63:0]
    .rx_axis_tuser(qsfp1_2_rx_axis_tuser_int[0]), // output

    .rx_otn_bip8_0(), // output [7:0]
    .rx_otn_bip8_1(), // output [7:0]
    .rx_otn_bip8_2(), // output [7:0]
    .rx_otn_bip8_3(), // output [7:0]
    .rx_otn_bip8_4(), // output [7:0]
    .rx_otn_data_0(), // output [65:0]
    .rx_otn_data_1(), // output [65:0]
    .rx_otn_data_2(), // output [65:0]
    .rx_otn_data_3(), // output [65:0]
    .rx_otn_data_4(), // output [65:0]
    .rx_otn_ena(), // output
    .rx_otn_lane0(), // output
    .rx_otn_vlmarker(), // output
    .rx_preambleout(), // output [55:0]
    .usr_rx_reset(qsfp1_2_rx_rst_int), // output
    .gt_rxusrclk2(), // output

    .rx_lane_aligner_fill_0(), // output [6:0]
    .rx_lane_aligner_fill_1(), // output [6:0]
    .rx_lane_aligner_fill_10(), // output [6:0]
    .rx_lane_aligner_fill_11(), // output [6:0]
    .rx_lane_aligner_fill_12(), // output [6:0]
    .rx_lane_aligner_fill_13(), // output [6:0]
    .rx_lane_aligner_fill_14(), // output [6:0]
    .rx_lane_aligner_fill_15(), // output [6:0]
    .rx_lane_aligner_fill_16(), // output [6:0]
    .rx_lane_aligner_fill_17(), // output [6:0]
    .rx_lane_aligner_fill_18(), // output [6:0]
    .rx_lane_aligner_fill_19(), // output [6:0]
    .rx_lane_aligner_fill_2(), // output [6:0]
    .rx_lane_aligner_fill_3(), // output [6:0]
    .rx_lane_aligner_fill_4(), // output [6:0]
    .rx_lane_aligner_fill_5(), // output [6:0]
    .rx_lane_aligner_fill_6(), // output [6:0]
    .rx_lane_aligner_fill_7(), // output [6:0]
    .rx_lane_aligner_fill_8(), // output [6:0]
    .rx_lane_aligner_fill_9(), // output [6:0]
    .rx_ptp_tstamp_out(qsfp1_2_rx_axis_tuser_int[80:1]), // output [79:0]
    .rx_ptp_pcslane_out(), // output [4:0]
    .ctl_rx_systemtimerin(qsfp1_2_rx_ptp_time_int), // input [79:0]

    .stat_rx_aligned(), // output
    .stat_rx_aligned_err(), // output
    .stat_rx_bad_code(), // output [2:0]
    .stat_rx_bad_fcs(), // output [2:0]
    .stat_rx_bad_preamble(), // output
    .stat_rx_bad_sfd(), // output
    .stat_rx_bip_err_0(), // output
    .stat_rx_bip_err_1(), // output
    .stat_rx_bip_err_10(), // output
    .stat_rx_bip_err_11(), // output
    .stat_rx_bip_err_12(), // output
    .stat_rx_bip_err_13(), // output
    .stat_rx_bip_err_14(), // output
    .stat_rx_bip_err_15(), // output
    .stat_rx_bip_err_16(), // output
    .stat_rx_bip_err_17(), // output
    .stat_rx_bip_err_18(), // output
    .stat_rx_bip_err_19(), // output
    .stat_rx_bip_err_2(), // output
    .stat_rx_bip_err_3(), // output
    .stat_rx_bip_err_4(), // output
    .stat_rx_bip_err_5(), // output
    .stat_rx_bip_err_6(), // output
    .stat_rx_bip_err_7(), // output
    .stat_rx_bip_err_8(), // output
    .stat_rx_bip_err_9(), // output
    .stat_rx_block_lock(), // output [19:0]
    .stat_rx_broadcast(), // output
    .stat_rx_fragment(), // output [2:0]
    .stat_rx_framing_err_0(), // output [1:0]
    .stat_rx_framing_err_1(), // output [1:0]
    .stat_rx_framing_err_10(), // output [1:0]
    .stat_rx_framing_err_11(), // output [1:0]
    .stat_rx_framing_err_12(), // output [1:0]
    .stat_rx_framing_err_13(), // output [1:0]
    .stat_rx_framing_err_14(), // output [1:0]
    .stat_rx_framing_err_15(), // output [1:0]
    .stat_rx_framing_err_16(), // output [1:0]
    .stat_rx_framing_err_17(), // output [1:0]
    .stat_rx_framing_err_18(), // output [1:0]
    .stat_rx_framing_err_19(), // output [1:0]
    .stat_rx_framing_err_2(), // output [1:0]
    .stat_rx_framing_err_3(), // output [1:0]
    .stat_rx_framing_err_4(), // output [1:0]
    .stat_rx_framing_err_5(), // output [1:0]
    .stat_rx_framing_err_6(), // output [1:0]
    .stat_rx_framing_err_7(), // output [1:0]
    .stat_rx_framing_err_8(), // output [1:0]
    .stat_rx_framing_err_9(), // output [1:0]
    .stat_rx_framing_err_valid_0(), // output
    .stat_rx_framing_err_valid_1(), // output
    .stat_rx_framing_err_valid_10(), // output
    .stat_rx_framing_err_valid_11(), // output
    .stat_rx_framing_err_valid_12(), // output
    .stat_rx_framing_err_valid_13(), // output
    .stat_rx_framing_err_valid_14(), // output
    .stat_rx_framing_err_valid_15(), // output
    .stat_rx_framing_err_valid_16(), // output
    .stat_rx_framing_err_valid_17(), // output
    .stat_rx_framing_err_valid_18(), // output
    .stat_rx_framing_err_valid_19(), // output
    .stat_rx_framing_err_valid_2(), // output
    .stat_rx_framing_err_valid_3(), // output
    .stat_rx_framing_err_valid_4(), // output
    .stat_rx_framing_err_valid_5(), // output
    .stat_rx_framing_err_valid_6(), // output
    .stat_rx_framing_err_valid_7(), // output
    .stat_rx_framing_err_valid_8(), // output
    .stat_rx_framing_err_valid_9(), // output
    .stat_rx_got_signal_os(), // output
    .stat_rx_hi_ber(), // output
    .stat_rx_inrangeerr(), // output
    .stat_rx_internal_local_fault(), // output
    .stat_rx_jabber(), // output
    .stat_rx_local_fault(), // output
    .stat_rx_mf_err(), // output [19:0]
    .stat_rx_mf_len_err(), // output [19:0]
    .stat_rx_mf_repeat_err(), // output [19:0]
    .stat_rx_misaligned(), // output
    .stat_rx_multicast(), // output
    .stat_rx_oversize(), // output
    .stat_rx_packet_1024_1518_bytes(), // output
    .stat_rx_packet_128_255_bytes(), // output
    .stat_rx_packet_1519_1522_bytes(), // output
    .stat_rx_packet_1523_1548_bytes(), // output
    .stat_rx_packet_1549_2047_bytes(), // output
    .stat_rx_packet_2048_4095_bytes(), // output
    .stat_rx_packet_256_511_bytes(), // output
    .stat_rx_packet_4096_8191_bytes(), // output
    .stat_rx_packet_512_1023_bytes(), // output
    .stat_rx_packet_64_bytes(), // output
    .stat_rx_packet_65_127_bytes(), // output
    .stat_rx_packet_8192_9215_bytes(), // output
    .stat_rx_packet_bad_fcs(), // output
    .stat_rx_packet_large(), // output
    .stat_rx_packet_small(), // output [2:0]

    .ctl_rx_enable(1'b1), // input
    .ctl_rx_force_resync(1'b0), // input
    .ctl_rx_test_pattern(1'b0), // input
    .ctl_rsfec_ieee_error_indication_mode(1'b0), // input
    .ctl_rx_rsfec_enable(1'b1), // input
    .ctl_rx_rsfec_enable_correction(1'b1), // input
    .ctl_rx_rsfec_enable_indication(1'b1), // input
    .core_rx_reset(1'b0), // input
    .rx_clk(qsfp1_2_rx_clk_int), // input

    .stat_rx_received_local_fault(), // output
    .stat_rx_remote_fault(), // output
    .stat_rx_status(qsfp1_2_rx_status), // output
    .stat_rx_stomped_fcs(), // output [2:0]
    .stat_rx_synced(), // output [19:0]
    .stat_rx_synced_err(), // output [19:0]
    .stat_rx_test_pattern_mismatch(), // output [2:0]
    .stat_rx_toolong(), // output
    .stat_rx_total_bytes(), // output [6:0]
    .stat_rx_total_good_bytes(), // output [13:0]
    .stat_rx_total_good_packets(), // output
    .stat_rx_total_packets(), // output [2:0]
    .stat_rx_truncated(), // output
    .stat_rx_undersize(), // output [2:0]
    .stat_rx_unicast(), // output
    .stat_rx_vlan(), // output
    .stat_rx_pcsl_demuxed(), // output [19:0]
    .stat_rx_pcsl_number_0(), // output [4:0]
    .stat_rx_pcsl_number_1(), // output [4:0]
    .stat_rx_pcsl_number_10(), // output [4:0]
    .stat_rx_pcsl_number_11(), // output [4:0]
    .stat_rx_pcsl_number_12(), // output [4:0]
    .stat_rx_pcsl_number_13(), // output [4:0]
    .stat_rx_pcsl_number_14(), // output [4:0]
    .stat_rx_pcsl_number_15(), // output [4:0]
    .stat_rx_pcsl_number_16(), // output [4:0]
    .stat_rx_pcsl_number_17(), // output [4:0]
    .stat_rx_pcsl_number_18(), // output [4:0]
    .stat_rx_pcsl_number_19(), // output [4:0]
    .stat_rx_pcsl_number_2(), // output [4:0]
    .stat_rx_pcsl_number_3(), // output [4:0]
    .stat_rx_pcsl_number_4(), // output [4:0]
    .stat_rx_pcsl_number_5(), // output [4:0]
    .stat_rx_pcsl_number_6(), // output [4:0]
    .stat_rx_pcsl_number_7(), // output [4:0]
    .stat_rx_pcsl_number_8(), // output [4:0]
    .stat_rx_pcsl_number_9(), // output [4:0]
    .stat_rx_rsfec_am_lock0(), // output
    .stat_rx_rsfec_am_lock1(), // output
    .stat_rx_rsfec_am_lock2(), // output
    .stat_rx_rsfec_am_lock3(), // output
    .stat_rx_rsfec_corrected_cw_inc(), // output
    .stat_rx_rsfec_cw_inc(), // output
    .stat_rx_rsfec_err_count0_inc(), // output [2:0]
    .stat_rx_rsfec_err_count1_inc(), // output [2:0]
    .stat_rx_rsfec_err_count2_inc(), // output [2:0]
    .stat_rx_rsfec_err_count3_inc(), // output [2:0]
    .stat_rx_rsfec_hi_ser(), // output
    .stat_rx_rsfec_lane_alignment_status(), // output
    .stat_rx_rsfec_lane_fill_0(), // output [13:0]
    .stat_rx_rsfec_lane_fill_1(), // output [13:0]
    .stat_rx_rsfec_lane_fill_2(), // output [13:0]
    .stat_rx_rsfec_lane_fill_3(), // output [13:0]
    .stat_rx_rsfec_lane_mapping(), // output [7:0]
    .stat_rx_rsfec_uncorrected_cw_inc(), // output

    .ctl_tx_systemtimerin(qsfp1_2_tx_ptp_time_int), // input [79:0]

    .stat_tx_ptp_fifo_read_error(), // output
    .stat_tx_ptp_fifo_write_error(), // output

    .tx_ptp_tstamp_valid_out(qsfp1_2_tx_ptp_ts_valid_int), // output
    .tx_ptp_pcslane_out(), // output [4:0]
    .tx_ptp_tstamp_tag_out(), // output [15:0]
    .tx_ptp_tstamp_out(qsfp1_2_tx_ptp_ts_int), // output [79:0]
    .tx_ptp_1588op_in(2'b10), // input [1:0]
    .tx_ptp_tag_field_in(16'd0), // input [15:0]
    .stat_tx_bad_fcs(), // output
    .stat_tx_broadcast(), // output
    .stat_tx_frame_error(), // output
    .stat_tx_local_fault(), // output
    .stat_tx_multicast(), // output
    .stat_tx_packet_1024_1518_bytes(), // output
    .stat_tx_packet_128_255_bytes(), // output
    .stat_tx_packet_1519_1522_bytes(), // output
    .stat_tx_packet_1523_1548_bytes(), // output
    .stat_tx_packet_1549_2047_bytes(), // output
    .stat_tx_packet_2048_4095_bytes(), // output
    .stat_tx_packet_256_511_bytes(), // output
    .stat_tx_packet_4096_8191_bytes(), // output
    .stat_tx_packet_512_1023_bytes(), // output
    .stat_tx_packet_64_bytes(), // output
    .stat_tx_packet_65_127_bytes(), // output
    .stat_tx_packet_8192_9215_bytes(), // output
    .stat_tx_packet_large(), // output
    .stat_tx_packet_small(), // output
    .stat_tx_total_bytes(), // output [5:0]
    .stat_tx_total_good_bytes(), // output [13:0]
    .stat_tx_total_good_packets(), // output
    .stat_tx_total_packets(), // output
    .stat_tx_unicast(), // output
    .stat_tx_vlan(), // output

    .ctl_tx_enable(1'b1), // input
    .ctl_tx_test_pattern(1'b0), // input
    .ctl_tx_rsfec_enable(1'b1), // input
    .ctl_tx_send_idle(1'b0), // input
    .ctl_tx_send_rfi(1'b0), // input
    .ctl_tx_send_lfi(1'b0), // input
    .core_tx_reset(1'b0), // input

    .tx_axis_tready(qsfp1_2_mac_tx_axis_tready), // output
    .tx_axis_tvalid(qsfp1_2_mac_tx_axis_tvalid), // input
    .tx_axis_tdata(qsfp1_2_mac_tx_axis_tdata), // input [511:0]
    .tx_axis_tlast(qsfp1_2_mac_tx_axis_tlast), // input
    .tx_axis_tkeep(qsfp1_2_mac_tx_axis_tkeep), // input [63:0]
    .tx_axis_tuser(qsfp1_2_mac_tx_axis_tuser), // input

    .tx_ovfout(), // output
    .tx_unfout(), // output
    .tx_preamblein(56'd0), // input [55:0]
    .usr_tx_reset(qsfp1_2_tx_rst_int), // output

    .core_drp_reset(1'b0), // input
    .drp_clk(1'b0), // input
    .drp_addr(10'd0), // input [9:0]
    .drp_di(16'd0), // input [15:0]
    .drp_en(1'b0), // input
    .drp_do(), // output [15:0]
    .drp_rdy(), // output
    .drp_we(1'b0) // input
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
    .qsfp1_tx_clk(qsfp1_1_tx_clk_int),
    .qsfp1_tx_rst(qsfp1_1_tx_rst_int),
    .qsfp1_tx_axis_tdata(qsfp1_1_tx_axis_tdata_int),
    .qsfp1_tx_axis_tkeep(qsfp1_1_tx_axis_tkeep_int),
    .qsfp1_tx_axis_tvalid(qsfp1_1_tx_axis_tvalid_int),
    .qsfp1_tx_axis_tready(qsfp1_1_tx_axis_tready_int),
    .qsfp1_tx_axis_tlast(qsfp1_1_tx_axis_tlast_int),
    .qsfp1_tx_axis_tuser(qsfp1_1_tx_axis_tuser_int),
    .qsfp1_tx_ptp_time(qsfp1_1_tx_ptp_time_int),
    .qsfp1_tx_ptp_ts(qsfp1_1_tx_ptp_ts_int),
    .qsfp1_tx_ptp_ts_valid(qsfp1_1_tx_ptp_ts_valid_int),
    .qsfp1_rx_clk(qsfp1_1_rx_clk_int),
    .qsfp1_rx_rst(qsfp1_1_rx_rst_int),
    .qsfp1_rx_axis_tdata(qsfp1_1_rx_axis_tdata_int),
    .qsfp1_rx_axis_tkeep(qsfp1_1_rx_axis_tkeep_int),
    .qsfp1_rx_axis_tvalid(qsfp1_1_rx_axis_tvalid_int),
    .qsfp1_rx_axis_tlast(qsfp1_1_rx_axis_tlast_int),
    .qsfp1_rx_axis_tuser(qsfp1_1_rx_axis_tuser_int),
    .qsfp1_rx_ptp_time(qsfp1_1_rx_ptp_time_int),

    .qsfp2_tx_clk(qsfp1_2_tx_clk_int),
    .qsfp2_tx_rst(qsfp1_2_tx_rst_int),
    .qsfp2_tx_axis_tdata(qsfp1_2_tx_axis_tdata_int),
    .qsfp2_tx_axis_tkeep(qsfp1_2_tx_axis_tkeep_int),
    .qsfp2_tx_axis_tvalid(qsfp1_2_tx_axis_tvalid_int),
    .qsfp2_tx_axis_tready(qsfp1_2_tx_axis_tready_int),
    .qsfp2_tx_axis_tlast(qsfp1_2_tx_axis_tlast_int),
    .qsfp2_tx_axis_tuser(qsfp1_2_tx_axis_tuser_int),
    .qsfp2_tx_ptp_time(qsfp1_2_tx_ptp_time_int),
    .qsfp2_tx_ptp_ts(qsfp1_2_tx_ptp_ts_int),
    .qsfp2_tx_ptp_ts_valid(qsfp1_2_tx_ptp_ts_valid_int),
    .qsfp2_rx_clk(qsfp1_2_rx_clk_int),
    .qsfp2_rx_rst(qsfp1_2_rx_rst_int),
    .qsfp2_rx_axis_tdata(qsfp1_2_rx_axis_tdata_int),
    .qsfp2_rx_axis_tkeep(qsfp1_2_rx_axis_tkeep_int),
    .qsfp2_rx_axis_tvalid(qsfp1_2_rx_axis_tvalid_int),
    .qsfp2_rx_axis_tlast(qsfp1_2_rx_axis_tlast_int),
    .qsfp2_rx_axis_tuser(qsfp1_2_rx_axis_tuser_int),
    .qsfp2_rx_ptp_time(qsfp1_2_rx_ptp_time_int),

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

// CMAC
wire                           qsfp2_1_tx_clk_int;
wire                           qsfp2_1_tx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp2_1_tx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp2_1_tx_axis_tkeep_int;
wire                           qsfp2_1_tx_axis_tvalid_int;
wire                           qsfp2_1_tx_axis_tready_int;
wire                           qsfp2_1_tx_axis_tlast_int;
wire                           qsfp2_1_tx_axis_tuser_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp2_1_mac_tx_axis_tdata;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp2_1_mac_tx_axis_tkeep;
wire                           qsfp2_1_mac_tx_axis_tvalid;
wire                           qsfp2_1_mac_tx_axis_tready;
wire                           qsfp2_1_mac_tx_axis_tlast;
wire                           qsfp2_1_mac_tx_axis_tuser;

wire [79:0]                    qsfp2_1_tx_ptp_time_int;
wire [79:0]                    qsfp2_1_tx_ptp_ts_int;
wire                           qsfp2_1_tx_ptp_ts_valid_int;

wire                           qsfp2_1_rx_clk_int;
wire                           qsfp2_1_rx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp2_1_rx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp2_1_rx_axis_tkeep_int;
wire                           qsfp2_1_rx_axis_tvalid_int;
wire                           qsfp2_1_rx_axis_tlast_int;
wire [80+1-1:0]                qsfp2_1_rx_axis_tuser_int;

wire [79:0]                    qsfp2_1_rx_ptp_time_int;

wire                           qsfp2_2_tx_clk_int;
wire                           qsfp2_2_tx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp2_2_tx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp2_2_tx_axis_tkeep_int;
wire                           qsfp2_2_tx_axis_tvalid_int;
wire                           qsfp2_2_tx_axis_tready_int;
wire                           qsfp2_2_tx_axis_tlast_int;
wire                           qsfp2_2_tx_axis_tuser_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp2_2_mac_tx_axis_tdata;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp2_2_mac_tx_axis_tkeep;
wire                           qsfp2_2_mac_tx_axis_tvalid;
wire                           qsfp2_2_mac_tx_axis_tready;
wire                           qsfp2_2_mac_tx_axis_tlast;
wire                           qsfp2_2_mac_tx_axis_tuser;

wire [79:0]                    qsfp2_2_tx_ptp_time_int;
wire [79:0]                    qsfp2_2_tx_ptp_ts_int;
wire                           qsfp2_2_tx_ptp_ts_valid_int;

wire                           qsfp2_2_rx_clk_int;
wire                           qsfp2_2_rx_rst_int;

wire [AXIS_ETH_DATA_WIDTH-1:0] qsfp2_2_rx_axis_tdata_int;
wire [AXIS_ETH_KEEP_WIDTH-1:0] qsfp2_2_rx_axis_tkeep_int;
wire                           qsfp2_2_rx_axis_tvalid_int;
wire                           qsfp2_2_rx_axis_tlast_int;
wire [80+1-1:0]                qsfp2_2_rx_axis_tuser_int;

wire [79:0]                    qsfp2_2_rx_ptp_time_int;

wire qsfp2_1_rx_status;
wire qsfp2_2_rx_status;

wire qsfp2_1_txuserclk2;

assign qsfp2_1_tx_clk_int = qsfp2_1_txuserclk2;
assign qsfp2_1_rx_clk_int = qsfp2_1_txuserclk2;

wire qsfp2_2_txuserclk2;

assign qsfp2_2_tx_clk_int = qsfp2_2_txuserclk2;
assign qsfp2_2_rx_clk_int = qsfp2_2_txuserclk2;

cmac_pad #(
    .DATA_WIDTH(AXIS_ETH_DATA_WIDTH),
    .KEEP_WIDTH(AXIS_ETH_KEEP_WIDTH),
    .USER_WIDTH(1)
)
qsfp2_1_cmac_pad_inst (
    .clk(qsfp2_1_tx_clk_int),
    .rst(qsfp2_1_tx_rst_int),

    .s_axis_tdata(qsfp2_1_tx_axis_tdata_int),
    .s_axis_tkeep(qsfp2_1_tx_axis_tkeep_int),
    .s_axis_tvalid(qsfp2_1_tx_axis_tvalid_int),
    .s_axis_tready(qsfp2_1_tx_axis_tready_int),
    .s_axis_tlast(qsfp2_1_tx_axis_tlast_int),
    .s_axis_tuser(qsfp2_1_tx_axis_tuser_int),

    .m_axis_tdata(qsfp2_1_mac_tx_axis_tdata),
    .m_axis_tkeep(qsfp2_1_mac_tx_axis_tkeep),
    .m_axis_tvalid(qsfp2_1_mac_tx_axis_tvalid),
    .m_axis_tready(qsfp2_1_mac_tx_axis_tready),
    .m_axis_tlast(qsfp2_1_mac_tx_axis_tlast),
    .m_axis_tuser(qsfp2_1_mac_tx_axis_tuser)
);

cmac_usplus_2_0
qsfp2_1_cmac_inst (
    .gt_rxp_in({qsfp2_1_rx4_p, qsfp2_1_rx3_p, qsfp2_1_rx2_p, qsfp2_1_rx1_p}), // input
    .gt_rxn_in({qsfp2_1_rx4_n, qsfp2_1_rx3_n, qsfp2_1_rx2_n, qsfp2_1_rx1_n}), // input
    .gt_txp_out({qsfp2_1_tx4_p, qsfp2_1_tx3_p, qsfp2_1_tx2_p, qsfp2_1_tx1_p}), // output
    .gt_txn_out({qsfp2_1_tx4_n, qsfp2_1_tx3_n, qsfp2_1_tx2_n, qsfp2_1_tx1_n}), // output
    .gt_txusrclk2(qsfp2_1_txuserclk2), // output
    .gt_loopback_in(12'd0), // input [11:0]
    .gt_rxrecclkout(), // output [3:0]
    .gt_powergoodout(), // output [3:0]
    .gt_ref_clk_out(), // output
    .gtwiz_reset_tx_datapath(1'b0), // input
    .gtwiz_reset_rx_datapath(1'b0), // input
    .sys_reset(rst_125mhz_int2), // input
    .gt_ref_clk_p(qsfp2_1_mgt_refclk_0_p), // input
    .gt_ref_clk_n(qsfp2_1_mgt_refclk_0_n), // input
    .init_clk(clk_125mhz_int2), // input

    .rx_axis_tvalid(qsfp2_1_rx_axis_tvalid_int), // output
    .rx_axis_tdata(qsfp2_1_rx_axis_tdata_int), // output [511:0]
    .rx_axis_tlast(qsfp2_1_rx_axis_tlast_int), // output
    .rx_axis_tkeep(qsfp2_1_rx_axis_tkeep_int), // output [63:0]
    .rx_axis_tuser(qsfp2_1_rx_axis_tuser_int[0]), // output

    .rx_otn_bip8_0(), // output [7:0]
    .rx_otn_bip8_1(), // output [7:0]
    .rx_otn_bip8_2(), // output [7:0]
    .rx_otn_bip8_3(), // output [7:0]
    .rx_otn_bip8_4(), // output [7:0]
    .rx_otn_data_0(), // output [65:0]
    .rx_otn_data_1(), // output [65:0]
    .rx_otn_data_2(), // output [65:0]
    .rx_otn_data_3(), // output [65:0]
    .rx_otn_data_4(), // output [65:0]
    .rx_otn_ena(), // output
    .rx_otn_lane0(), // output
    .rx_otn_vlmarker(), // output
    .rx_preambleout(), // output [55:0]
    .usr_rx_reset(qsfp2_1_rx_rst_int), // output
    .gt_rxusrclk2(), // output

    .rx_lane_aligner_fill_0(), // output [6:0]
    .rx_lane_aligner_fill_1(), // output [6:0]
    .rx_lane_aligner_fill_10(), // output [6:0]
    .rx_lane_aligner_fill_11(), // output [6:0]
    .rx_lane_aligner_fill_12(), // output [6:0]
    .rx_lane_aligner_fill_13(), // output [6:0]
    .rx_lane_aligner_fill_14(), // output [6:0]
    .rx_lane_aligner_fill_15(), // output [6:0]
    .rx_lane_aligner_fill_16(), // output [6:0]
    .rx_lane_aligner_fill_17(), // output [6:0]
    .rx_lane_aligner_fill_18(), // output [6:0]
    .rx_lane_aligner_fill_19(), // output [6:0]
    .rx_lane_aligner_fill_2(), // output [6:0]
    .rx_lane_aligner_fill_3(), // output [6:0]
    .rx_lane_aligner_fill_4(), // output [6:0]
    .rx_lane_aligner_fill_5(), // output [6:0]
    .rx_lane_aligner_fill_6(), // output [6:0]
    .rx_lane_aligner_fill_7(), // output [6:0]
    .rx_lane_aligner_fill_8(), // output [6:0]
    .rx_lane_aligner_fill_9(), // output [6:0]
    .rx_ptp_tstamp_out(qsfp2_1_rx_axis_tuser_int[80:1]), // output [79:0]
    .rx_ptp_pcslane_out(), // output [4:0]
    .ctl_rx_systemtimerin(qsfp2_1_rx_ptp_time_int), // input [79:0]
    .stat_rx_aligned(), // output
    .stat_rx_aligned_err(), // output
    .stat_rx_bad_code(), // output [2:0]
    .stat_rx_bad_fcs(), // output [2:0]
    .stat_rx_bad_preamble(), // output
    .stat_rx_bad_sfd(), // output
    .stat_rx_bip_err_0(), // output
    .stat_rx_bip_err_1(), // output
    .stat_rx_bip_err_10(), // output
    .stat_rx_bip_err_11(), // output
    .stat_rx_bip_err_12(), // output
    .stat_rx_bip_err_13(), // output
    .stat_rx_bip_err_14(), // output
    .stat_rx_bip_err_15(), // output
    .stat_rx_bip_err_16(), // output
    .stat_rx_bip_err_17(), // output
    .stat_rx_bip_err_18(), // output
    .stat_rx_bip_err_19(), // output
    .stat_rx_bip_err_2(), // output
    .stat_rx_bip_err_3(), // output
    .stat_rx_bip_err_4(), // output
    .stat_rx_bip_err_5(), // output
    .stat_rx_bip_err_6(), // output
    .stat_rx_bip_err_7(), // output
    .stat_rx_bip_err_8(), // output
    .stat_rx_bip_err_9(), // output
    .stat_rx_block_lock(), // output [19:0]
    .stat_rx_broadcast(), // output
    .stat_rx_fragment(), // output [2:0]
    .stat_rx_framing_err_0(), // output [1:0]
    .stat_rx_framing_err_1(), // output [1:0]
    .stat_rx_framing_err_10(), // output [1:0]
    .stat_rx_framing_err_11(), // output [1:0]
    .stat_rx_framing_err_12(), // output [1:0]
    .stat_rx_framing_err_13(), // output [1:0]
    .stat_rx_framing_err_14(), // output [1:0]
    .stat_rx_framing_err_15(), // output [1:0]
    .stat_rx_framing_err_16(), // output [1:0]
    .stat_rx_framing_err_17(), // output [1:0]
    .stat_rx_framing_err_18(), // output [1:0]
    .stat_rx_framing_err_19(), // output [1:0]
    .stat_rx_framing_err_2(), // output [1:0]
    .stat_rx_framing_err_3(), // output [1:0]
    .stat_rx_framing_err_4(), // output [1:0]
    .stat_rx_framing_err_5(), // output [1:0]
    .stat_rx_framing_err_6(), // output [1:0]
    .stat_rx_framing_err_7(), // output [1:0]
    .stat_rx_framing_err_8(), // output [1:0]
    .stat_rx_framing_err_9(), // output [1:0]
    .stat_rx_framing_err_valid_0(), // output
    .stat_rx_framing_err_valid_1(), // output
    .stat_rx_framing_err_valid_10(), // output
    .stat_rx_framing_err_valid_11(), // output
    .stat_rx_framing_err_valid_12(), // output
    .stat_rx_framing_err_valid_13(), // output
    .stat_rx_framing_err_valid_14(), // output
    .stat_rx_framing_err_valid_15(), // output
    .stat_rx_framing_err_valid_16(), // output
    .stat_rx_framing_err_valid_17(), // output
    .stat_rx_framing_err_valid_18(), // output
    .stat_rx_framing_err_valid_19(), // output
    .stat_rx_framing_err_valid_2(), // output
    .stat_rx_framing_err_valid_3(), // output
    .stat_rx_framing_err_valid_4(), // output
    .stat_rx_framing_err_valid_5(), // output
    .stat_rx_framing_err_valid_6(), // output
    .stat_rx_framing_err_valid_7(), // output
    .stat_rx_framing_err_valid_8(), // output
    .stat_rx_framing_err_valid_9(), // output
    .stat_rx_got_signal_os(), // output
    .stat_rx_hi_ber(), // output
    .stat_rx_inrangeerr(), // output
    .stat_rx_internal_local_fault(), // output
    .stat_rx_jabber(), // output
    .stat_rx_local_fault(), // output
    .stat_rx_mf_err(), // output [19:0]
    .stat_rx_mf_len_err(), // output [19:0]
    .stat_rx_mf_repeat_err(), // output [19:0]
    .stat_rx_misaligned(), // output
    .stat_rx_multicast(), // output
    .stat_rx_oversize(), // output
    .stat_rx_packet_1024_1518_bytes(), // output
    .stat_rx_packet_128_255_bytes(), // output
    .stat_rx_packet_1519_1522_bytes(), // output
    .stat_rx_packet_1523_1548_bytes(), // output
    .stat_rx_packet_1549_2047_bytes(), // output
    .stat_rx_packet_2048_4095_bytes(), // output
    .stat_rx_packet_256_511_bytes(), // output
    .stat_rx_packet_4096_8191_bytes(), // output
    .stat_rx_packet_512_1023_bytes(), // output
    .stat_rx_packet_64_bytes(), // output
    .stat_rx_packet_65_127_bytes(), // output
    .stat_rx_packet_8192_9215_bytes(), // output
    .stat_rx_packet_bad_fcs(), // output
    .stat_rx_packet_large(), // output
    .stat_rx_packet_small(), // output [2:0]

    .ctl_rx_enable(1'b1), // input
    .ctl_rx_force_resync(1'b0), // input
    .ctl_rx_test_pattern(1'b0), // input
    .ctl_rsfec_ieee_error_indication_mode(1'b0), // input
    .ctl_rx_rsfec_enable(1'b1), // input
    .ctl_rx_rsfec_enable_correction(1'b1), // input
    .ctl_rx_rsfec_enable_indication(1'b1), // input
    .core_rx_reset(1'b0), // input
    .rx_clk(qsfp2_1_rx_clk_int), // input

    .stat_rx_received_local_fault(), // output
    .stat_rx_remote_fault(), // output
    .stat_rx_status(qsfp2_1_rx_status), // output
    .stat_rx_stomped_fcs(), // output [2:0]
    .stat_rx_synced(), // output [19:0]
    .stat_rx_synced_err(), // output [19:0]
    .stat_rx_test_pattern_mismatch(), // output [2:0]
    .stat_rx_toolong(), // output
    .stat_rx_total_bytes(), // output [6:0]
    .stat_rx_total_good_bytes(), // output [13:0]
    .stat_rx_total_good_packets(), // output
    .stat_rx_total_packets(), // output [2:0]
    .stat_rx_truncated(), // output
    .stat_rx_undersize(), // output [2:0]
    .stat_rx_unicast(), // output
    .stat_rx_vlan(), // output
    .stat_rx_pcsl_demuxed(), // output [19:0]
    .stat_rx_pcsl_number_0(), // output [4:0]
    .stat_rx_pcsl_number_1(), // output [4:0]
    .stat_rx_pcsl_number_10(), // output [4:0]
    .stat_rx_pcsl_number_11(), // output [4:0]
    .stat_rx_pcsl_number_12(), // output [4:0]
    .stat_rx_pcsl_number_13(), // output [4:0]
    .stat_rx_pcsl_number_14(), // output [4:0]
    .stat_rx_pcsl_number_15(), // output [4:0]
    .stat_rx_pcsl_number_16(), // output [4:0]
    .stat_rx_pcsl_number_17(), // output [4:0]
    .stat_rx_pcsl_number_18(), // output [4:0]
    .stat_rx_pcsl_number_19(), // output [4:0]
    .stat_rx_pcsl_number_2(), // output [4:0]
    .stat_rx_pcsl_number_3(), // output [4:0]
    .stat_rx_pcsl_number_4(), // output [4:0]
    .stat_rx_pcsl_number_5(), // output [4:0]
    .stat_rx_pcsl_number_6(), // output [4:0]
    .stat_rx_pcsl_number_7(), // output [4:0]
    .stat_rx_pcsl_number_8(), // output [4:0]
    .stat_rx_pcsl_number_9(), // output [4:0]
    .stat_rx_rsfec_am_lock0(), // output
    .stat_rx_rsfec_am_lock1(), // output
    .stat_rx_rsfec_am_lock2(), // output
    .stat_rx_rsfec_am_lock3(), // output
    .stat_rx_rsfec_corrected_cw_inc(), // output
    .stat_rx_rsfec_cw_inc(), // output
    .stat_rx_rsfec_err_count0_inc(), // output [2:0]
    .stat_rx_rsfec_err_count1_inc(), // output [2:0]
    .stat_rx_rsfec_err_count2_inc(), // output [2:0]
    .stat_rx_rsfec_err_count3_inc(), // output [2:0]
    .stat_rx_rsfec_hi_ser(), // output
    .stat_rx_rsfec_lane_alignment_status(), // output
    .stat_rx_rsfec_lane_fill_0(), // output [13:0]
    .stat_rx_rsfec_lane_fill_1(), // output [13:0]
    .stat_rx_rsfec_lane_fill_2(), // output [13:0]
    .stat_rx_rsfec_lane_fill_3(), // output [13:0]
    .stat_rx_rsfec_lane_mapping(), // output [7:0]
    .stat_rx_rsfec_uncorrected_cw_inc(), // output

    .ctl_tx_systemtimerin(qsfp2_1_tx_ptp_time_int), // input [79:0]

    .stat_tx_ptp_fifo_read_error(), // output
    .stat_tx_ptp_fifo_write_error(), // output

    .tx_ptp_tstamp_valid_out(qsfp2_1_tx_ptp_ts_valid_int), // output
    .tx_ptp_pcslane_out(), // output [4:0]
    .tx_ptp_tstamp_tag_out(), // output [15:0]
    .tx_ptp_tstamp_out(qsfp2_1_tx_ptp_ts_int), // output [79:0]
    .tx_ptp_1588op_in(2'b10), // input [1:0]
    .tx_ptp_tag_field_in(16'd0), // input [15:0]

    .stat_tx_bad_fcs(), // output
    .stat_tx_broadcast(), // output
    .stat_tx_frame_error(), // output
    .stat_tx_local_fault(), // output
    .stat_tx_multicast(), // output
    .stat_tx_packet_1024_1518_bytes(), // output
    .stat_tx_packet_128_255_bytes(), // output
    .stat_tx_packet_1519_1522_bytes(), // output
    .stat_tx_packet_1523_1548_bytes(), // output
    .stat_tx_packet_1549_2047_bytes(), // output
    .stat_tx_packet_2048_4095_bytes(), // output
    .stat_tx_packet_256_511_bytes(), // output
    .stat_tx_packet_4096_8191_bytes(), // output
    .stat_tx_packet_512_1023_bytes(), // output
    .stat_tx_packet_64_bytes(), // output
    .stat_tx_packet_65_127_bytes(), // output
    .stat_tx_packet_8192_9215_bytes(), // output
    .stat_tx_packet_large(), // output
    .stat_tx_packet_small(), // output
    .stat_tx_total_bytes(), // output [5:0]
    .stat_tx_total_good_bytes(), // output [13:0]
    .stat_tx_total_good_packets(), // output
    .stat_tx_total_packets(), // output
    .stat_tx_unicast(), // output
    .stat_tx_vlan(), // output

    .ctl_tx_enable(1'b1), // input
    .ctl_tx_test_pattern(1'b0), // input
    .ctl_tx_rsfec_enable(1'b1), // input
    .ctl_tx_send_idle(1'b0), // input
    .ctl_tx_send_rfi(1'b0), // input
    .ctl_tx_send_lfi(1'b0), // input
    .core_tx_reset(1'b0), // input

    .tx_axis_tready(qsfp2_1_mac_tx_axis_tready), // output
    .tx_axis_tvalid(qsfp2_1_mac_tx_axis_tvalid), // input
    .tx_axis_tdata(qsfp2_1_mac_tx_axis_tdata), // input [511:0]
    .tx_axis_tlast(qsfp2_1_mac_tx_axis_tlast), // input
    .tx_axis_tkeep(qsfp2_1_mac_tx_axis_tkeep), // input [63:0]
    .tx_axis_tuser(qsfp2_1_mac_tx_axis_tuser), // input

    .tx_ovfout(), // output
    .tx_unfout(), // output
    .tx_preamblein(56'd0), // input [55:0]
    .usr_tx_reset(qsfp2_1_tx_rst_int), // output

    .core_drp_reset(1'b0), // input
    .drp_clk(1'b0), // input
    .drp_addr(10'd0), // input [9:0]
    .drp_di(16'd0), // input [15:0]
    .drp_en(1'b0), // input
    .drp_do(), // output [15:0]
    .drp_rdy(), // output
    .drp_we(1'b0) // input
);

cmac_pad #(
    .DATA_WIDTH(AXIS_ETH_DATA_WIDTH),
    .KEEP_WIDTH(AXIS_ETH_KEEP_WIDTH),
    .USER_WIDTH(1)
)
qsfp2_2_cmac_pad_inst (
    .clk(qsfp2_2_tx_clk_int),
    .rst(qsfp2_2_tx_rst_int),

    .s_axis_tdata(qsfp2_2_tx_axis_tdata_int),
    .s_axis_tkeep(qsfp2_2_tx_axis_tkeep_int),
    .s_axis_tvalid(qsfp2_2_tx_axis_tvalid_int),
    .s_axis_tready(qsfp2_2_tx_axis_tready_int),
    .s_axis_tlast(qsfp2_2_tx_axis_tlast_int),
    .s_axis_tuser(qsfp2_2_tx_axis_tuser_int),

    .m_axis_tdata(qsfp2_2_mac_tx_axis_tdata),
    .m_axis_tkeep(qsfp2_2_mac_tx_axis_tkeep),
    .m_axis_tvalid(qsfp2_2_mac_tx_axis_tvalid),
    .m_axis_tready(qsfp2_2_mac_tx_axis_tready),
    .m_axis_tlast(qsfp2_2_mac_tx_axis_tlast),
    .m_axis_tuser(qsfp2_2_mac_tx_axis_tuser)
);

cmac_usplus_2_1
qsfp2_2_cmac_inst (
    .gt_rxp_in({qsfp2_2_rx4_p, qsfp2_2_rx3_p, qsfp2_2_rx2_p, qsfp2_2_rx1_p}), // input
    .gt_rxn_in({qsfp2_2_rx4_n, qsfp2_2_rx3_n, qsfp2_2_rx2_n, qsfp2_2_rx1_n}), // input
    .gt_txp_out({qsfp2_2_tx4_p, qsfp2_2_tx3_p, qsfp2_2_tx2_p, qsfp2_2_tx1_p}), // output
    .gt_txn_out({qsfp2_2_tx4_n, qsfp2_2_tx3_n, qsfp2_2_tx2_n, qsfp2_2_tx1_n}), // output
    .gt_txusrclk2(qsfp2_2_txuserclk2), // output
    .gt_loopback_in(12'd0), // input [11:0]
    .gt_rxrecclkout(), // output [3:0]
    .gt_powergoodout(), // output [3:0]
    .gt_ref_clk_out(), // output
    .gtwiz_reset_tx_datapath(1'b0), // input
    .gtwiz_reset_rx_datapath(1'b0), // input
    .sys_reset(rst_125mhz_int2), // input
    .gt_ref_clk_p(qsfp2_2_mgt_refclk_0_p), // input
    .gt_ref_clk_n(qsfp2_2_mgt_refclk_0_n), // input
    .init_clk(clk_125mhz_int2), // input

    .rx_axis_tvalid(qsfp2_2_rx_axis_tvalid_int), // output
    .rx_axis_tdata(qsfp2_2_rx_axis_tdata_int), // output [511:0]
    .rx_axis_tlast(qsfp2_2_rx_axis_tlast_int), // output
    .rx_axis_tkeep(qsfp2_2_rx_axis_tkeep_int), // output [63:0]
    .rx_axis_tuser(qsfp2_2_rx_axis_tuser_int[0]), // output

    .rx_otn_bip8_0(), // output [7:0]
    .rx_otn_bip8_1(), // output [7:0]
    .rx_otn_bip8_2(), // output [7:0]
    .rx_otn_bip8_3(), // output [7:0]
    .rx_otn_bip8_4(), // output [7:0]
    .rx_otn_data_0(), // output [65:0]
    .rx_otn_data_1(), // output [65:0]
    .rx_otn_data_2(), // output [65:0]
    .rx_otn_data_3(), // output [65:0]
    .rx_otn_data_4(), // output [65:0]
    .rx_otn_ena(), // output
    .rx_otn_lane0(), // output
    .rx_otn_vlmarker(), // output
    .rx_preambleout(), // output [55:0]
    .usr_rx_reset(qsfp2_2_rx_rst_int), // output
    .gt_rxusrclk2(), // output

    .rx_lane_aligner_fill_0(), // output [6:0]
    .rx_lane_aligner_fill_1(), // output [6:0]
    .rx_lane_aligner_fill_10(), // output [6:0]
    .rx_lane_aligner_fill_11(), // output [6:0]
    .rx_lane_aligner_fill_12(), // output [6:0]
    .rx_lane_aligner_fill_13(), // output [6:0]
    .rx_lane_aligner_fill_14(), // output [6:0]
    .rx_lane_aligner_fill_15(), // output [6:0]
    .rx_lane_aligner_fill_16(), // output [6:0]
    .rx_lane_aligner_fill_17(), // output [6:0]
    .rx_lane_aligner_fill_18(), // output [6:0]
    .rx_lane_aligner_fill_19(), // output [6:0]
    .rx_lane_aligner_fill_2(), // output [6:0]
    .rx_lane_aligner_fill_3(), // output [6:0]
    .rx_lane_aligner_fill_4(), // output [6:0]
    .rx_lane_aligner_fill_5(), // output [6:0]
    .rx_lane_aligner_fill_6(), // output [6:0]
    .rx_lane_aligner_fill_7(), // output [6:0]
    .rx_lane_aligner_fill_8(), // output [6:0]
    .rx_lane_aligner_fill_9(), // output [6:0]
    .rx_ptp_tstamp_out(qsfp2_2_rx_axis_tuser_int[80:1]), // output [79:0]
    .rx_ptp_pcslane_out(), // output [4:0]
    .ctl_rx_systemtimerin(qsfp2_2_rx_ptp_time_int), // input [79:0]

    .stat_rx_aligned(), // output
    .stat_rx_aligned_err(), // output
    .stat_rx_bad_code(), // output [2:0]
    .stat_rx_bad_fcs(), // output [2:0]
    .stat_rx_bad_preamble(), // output
    .stat_rx_bad_sfd(), // output
    .stat_rx_bip_err_0(), // output
    .stat_rx_bip_err_1(), // output
    .stat_rx_bip_err_10(), // output
    .stat_rx_bip_err_11(), // output
    .stat_rx_bip_err_12(), // output
    .stat_rx_bip_err_13(), // output
    .stat_rx_bip_err_14(), // output
    .stat_rx_bip_err_15(), // output
    .stat_rx_bip_err_16(), // output
    .stat_rx_bip_err_17(), // output
    .stat_rx_bip_err_18(), // output
    .stat_rx_bip_err_19(), // output
    .stat_rx_bip_err_2(), // output
    .stat_rx_bip_err_3(), // output
    .stat_rx_bip_err_4(), // output
    .stat_rx_bip_err_5(), // output
    .stat_rx_bip_err_6(), // output
    .stat_rx_bip_err_7(), // output
    .stat_rx_bip_err_8(), // output
    .stat_rx_bip_err_9(), // output
    .stat_rx_block_lock(), // output [19:0]
    .stat_rx_broadcast(), // output
    .stat_rx_fragment(), // output [2:0]
    .stat_rx_framing_err_0(), // output [1:0]
    .stat_rx_framing_err_1(), // output [1:0]
    .stat_rx_framing_err_10(), // output [1:0]
    .stat_rx_framing_err_11(), // output [1:0]
    .stat_rx_framing_err_12(), // output [1:0]
    .stat_rx_framing_err_13(), // output [1:0]
    .stat_rx_framing_err_14(), // output [1:0]
    .stat_rx_framing_err_15(), // output [1:0]
    .stat_rx_framing_err_16(), // output [1:0]
    .stat_rx_framing_err_17(), // output [1:0]
    .stat_rx_framing_err_18(), // output [1:0]
    .stat_rx_framing_err_19(), // output [1:0]
    .stat_rx_framing_err_2(), // output [1:0]
    .stat_rx_framing_err_3(), // output [1:0]
    .stat_rx_framing_err_4(), // output [1:0]
    .stat_rx_framing_err_5(), // output [1:0]
    .stat_rx_framing_err_6(), // output [1:0]
    .stat_rx_framing_err_7(), // output [1:0]
    .stat_rx_framing_err_8(), // output [1:0]
    .stat_rx_framing_err_9(), // output [1:0]
    .stat_rx_framing_err_valid_0(), // output
    .stat_rx_framing_err_valid_1(), // output
    .stat_rx_framing_err_valid_10(), // output
    .stat_rx_framing_err_valid_11(), // output
    .stat_rx_framing_err_valid_12(), // output
    .stat_rx_framing_err_valid_13(), // output
    .stat_rx_framing_err_valid_14(), // output
    .stat_rx_framing_err_valid_15(), // output
    .stat_rx_framing_err_valid_16(), // output
    .stat_rx_framing_err_valid_17(), // output
    .stat_rx_framing_err_valid_18(), // output
    .stat_rx_framing_err_valid_19(), // output
    .stat_rx_framing_err_valid_2(), // output
    .stat_rx_framing_err_valid_3(), // output
    .stat_rx_framing_err_valid_4(), // output
    .stat_rx_framing_err_valid_5(), // output
    .stat_rx_framing_err_valid_6(), // output
    .stat_rx_framing_err_valid_7(), // output
    .stat_rx_framing_err_valid_8(), // output
    .stat_rx_framing_err_valid_9(), // output
    .stat_rx_got_signal_os(), // output
    .stat_rx_hi_ber(), // output
    .stat_rx_inrangeerr(), // output
    .stat_rx_internal_local_fault(), // output
    .stat_rx_jabber(), // output
    .stat_rx_local_fault(), // output
    .stat_rx_mf_err(), // output [19:0]
    .stat_rx_mf_len_err(), // output [19:0]
    .stat_rx_mf_repeat_err(), // output [19:0]
    .stat_rx_misaligned(), // output
    .stat_rx_multicast(), // output
    .stat_rx_oversize(), // output
    .stat_rx_packet_1024_1518_bytes(), // output
    .stat_rx_packet_128_255_bytes(), // output
    .stat_rx_packet_1519_1522_bytes(), // output
    .stat_rx_packet_1523_1548_bytes(), // output
    .stat_rx_packet_1549_2047_bytes(), // output
    .stat_rx_packet_2048_4095_bytes(), // output
    .stat_rx_packet_256_511_bytes(), // output
    .stat_rx_packet_4096_8191_bytes(), // output
    .stat_rx_packet_512_1023_bytes(), // output
    .stat_rx_packet_64_bytes(), // output
    .stat_rx_packet_65_127_bytes(), // output
    .stat_rx_packet_8192_9215_bytes(), // output
    .stat_rx_packet_bad_fcs(), // output
    .stat_rx_packet_large(), // output
    .stat_rx_packet_small(), // output [2:0]

    .ctl_rx_enable(1'b1), // input
    .ctl_rx_force_resync(1'b0), // input
    .ctl_rx_test_pattern(1'b0), // input
    .ctl_rsfec_ieee_error_indication_mode(1'b0), // input
    .ctl_rx_rsfec_enable(1'b1), // input
    .ctl_rx_rsfec_enable_correction(1'b1), // input
    .ctl_rx_rsfec_enable_indication(1'b1), // input
    .core_rx_reset(1'b0), // input
    .rx_clk(qsfp2_2_rx_clk_int), // input

    .stat_rx_received_local_fault(), // output
    .stat_rx_remote_fault(), // output
    .stat_rx_status(qsfp2_2_rx_status), // output
    .stat_rx_stomped_fcs(), // output [2:0]
    .stat_rx_synced(), // output [19:0]
    .stat_rx_synced_err(), // output [19:0]
    .stat_rx_test_pattern_mismatch(), // output [2:0]
    .stat_rx_toolong(), // output
    .stat_rx_total_bytes(), // output [6:0]
    .stat_rx_total_good_bytes(), // output [13:0]
    .stat_rx_total_good_packets(), // output
    .stat_rx_total_packets(), // output [2:0]
    .stat_rx_truncated(), // output
    .stat_rx_undersize(), // output [2:0]
    .stat_rx_unicast(), // output
    .stat_rx_vlan(), // output
    .stat_rx_pcsl_demuxed(), // output [19:0]
    .stat_rx_pcsl_number_0(), // output [4:0]
    .stat_rx_pcsl_number_1(), // output [4:0]
    .stat_rx_pcsl_number_10(), // output [4:0]
    .stat_rx_pcsl_number_11(), // output [4:0]
    .stat_rx_pcsl_number_12(), // output [4:0]
    .stat_rx_pcsl_number_13(), // output [4:0]
    .stat_rx_pcsl_number_14(), // output [4:0]
    .stat_rx_pcsl_number_15(), // output [4:0]
    .stat_rx_pcsl_number_16(), // output [4:0]
    .stat_rx_pcsl_number_17(), // output [4:0]
    .stat_rx_pcsl_number_18(), // output [4:0]
    .stat_rx_pcsl_number_19(), // output [4:0]
    .stat_rx_pcsl_number_2(), // output [4:0]
    .stat_rx_pcsl_number_3(), // output [4:0]
    .stat_rx_pcsl_number_4(), // output [4:0]
    .stat_rx_pcsl_number_5(), // output [4:0]
    .stat_rx_pcsl_number_6(), // output [4:0]
    .stat_rx_pcsl_number_7(), // output [4:0]
    .stat_rx_pcsl_number_8(), // output [4:0]
    .stat_rx_pcsl_number_9(), // output [4:0]
    .stat_rx_rsfec_am_lock0(), // output
    .stat_rx_rsfec_am_lock1(), // output
    .stat_rx_rsfec_am_lock2(), // output
    .stat_rx_rsfec_am_lock3(), // output
    .stat_rx_rsfec_corrected_cw_inc(), // output
    .stat_rx_rsfec_cw_inc(), // output
    .stat_rx_rsfec_err_count0_inc(), // output [2:0]
    .stat_rx_rsfec_err_count1_inc(), // output [2:0]
    .stat_rx_rsfec_err_count2_inc(), // output [2:0]
    .stat_rx_rsfec_err_count3_inc(), // output [2:0]
    .stat_rx_rsfec_hi_ser(), // output
    .stat_rx_rsfec_lane_alignment_status(), // output
    .stat_rx_rsfec_lane_fill_0(), // output [13:0]
    .stat_rx_rsfec_lane_fill_1(), // output [13:0]
    .stat_rx_rsfec_lane_fill_2(), // output [13:0]
    .stat_rx_rsfec_lane_fill_3(), // output [13:0]
    .stat_rx_rsfec_lane_mapping(), // output [7:0]
    .stat_rx_rsfec_uncorrected_cw_inc(), // output

    .ctl_tx_systemtimerin(qsfp2_2_tx_ptp_time_int), // input [79:0]

    .stat_tx_ptp_fifo_read_error(), // output
    .stat_tx_ptp_fifo_write_error(), // output

    .tx_ptp_tstamp_valid_out(qsfp2_2_tx_ptp_ts_valid_int), // output
    .tx_ptp_pcslane_out(), // output [4:0]
    .tx_ptp_tstamp_tag_out(), // output [15:0]
    .tx_ptp_tstamp_out(qsfp2_2_tx_ptp_ts_int), // output [79:0]
    .tx_ptp_1588op_in(2'b10), // input [1:0]
    .tx_ptp_tag_field_in(16'd0), // input [15:0]
    .stat_tx_bad_fcs(), // output
    .stat_tx_broadcast(), // output
    .stat_tx_frame_error(), // output
    .stat_tx_local_fault(), // output
    .stat_tx_multicast(), // output
    .stat_tx_packet_1024_1518_bytes(), // output
    .stat_tx_packet_128_255_bytes(), // output
    .stat_tx_packet_1519_1522_bytes(), // output
    .stat_tx_packet_1523_1548_bytes(), // output
    .stat_tx_packet_1549_2047_bytes(), // output
    .stat_tx_packet_2048_4095_bytes(), // output
    .stat_tx_packet_256_511_bytes(), // output
    .stat_tx_packet_4096_8191_bytes(), // output
    .stat_tx_packet_512_1023_bytes(), // output
    .stat_tx_packet_64_bytes(), // output
    .stat_tx_packet_65_127_bytes(), // output
    .stat_tx_packet_8192_9215_bytes(), // output
    .stat_tx_packet_large(), // output
    .stat_tx_packet_small(), // output
    .stat_tx_total_bytes(), // output [5:0]
    .stat_tx_total_good_bytes(), // output [13:0]
    .stat_tx_total_good_packets(), // output
    .stat_tx_total_packets(), // output
    .stat_tx_unicast(), // output
    .stat_tx_vlan(), // output

    .ctl_tx_enable(1'b1), // input
    .ctl_tx_test_pattern(1'b0), // input
    .ctl_tx_rsfec_enable(1'b1), // input
    .ctl_tx_send_idle(1'b0), // input
    .ctl_tx_send_rfi(1'b0), // input
    .ctl_tx_send_lfi(1'b0), // input
    .core_tx_reset(1'b0), // input

    .tx_axis_tready(qsfp2_2_mac_tx_axis_tready), // output
    .tx_axis_tvalid(qsfp2_2_mac_tx_axis_tvalid), // input
    .tx_axis_tdata(qsfp2_2_mac_tx_axis_tdata), // input [511:0]
    .tx_axis_tlast(qsfp2_2_mac_tx_axis_tlast), // input
    .tx_axis_tkeep(qsfp2_2_mac_tx_axis_tkeep), // input [63:0]
    .tx_axis_tuser(qsfp2_2_mac_tx_axis_tuser), // input

    .tx_ovfout(), // output
    .tx_unfout(), // output
    .tx_preamblein(56'd0), // input [55:0]
    .usr_tx_reset(qsfp2_2_tx_rst_int), // output

    .core_drp_reset(1'b0), // input
    .drp_clk(1'b0), // input
    .drp_addr(10'd0), // input [9:0]
    .drp_di(16'd0), // input [15:0]
    .drp_en(1'b0), // input
    .drp_do(), // output [15:0]
    .drp_rdy(), // output
    .drp_we(1'b0) // input
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
     * GPIO NC
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
    .qsfp1_tx_clk(qsfp2_1_tx_clk_int),
    .qsfp1_tx_rst(qsfp2_1_tx_rst_int),
    .qsfp1_tx_axis_tdata(qsfp2_1_tx_axis_tdata_int),
    .qsfp1_tx_axis_tkeep(qsfp2_1_tx_axis_tkeep_int),
    .qsfp1_tx_axis_tvalid(qsfp2_1_tx_axis_tvalid_int),
    .qsfp1_tx_axis_tready(qsfp2_1_tx_axis_tready_int),
    .qsfp1_tx_axis_tlast(qsfp2_1_tx_axis_tlast_int),
    .qsfp1_tx_axis_tuser(qsfp2_1_tx_axis_tuser_int),
    .qsfp1_tx_ptp_time(qsfp2_1_tx_ptp_time_int),
    .qsfp1_tx_ptp_ts(qsfp2_1_tx_ptp_ts_int),
    .qsfp1_tx_ptp_ts_valid(qsfp2_1_tx_ptp_ts_valid_int),
    .qsfp1_rx_clk(qsfp2_1_rx_clk_int),
    .qsfp1_rx_rst(qsfp2_1_rx_rst_int),
    .qsfp1_rx_axis_tdata(qsfp2_1_rx_axis_tdata_int),
    .qsfp1_rx_axis_tkeep(qsfp2_1_rx_axis_tkeep_int),
    .qsfp1_rx_axis_tvalid(qsfp2_1_rx_axis_tvalid_int),
    .qsfp1_rx_axis_tlast(qsfp2_1_rx_axis_tlast_int),
    .qsfp1_rx_axis_tuser(qsfp2_1_rx_axis_tuser_int),
    .qsfp1_rx_ptp_time(qsfp2_1_rx_ptp_time_int),

    .qsfp2_tx_clk(qsfp2_2_tx_clk_int),
    .qsfp2_tx_rst(qsfp2_2_tx_rst_int),
    .qsfp2_tx_axis_tdata(qsfp2_2_tx_axis_tdata_int),
    .qsfp2_tx_axis_tkeep(qsfp2_2_tx_axis_tkeep_int),
    .qsfp2_tx_axis_tvalid(qsfp2_2_tx_axis_tvalid_int),
    .qsfp2_tx_axis_tready(qsfp2_2_tx_axis_tready_int),
    .qsfp2_tx_axis_tlast(qsfp2_2_tx_axis_tlast_int),
    .qsfp2_tx_axis_tuser(qsfp2_2_tx_axis_tuser_int),
    .qsfp2_tx_ptp_time(qsfp2_2_tx_ptp_time_int),
    .qsfp2_tx_ptp_ts(qsfp2_2_tx_ptp_ts_int),
    .qsfp2_tx_ptp_ts_valid(qsfp2_2_tx_ptp_ts_valid_int),
    .qsfp2_rx_clk(qsfp2_2_rx_clk_int),
    .qsfp2_rx_rst(qsfp2_2_rx_rst_int),
    .qsfp2_rx_axis_tdata(qsfp2_2_rx_axis_tdata_int),
    .qsfp2_rx_axis_tkeep(qsfp2_2_rx_axis_tkeep_int),
    .qsfp2_rx_axis_tvalid(qsfp2_2_rx_axis_tvalid_int),
    .qsfp2_rx_axis_tlast(qsfp2_2_rx_axis_tlast_int),
    .qsfp2_rx_axis_tuser(qsfp2_2_rx_axis_tuser_int),
    .qsfp2_rx_ptp_time(qsfp2_2_rx_ptp_time_int),


    /*
     * QSPI flash NC
     */
    .fpga_boot(),
    .qspi_clk(),
    .qspi_0_dq_i(4'h0),
    .qspi_0_dq_o(),
    .qspi_0_dq_oe(),
    .qspi_0_cs()
);

endmodule
