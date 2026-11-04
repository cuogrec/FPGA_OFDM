// ============================================================================
// axis_tap_pkt : non-blocking debug tap. Always accepts input (never stalls the
//                DSP chain), forwards samples in packets of PKT_LEN with TLAST so
//                AXI DMA S2MM can capture raw baseband to DDR. Packets start only
//                when 'enable' = 1. Samples that cannot be forwarded are dropped
//                (counted) -- arm the DMA before setting enable.
// ============================================================================
`timescale 1ns/1ps
module axis_tap_pkt #(
    parameter integer PKT_LEN = 4096
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    output reg  [31:0] m_axis_tdata,
    output reg         m_axis_tlast,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    input  wire        enable,
    output reg  [15:0] drop_cnt
);
    assign s_axis_tready = 1'b1;
    reg        inpkt;
    reg [15:0] cnt;
    always @(posedge aclk) begin
        if (!aresetn) begin
            inpkt <= 1'b0; cnt <= 0; m_axis_tvalid <= 1'b0; m_axis_tlast <= 1'b0; drop_cnt <= 0;
        end else begin
            if (m_axis_tvalid && m_axis_tready) m_axis_tvalid <= 1'b0;
            if (s_axis_tvalid && (inpkt || enable)) begin
                if (m_axis_tvalid && !m_axis_tready) drop_cnt <= drop_cnt + 1'b1;
                else begin
                    m_axis_tdata  <= s_axis_tdata;
                    m_axis_tvalid <= 1'b1;
                    m_axis_tlast  <= (cnt == PKT_LEN-1);
                    if (cnt == PKT_LEN-1) begin cnt <= 0; inpkt <= 1'b0; end
                    else begin cnt <= cnt + 1; inpkt <= 1'b1; end
                end
            end
        end
    end
endmodule
