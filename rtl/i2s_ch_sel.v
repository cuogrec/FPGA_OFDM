// ============================================================================
// i2s_ch_sel : Xilinx I2S Receiver AXIS (AES-like 32-bit word, TID = channel)
//              -> one channel, 16-bit, real sample packed as complex {imag=0, re}
//              for the Complex Multiplier (mixer) A-port.
//  * I2S Receiver word: [27:4] = 24-bit audio sample (MSB at bit 27)
//  * gain_shift : digital gain 0..15 (left shift) before taking 16 MSBs, saturated
//  * Never back-pressures the I2S Receiver (tready = 1); counts dropped samples.
// ============================================================================
`timescale 1ns/1ps
module i2s_ch_sel (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,
    // from I2S Receiver
    input  wire [31:0] s_axis_tdata,
    input  wire [2:0]  s_axis_tid,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    // to cmpy A port (re = [15:0], im = [31:16])
    output reg  [31:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    // control / status
    input  wire [3:0]  gain_shift,
    input  wire        ch_sel,          // 0 = left (TID 0), 1 = right (TID 1)
    output reg  [15:0] drop_cnt
);
    assign s_axis_tready = 1'b1;

    wire signed [23:0] s24 = s_axis_tdata[27:4];
    wire signed [39:0] sh  = $signed(s24) <<< gain_shift;      // up to 39 bits
    wire signed [31:0] s32 = sh[39:8];                           // keep 16 MSB + headroom
    wire        [15:0] sat = (s32 >  32'sd32767) ? 16'h7FFF :
                             (s32 < -32'sd32768) ? 16'h8000 : s32[15:0];
    wire take = s_axis_tvalid && (s_axis_tid[0] == ch_sel);

    always @(posedge aclk) begin
        if (!aresetn) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tdata  <= 32'd0;
            drop_cnt      <= 16'd0;
        end else begin
            if (m_axis_tvalid && m_axis_tready) m_axis_tvalid <= 1'b0;
            if (take) begin
                if (m_axis_tvalid && !m_axis_tready)
                    drop_cnt <= drop_cnt + 1'b1;               // previous not consumed
                else begin
                    m_axis_tdata  <= {16'h0000, sat};
                    m_axis_tvalid <= 1'b1;
                end
            end
        end
    end
endmodule
