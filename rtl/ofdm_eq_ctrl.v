// ============================================================================
// ofdm_eq_ctrl : LTF channel estimation + operand feeder for the equalizer cmpy
//
//  Input  : FFT output (unscaled, 24-bit re/im, natural order), 128 bins/symbol,
//           NSYM+1 symbols per frame (symbol 0 = LTF).
//  LTF    : H[k] = Y[k] * X_ltf[k]  (X_ltf = +/-1, BPSK)  for the 64 active bins
//           common shift sh chosen so max|H| < 2^14 (16-bit operands)
//  Data   : for each active bin emit A = Y>>sh (16b) and B = conj(H>>sh) (16b)
//           to Complex Multiplier -> Z = Y*conj(H)  (phase-equalized, |H|^2 weighted)
//           A.tuser = {last_symbol_of_frame, i[6:0]}, i = active index 0..63
//             i = 0..31  -> f = +1..+32   (bin k = i+1)
//             i = 32..63 -> f = -32..-1   (bin k = i+64)
//           A.tlast = (i == 63)
// ============================================================================
`timescale 1ns/1ps
module ofdm_eq_ctrl #(
    parameter integer NSYM = 16,
    parameter [63:0]  LTF_BITS = 64'hE78B72904CE8FBC1   // bit j: f = -32+j (j<32), j-31 (j>=32); 1 -> -1
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis_a:m_axis_b, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,
    input  wire [47:0] s_axis_tdata,     // re [23:0], im [47:24]
    input  wire [7:0]  s_axis_tuser,     // xk_index
    input  wire        s_axis_tlast,
    input  wire        s_axis_tvalid,
    output reg         s_axis_tready,
    output reg  [31:0] m_axis_a_tdata,
    output reg  [7:0]  m_axis_a_tuser,
    output reg         m_axis_a_tlast,
    output reg         m_axis_a_tvalid,
    input  wire        m_axis_a_tready,
    output reg  [31:0] m_axis_b_tdata,
    output reg         m_axis_b_tvalid,
    input  wire        m_axis_b_tready,
    output reg  [4:0]  dbg_shift
);
    reg [47:0] H [0:63];
    reg [7:0]  sym;          // 0 = LTF
    reg [23:0] maxabs;
    reg [4:0]  sh;

    wire signed [23:0] yr = s_axis_tdata[23:0];
    wire signed [23:0] yi = s_axis_tdata[47:24];
    wire [6:0] k = s_axis_tuser[6:0];            // FFT xk_index (natural order)
    wire [23:0] mabs = (absv(yr) > absv(yi)) ? absv(yr) : absv(yi);
    wire act  = (k >= 7'd1 && k <= 7'd32) || (k >= 7'd96);
    wire [6:0] ai = (k <= 7'd32) ? (k - 7'd1) : (k - 7'd64);    // active index i
    wire [5:0] jj = (k <= 7'd32) ? (k + 7'd31) : (k - 7'd96);   // LTF index j
    wire neg  = LTF_BITS[jj];

    function [23:0] absv(input signed [23:0] v); absv = v[23] ? -v : v; endfunction
    function [15:0] sat16(input signed [23:0] v, input [4:0] s);
        reg signed [23:0] t;
        begin
            t = v >>> s;
            sat16 = (t > 24'sd32767) ? 16'h7FFF : (t < -24'sd32767) ? 16'h8001 : t[15:0];
        end
    endfunction
    function [4:0] msb(input [23:0] v);      // index of highest '1'
        integer b; begin msb = 0; for (b = 0; b < 24; b = b + 1) if (v[b]) msb = b; end
    endfunction

    localparam [1:0] S_IN = 0, S_OUT = 1;
    reg [1:0] st;
    reg signed [23:0] hr_r, hi_r;
    wire signed [23:0] hr = H[ai][23:0];
    wire signed [23:0] hi = H[ai][47:24];

    always @(posedge aclk) begin
        if (!aresetn) begin
            st <= S_IN; s_axis_tready <= 1'b1; sym <= 0; maxabs <= 0; sh <= 0;
            m_axis_a_tvalid <= 1'b0; m_axis_b_tvalid <= 1'b0; dbg_shift <= 0;
        end else case (st)
            S_IN: if (s_axis_tvalid) begin
                    if (sym == 0) begin
                        if (act) begin
                            H[ai] <= neg ? {-yi, -yr} : {yi, yr};
                            if (mabs > maxabs) maxabs <= mabs;
                        end
                    end else if (act) begin
                        m_axis_a_tdata  <= {sat16(yi, sh), sat16(yr, sh)};
                        m_axis_a_tuser  <= {(sym == NSYM), 1'b0, ai[5:0]};
                        m_axis_a_tlast  <= (ai == 7'd63);
                        m_axis_b_tdata  <= {sat16(-hi, sh), sat16(hr, sh)};   // conj(H)
                        m_axis_a_tvalid <= 1'b1;
                        m_axis_b_tvalid <= 1'b1;
                        s_axis_tready   <= 1'b0;
                        st <= S_OUT;
                    end
                    // bin / symbol bookkeeping
                    if (s_axis_tlast) begin
                        if (sym == 0) begin
                            // shift so that max < 2^14 : sh = max(0, msb-13)
                            sh <= (msb(maxabs) > 13) ? msb(maxabs) - 5'd13 : 5'd0;
                            dbg_shift <= (msb(maxabs) > 13) ? msb(maxabs) - 5'd13 : 5'd0;
                        end
                        if (sym == NSYM) begin sym <= 0; maxabs <= 0; end
                        else sym <= sym + 1;
                    end
                end
            S_OUT: begin
                    if (m_axis_a_tready) m_axis_a_tvalid <= 1'b0;
                    if (m_axis_b_tready) m_axis_b_tvalid <= 1'b0;
                    if ((!m_axis_a_tvalid || m_axis_a_tready) &&
                        (!m_axis_b_tvalid || m_axis_b_tready)) begin
                        s_axis_tready <= 1'b1; st <= S_IN;
                    end
                end
            default: st <= S_IN;
        endcase
    end
endmodule
