// ============================================================================
// ofdm_qpsk_demap : pilot-aided phase tracking + QPSK hard decision
//                   + LSB-first bit packing into 32-bit words for AXI DMA
//
//  Input  : Z = Y*conj(H) from Complex Multiplier (24-bit re/im)
//           tuser = {last_symbol_of_frame, 0, i[5:0]}, tlast at i = 63
//           i = 0..31 -> f = +1..+32 ; i = 32..63 -> f = -32..-1
//  Pilots : f = -28,-20,-12,-4,4,12,20,28 (value +1), spacing 8
//  Phase  : per data subcarrier f, reference C(f) is the linear interpolation
//           of the two neighbouring pilots (nearest pilot at the band edges)
//              C = (8-a)*Z[fA] + a*Z[fB],  a = clamp(f - fA, 0, 8)
//           -> corrects common phase (CFO residual) AND the phase slope caused
//              by sampling-clock offset / timing drift.  Decision on Z*conj(C).
//  Order  : data subcarriers by increasing frequency f = -32 .. +32
//  QPSK   : b0 = (I < 0), b1 = (Q < 0)  (Gray, matches TX)
//  Output : 56 data x 2 bit = 112 bit/symbol, 16 symbols -> 56 words, TLAST on last
// ============================================================================
`timescale 1ns/1ps
module ofdm_qpsk_demap (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,
    input  wire [47:0] s_axis_tdata,
    input  wire [7:0]  s_axis_tuser,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tvalid,
    output reg         s_axis_tready,
    output reg  [31:0] m_axis_tdata,
    output reg         m_axis_tlast,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg  [15:0] frame_cnt
);
    reg [47:0] Z [0:63];
    reg        last_sym;

    // ---- helpers -----------------------------------------------------------
    function [5:0] f2i(input signed [6:0] f);           // frequency -> storage index
        f2i = (f > 0) ? (f - 1) : (f + 64);
    endfunction
    function is_pilot_f(input signed [6:0] f);
        is_pilot_f = (f == -28) || (f == -20) || (f == -12) || (f == -4) ||
                     (f ==   4) || (f ==  12) || (f ==  20) || (f ==  28);
    endfunction
    function signed [6:0] pil_a(input signed [6:0] f);  // lower pilot of the interpolation pair
        pil_a = (f < -20) ? -7'sd28 : (f < -12) ? -7'sd20 : (f < -4) ? -7'sd12 :
                (f <   4) ? -7'sd4  : (f <  12) ?  7'sd4  : (f < 20) ?  7'sd12 : 7'sd20;
    endfunction
    function [23:0] absv(input signed [23:0] v); absv = v[23] ? -v : v; endfunction
    function [4:0] msb(input [23:0] v);
        integer b; begin msb = 0; for (b = 0; b < 24; b = b + 1) if (v[b]) msb = b; end
    endfunction

    localparam [2:0] S_IN = 0, S_PMAX = 1, S_C = 2, S_D = 3, S_PUSH = 4, S_FLUSH = 5, S_NEXT = 6;
    reg [2:0]  st;
    reg signed [6:0] f;                 // current frequency -32..32
    reg [3:0]  pc;                      // pilot counter for max scan
    reg [23:0] pmax;
    reg [4:0]  psh;
    reg signed [15:0] Cr, Ci;
    reg [31:0] word;
    reg [5:0]  bcnt;

    // pilot scan
    wire signed [6:0] fp = (pc < 4) ? (-7'sd28 + $signed({1'b0, pc, 3'b000})) :
                                      ( 7'sd4  + $signed({1'b0, pc - 4'd4, 3'b000}));
    wire signed [23:0] pr_s = Z[f2i(fp)][23:0];
    wire signed [23:0] pi_s = Z[f2i(fp)][47:24];
    wire [23:0] pabs = (absv(pr_s) > absv(pi_s)) ? absv(pr_s) : absv(pi_s);

    // interpolation (stage 1)
    wire signed [6:0]  fa = pil_a(f);
    wire signed [6:0]  fb = fa + 7'sd8;
    wire signed [6:0]  wraw = f - fa;                 // -4 .. 12
    // band edges (|f| > 28): nearest pilot, no extrapolation (|Z_p| ~ |H|^2 differs
    // strongly between pilots -> extrapolated reference could flip sign)
    wire signed [6:0]  wb = (wraw < 0) ? 7'sd0 : (wraw > 8) ? 7'sd8 : wraw;
    wire signed [6:0]  wa = 7'sd8 - wb;
    wire signed [23:0] zar = $signed(Z[f2i(fa)][23:0])  >>> psh;
    wire signed [23:0] zai = $signed(Z[f2i(fa)][47:24]) >>> psh;
    wire signed [23:0] zbr = $signed(Z[f2i(fb)][23:0])  >>> psh;
    wire signed [23:0] zbi = $signed(Z[f2i(fb)][47:24]) >>> psh;
    wire signed [31:0] ccr = (zar * wa + zbr * wb) >>> 3;
    wire signed [31:0] cci = (zai * wa + zbi * wb) >>> 3;

    // decision (stage 2)
    wire signed [23:0] zr = Z[f2i(f)][23:0];
    wire signed [23:0] zi = Z[f2i(f)][47:24];
    wire signed [40:0] dre = zr * Cr + zi * Ci;       // Re(Z conj C)
    wire signed [40:0] dim = zi * Cr - zr * Ci;       // Im(Z conj C)
    wire last_data = (f == 7'sd32);

    always @(posedge aclk) begin
        if (!aresetn) begin
            st <= S_IN; s_axis_tready <= 1'b1; m_axis_tvalid <= 1'b0; m_axis_tlast <= 1'b0;
            word <= 0; bcnt <= 0; frame_cnt <= 0; last_sym <= 0;
        end else case (st)
            S_IN: if (s_axis_tvalid) begin
                    Z[s_axis_tuser[5:0]] <= s_axis_tdata;
                    if (s_axis_tlast) begin
                        last_sym <= s_axis_tuser[7];
                        s_axis_tready <= 1'b0;
                        pc <= 0; pmax <= 0;
                        st <= S_PMAX;
                    end
                end
            S_PMAX: begin                                     // common pilot scaling
                    if (pabs > pmax) pmax <= pabs;
                    if (pc == 4'd7) st <= S_NEXT; else pc <= pc + 1;
                    f <= -7'sd32;
                end
            S_NEXT: begin
                    psh <= (msb(pmax) > 12) ? msb(pmax) - 5'd12 : 5'd0;   // |pilot| < 2^13
                    st  <= S_C;
                end
            S_C: begin
                    if (f == 0 || is_pilot_f(f)) f <= f + 1;
                    else begin Cr <= ccr[15:0]; Ci <= cci[15:0]; st <= S_D; end
                    if (f == 7'sd33) begin                       // symbol done
                        if (last_sym && bcnt != 0) st <= S_FLUSH;
                        else begin
                            if (last_sym) frame_cnt <= frame_cnt + 1;
                            s_axis_tready <= 1'b1; st <= S_IN;
                        end
                    end
                end
            S_D: begin
                    word[bcnt]     <= dre[40];
                    word[bcnt + 1] <= dim[40];
                    f <= f + 1;
                    if (bcnt == 6'd30) begin
                        m_axis_tdata  <= {dim[40], dre[40], word[29:0]};
                        m_axis_tlast  <= last_sym && last_data;
                        m_axis_tvalid <= 1'b1;
                        bcnt <= 0;
                        st <= S_PUSH;
                    end else begin bcnt <= bcnt + 2; st <= S_C; end
                end
            S_PUSH: if (m_axis_tready) begin m_axis_tvalid <= 1'b0; st <= S_C; end
            S_FLUSH: begin                                    // pad partial word (unused for 16 sym)
                    if (!m_axis_tvalid) begin
                        m_axis_tdata <= word; m_axis_tlast <= 1'b1; m_axis_tvalid <= 1'b1;
                    end else if (m_axis_tready) begin
                        m_axis_tvalid <= 1'b0; bcnt <= 0; word <= 0;
                        frame_cnt <= frame_cnt + 1; s_axis_tready <= 1'b1; st <= S_IN;
                    end
                end
            default: st <= S_IN;
        endcase
    end
endmodule
