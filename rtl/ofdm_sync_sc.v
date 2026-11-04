// ============================================================================
// ofdm_sync_sc : Schmidl & Cox frame sync + CFO estimation/correction control
//
//  Baseband: 8 kHz complex, N = 128, CP = 32, L = N/2 = 64
//  Frame   : [CP|SC preamble][CP|LTF][CP|DATA] x NSYM
//
//  * Metric  P(d) = sum_{m<L} conj(r[d+m]) r[d+m+L],  R(d) = sum_{m<2L} |r[d+m]|^2
//    updated recursively (exact integers). Detect 2|P| * 256 >= thr * R, R >= rmin.
//  * Timing : midpoint of the above-threshold plateau (~ middle of SC CP)
//             FFT window of SC = mid + CP/4 + toffs   (default 8 samples into CP)
//  * CFO    : P at max |P| -> CORDIC (translate) -> angle phi = pi*eps
//             each output sample n is rotated by -phi*(n-n0)/L via CORDIC (rotate)
//  * Output : LTF + NSYM data symbols, CP removed, 128 samples each, TLAST per symbol,
//             to CORDIC rotate (cartesian = sample>>>2, phase channel), then FFT.
//
//  CORDIC format: Scaled radians, 16 bit, 2^13 == pi; cartesian 16 bit, 2^14 == 1.0
// ============================================================================
`timescale 1ns/1ps
module ofdm_sync_sc #(
    parameter integer NFFT   = 128,
    parameter integer NCP    = 32,
    parameter integer NSYM   = 16,       // data symbols per frame
    parameter integer AW     = 11        // sample buffer = 2^AW
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis_cfo:s_axis_phi:m_axis_cart:m_axis_phase, ASSOCIATED_RESET aresetn" *)
    input  wire        aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        aresetn,
    // baseband in (re [15:0], im [31:16]) -- never stalled
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    // control
    input  wire        enable,
    input  wire [7:0]  thr,              // metric threshold, 1/256 units (e.g. 150)
    input  wire [31:0] rmin,             // minimum energy R (gate on silence)
    input  wire [7:0]  toffs,            // signed timing offset (samples)
    input  wire [2:0]  bb_shift,         // extra gain (left shift) before CORDIC, saturated
    // CFO estimate -> CORDIC translate
    output reg  [31:0] m_axis_cfo_tdata,
    output reg         m_axis_cfo_tvalid,
    input  wire        m_axis_cfo_tready,
    input  wire [31:0] s_axis_phi_tdata, // [15:0] magnitude, [31:16] phase
    input  wire        s_axis_phi_tvalid,
    output reg         s_axis_phi_tready,
    // CORDIC rotate inputs
    output reg  [31:0] m_axis_cart_tdata,
    output reg         m_axis_cart_tlast,
    output reg         m_axis_cart_tvalid,
    input  wire        m_axis_cart_tready,
    output reg  [15:0] m_axis_phase_tdata,
    output reg         m_axis_phase_tvalid,
    input  wire        m_axis_phase_tready,
    // status
    output reg  [15:0] frame_cnt,
    output reg  [15:0] cfo_phase
);
    localparam integer L    = NFFT/2;
    localparam integer SYML = NFFT + NCP;
    localparam integer NOUT = NSYM + 1;          // LTF + data
    localparam integer HOLD = NCP/2;             // samples below thr to end plateau
    localparam integer MAXP = 4*NCP + 2*L;       // plateau timeout
    assign s_axis_tready = 1'b1;

    // ---------------- sample buffers (two copies: metric / output) ------------
    reg [31:0] buf_m [0:(1<<AW)-1];
    reg [31:0] buf_o [0:(1<<AW)-1];
    integer ii;
    initial for (ii = 0; ii < (1<<AW); ii = ii + 1) begin buf_m[ii] = 0; buf_o[ii] = 0; end

    reg  [31:0] ncnt;                // number of samples written (absolute index of next)
    reg  [AW-1:0] rd_m_addr;  reg [31:0] rd_m_q;
    reg  [AW-1:0] rd_o_addr;  reg [31:0] rd_o_q;
    reg         wr_en;   reg [31:0] wr_dat; reg [AW-1:0] wr_addr;

    always @(posedge aclk) begin
        if (wr_en) begin buf_m[wr_addr] <= wr_dat; buf_o[wr_addr] <= wr_dat; end
        rd_m_q <= buf_m[rd_m_addr];
        rd_o_q <= buf_o[rd_o_addr];
    end

    // ---------------- metric pipeline (one sample at a time) ------------------
    localparam [2:0] M_IDLE=0, M_RA=1, M_RC=2, M_WC=3, M_UPD=4, M_DEC=5;
    reg [2:0]  mst;
    reg [31:0] smp_b, smp_a, smp_c;
    reg signed [47:0] Pr, Pi, R;
    reg        met_valid;            // one-cycle strobe: metric for d = ncnt-2L
    reg [31:0] met_d;

    wire signed [15:0] ar = smp_a[15:0], ai = smp_a[31:16];
    wire signed [15:0] br = smp_b[15:0], bi = smp_b[31:16];
    wire signed [15:0] cr = smp_c[15:0], ci = smp_c[31:16];
    // conj(a)*b - conj(c)*a ,  |b|^2 - |a|^2
    wire signed [33:0] dPr = ($signed(ar)*br + $signed(ai)*bi) - ($signed(cr)*ar + $signed(ci)*ai);
    wire signed [33:0] dPi = ($signed(ar)*bi - $signed(ai)*br) - ($signed(cr)*ai - $signed(ci)*ar);
    wire signed [33:0] dR  = ($signed(br)*br + $signed(bi)*bi) - ($signed(cr)*cr + $signed(ci)*ci);

    always @(posedge aclk) begin
        met_valid <= 1'b0;
        wr_en     <= 1'b0;
        if (!aresetn) begin
            mst <= M_IDLE; ncnt <= 0; Pr <= 0; Pi <= 0; R <= 0; met_d <= 0;
        end else case (mst)
            M_IDLE: if (s_axis_tvalid) begin
                        smp_b     <= s_axis_tdata;
                        wr_en     <= 1'b1; wr_addr <= ncnt[AW-1:0]; wr_dat <= s_axis_tdata;
                        rd_m_addr <= ncnt[AW-1:0] - L;          // r[n-L]
                        mst <= M_RA;
                    end
            M_RA:   begin rd_m_addr <= ncnt[AW-1:0] - 2*L; mst <= M_RC; end   // r[n-2L]
            M_RC:   begin smp_a <= rd_m_q; mst <= M_WC; end
            M_WC:   begin smp_c <= rd_m_q; mst <= M_UPD; end
            M_UPD:  begin
                        Pr <= Pr + dPr; Pi <= Pi + dPi; R <= R + dR;
                        met_d <= ncnt - (2*L - 1);
                        ncnt  <= ncnt + 1;
                        mst   <= M_DEC;
                    end
            M_DEC:  begin met_valid <= 1'b1; mst <= M_IDLE; end
            default: mst <= M_IDLE;
        endcase
    end

    // R = sum of |r|^2 over the whole 2L window  ->  |P| <= R/2 always (no false
    // alarms at burst end).  |P| ~ max + min/2 ;  above = |P|*512 >= thr*R && R >= rmin
    wire [47:0] aPr  = Pr[47] ? -Pr : Pr;
    wire [47:0] aPi  = Pi[47] ? -Pi : Pi;
    wire [47:0] pmax = (aPr > aPi) ? aPr : aPi;
    wire [47:0] pmin = (aPr > aPi) ? aPi : aPr;
    wire [48:0] pabs = pmax + (pmin >> 1);
    wire [57:0] lhs  = {pabs, 9'd0};
    wire [57:0] rhs  = R[47:0] * thr;
    wire        above = (lhs >= rhs) && (R >= $signed({16'd0, rmin})) && (R > 0);

    // ---------------- detection FSM -------------------------------------------
    localparam [3:0] D_WAITLOW=0, D_ARM=1, D_PLAT=2, D_NORM=3, D_CFO=4, D_PHI=5,
                     D_OUT=6;
    reg [3:0]  dst;
    reg [31:0] d1, d2;
    reg [15:0] below_cnt;
    reg signed [47:0] Pbr, Pbi;
    reg [48:0] pbest;
    reg [31:0] n0;                    // SC FFT-window start (absolute)
    reg signed [21:0] inc;            // phase increment per sample, 2^21 == pi
    reg        out_go, out_done;

    always @(posedge aclk) begin
        out_go <= 1'b0;
        if (!aresetn) begin
            dst <= D_WAITLOW; below_cnt <= 0; frame_cnt <= 0; cfo_phase <= 0;
            m_axis_cfo_tvalid <= 1'b0; s_axis_phi_tready <= 1'b0; inc <= 0;
            d1 <= 0; d2 <= 0; n0 <= 0; Pbr <= 0; Pbi <= 0; pbest <= 0;
        end else case (dst)
            D_WAITLOW: if (met_valid) begin                 // re-arm after HOLD quiet samples
                    if (above || !enable) below_cnt <= 0;
                    else if (below_cnt == HOLD-1) begin below_cnt <= 0; dst <= D_ARM; end
                    else below_cnt <= below_cnt + 1;
                end
            D_ARM: if (met_valid && enable && above) begin
                    d1 <= met_d; d2 <= met_d; below_cnt <= 0;
                    Pbr <= Pr; Pbi <= Pi; pbest <= pabs;
                    dst <= D_PLAT;
                end
            D_PLAT: if (met_valid) begin
                    if (above) begin
                        d2 <= met_d; below_cnt <= 0;
                        if (pabs > pbest) begin Pbr <= Pr; Pbi <= Pi; pbest <= pabs; end
                    end else below_cnt <= below_cnt + 1;
                    if (met_d - d1 > MAXP) begin dst <= D_WAITLOW; below_cnt <= 0; end
                    else if (!above && below_cnt == HOLD-1) begin
                        n0  <= ((d1 + d2) >> 1) + NCP/4 + {{24{toffs[7]}}, toffs};
                        dst <= D_NORM;
                    end
                end
            D_NORM: begin                                     // scale P into |x| < 2^13
                    if ((Pbr > 48'sd8191) || (Pbr < -48'sd8191) ||
                        (Pbi > 48'sd8191) || (Pbi < -48'sd8191)) begin
                        Pbr <= Pbr >>> 1; Pbi <= Pbi >>> 1;
                    end else begin
                        m_axis_cfo_tdata  <= {Pbi[15:0], Pbr[15:0]};
                        m_axis_cfo_tvalid <= 1'b1;
                        dst <= D_CFO;
                    end
                end
            D_CFO: if (m_axis_cfo_tready) begin
                    m_axis_cfo_tvalid <= 1'b0; s_axis_phi_tready <= 1'b1; dst <= D_PHI;
                end
            D_PHI: if (s_axis_phi_tvalid) begin
                    s_axis_phi_tready <= 1'b0;
                    cfo_phase <= s_axis_phi_tdata[31:16];
                    // inc = -(phi * 2^8) / L  = -(phi << 2)  for L = 64
                    inc   <= -($signed({{6{s_axis_phi_tdata[31]}}, s_axis_phi_tdata[31:16]}) <<< 2);
                    out_go <= 1'b1;
                    dst <= D_OUT;
                end
            D_OUT: if (out_done) begin
                    frame_cnt <= frame_cnt + 1; below_cnt <= 0; dst <= D_WAITLOW;
                end
            default: dst <= D_WAITLOW;
        endcase
    end

    // ---------------- output reader -------------------------------------------
    localparam [2:0] O_IDLE=0, O_WAIT=1, O_RD=2, O_LAT=3, O_SEND=4;
    reg [2:0]  ost;
    reg [7:0]  osym;          // 0 = LTF, 1..NSYM data
    reg [7:0]  oidx;          // 0..NFFT-1
    reg [31:0] oaddr;         // absolute sample index
    reg [11:0] ooff;          // oaddr - n0
    // sample * 2^bb_shift / 4, saturated to |x| <= 8191 (CORDIC input |v| < 1.0 = 2^14)
    function [15:0] cart_sat(input signed [15:0] v);
        reg signed [23:0] t;
        begin
            t = ($signed({{8{v[15]}}, v}) <<< bb_shift) >>> 2;
            cart_sat = (t > 24'sd8191) ? 16'sd8191 : (t < -24'sd8191) ? -16'sd8191 : t[15:0];
        end
    endfunction
    wire signed [33:0] theta = $signed({1'b0, ooff}) * inc;

    always @(posedge aclk) begin
        out_done <= 1'b0;
        if (!aresetn) begin
            ost <= O_IDLE; m_axis_cart_tvalid <= 1'b0; m_axis_phase_tvalid <= 1'b0;
            m_axis_cart_tlast <= 1'b0; osym <= 0; oidx <= 0;
        end else case (ost)
            O_IDLE: if (out_go) begin
                    osym <= 0; oidx <= 0;
                    oaddr <= n0 + SYML;           // LTF window start
                    ooff  <= SYML;
                    ost   <= O_WAIT;
                end
            O_WAIT: if ($signed(oaddr - ncnt) < 0) begin   // sample already written
                    rd_o_addr <= oaddr[AW-1:0]; ost <= O_RD;
                end
            O_RD:   ost <= O_LAT;
            O_LAT:  begin
                    m_axis_cart_tdata   <= {cart_sat(rd_o_q[31:16]), cart_sat(rd_o_q[15:0])};
                    m_axis_cart_tlast   <= (oidx == NFFT-1);
                    m_axis_phase_tdata  <= {{2{theta[21]}}, theta[21:8]};
                    m_axis_cart_tvalid  <= 1'b1;
                    m_axis_phase_tvalid <= 1'b1;
                    ost <= O_SEND;
                end
            O_SEND: begin
                    if (m_axis_cart_tready)  m_axis_cart_tvalid  <= 1'b0;
                    if (m_axis_phase_tready) m_axis_phase_tvalid <= 1'b0;
                    if ((!m_axis_cart_tvalid  || m_axis_cart_tready) &&
                        (!m_axis_phase_tvalid || m_axis_phase_tready)) begin
                        if (oidx == NFFT-1) begin
                            oidx <= 0;
                            oaddr <= oaddr + NCP + 1; ooff <= ooff + NCP + 1;
                            if (osym == NOUT-1) begin ost <= O_IDLE; out_done <= 1'b1; end
                            else begin osym <= osym + 1; ost <= O_WAIT; end
                        end else begin
                            oidx <= oidx + 1; oaddr <= oaddr + 1; ooff <= ooff + 1;
                            ost <= O_WAIT;
                        end
                    end
                end
            default: ost <= O_IDLE;
        endcase
    end
endmodule
