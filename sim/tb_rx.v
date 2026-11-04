// End-to-end testbench: baseband (8 kHz IQ) -> sync -> [CORDIC models] -> [FFT model]
// -> eq_ctrl -> [cmpy model] -> demap -> words.  Behavioral models stand in for the
// Vivado IP (CORDIC translate/rotate, xFFT unscaled natural order, cmpy 16x16->24).
`timescale 1ns/1ps
module tb_rx;
    reg clk = 0, rstn = 0;
    always #5 clk = ~clk;
    integer SPACING = 120;          // clocks per baseband sample (real HW: 12500)

    // ---------------- stimulus ----------------
    reg [31:0] mem [0:65535];
    integer nsamp, si, cyc;
    reg [31:0] bb_d; reg bb_v;
    initial begin
        for (si = 0; si < 65536; si = si + 1) mem[si] = 32'hxxxxxxxx;
        $readmemh("bb_in.hex", mem);
        nsamp = 0; while (mem[nsamp] !== 32'hxxxxxxxx) nsamp = nsamp + 1;
        $display("samples: %0d", nsamp);
    end
    initial begin
        bb_v = 0; bb_d = 0;
        repeat (20) @(posedge clk); rstn = 1;
        for (si = 0; si < nsamp; si = si + 1) begin
            @(posedge clk); bb_d <= mem[si]; bb_v <= 1;
            @(posedge clk); bb_v <= 0;
            repeat (SPACING - 2) @(posedge clk);
        end
        repeat (20000) @(posedge clk);
        $display("sync frames=%0d demap frames=%0d words=%0d cfo_phase=%0d shift=%0d",
                 u_sync.frame_cnt, u_dem.frame_cnt, nwords, $signed(u_sync.cfo_phase), u_eq.dbg_shift);
        $fclose(fo); $finish;
    end

    // ---------------- sync ----------------
    wire [31:0] cfo_d; wire cfo_v; reg cfo_r = 1;
    reg  [31:0] phi_d; reg phi_v = 0; wire phi_r;
    wire [31:0] cart_d; wire cart_l, cart_v; wire [15:0] ph_d; wire ph_v;
    reg  cart_r, ph_r;
    wire [15:0] fcnt, cfop;
    ofdm_sync_sc u_sync (.aclk(clk), .aresetn(rstn),
        .s_axis_tdata(bb_d), .s_axis_tvalid(bb_v), .s_axis_tready(),
        .enable(1'b1), .thr(8'd150), .rmin(32'd1000), .toffs(8'd0), .bb_shift(3'd2),
        .m_axis_cfo_tdata(cfo_d), .m_axis_cfo_tvalid(cfo_v), .m_axis_cfo_tready(cfo_r),
        .s_axis_phi_tdata(phi_d), .s_axis_phi_tvalid(phi_v), .s_axis_phi_tready(phi_r),
        .m_axis_cart_tdata(cart_d), .m_axis_cart_tlast(cart_l), .m_axis_cart_tvalid(cart_v),
        .m_axis_cart_tready(cart_r),
        .m_axis_phase_tdata(ph_d), .m_axis_phase_tvalid(ph_v), .m_axis_phase_tready(ph_r),
        .frame_cnt(fcnt), .cfo_phase(cfop));

    // CORDIC translate model (latency 20)
    real ang;
    always @(posedge clk) if (cfo_v && cfo_r) begin
        ang = $atan2($itor($signed(cfo_d[31:16])), $itor($signed(cfo_d[15:0])));
        repeat (20) @(posedge clk);
        phi_d <= {16'($rtoi(ang / 3.14159265358979 * 8192.0)), 16'd0};
        phi_v <= 1;
        @(posedge clk); while (!phi_r) @(posedge clk);
        phi_v <= 0;
    end

    // CORDIC rotate model -> FFT input buffer (blocking join of cart+phase)
    reg signed [15:0] fin_r [0:127]; reg signed [15:0] fin_i [0:127];
    integer fin_n = 0; reg fft_busy = 0;
    real th, xr, xi;
    always @* begin cart_r = cart_v && ph_v && !fft_busy; ph_r = cart_r; end
    always @(posedge clk) if (cart_v && ph_v && cart_r) begin
        th = $itor($signed(ph_d)) / 8192.0 * 3.14159265358979;
        xr = $itor($signed(cart_d[15:0])); xi = $itor($signed(cart_d[31:16]));
        fin_r[fin_n] <= $rtoi(xr * $cos(th) - xi * $sin(th));
        fin_i[fin_n] <= $rtoi(xr * $sin(th) + xi * $cos(th));
        if (cart_l !== (fin_n == 127)) $display("ERR tlast at %0d", fin_n);
        if (fin_n == 127) begin fin_n <= 0; fft_busy <= 1; end else fin_n <= fin_n + 1;
    end

    // FFT model: unscaled forward DFT, natural order, 24-bit, xk_index in tuser
    reg [47:0] fo_d; reg [7:0] fo_u; reg fo_l, fo_v = 0; wire fo_r;
    integer k, n2; real sr, sim, a2;
    always @(posedge clk) if (fft_busy && !fo_v && fin_n == 0) begin
        for (k = 0; k < 128; k = k + 1) begin
            sr = 0; sim = 0;
            for (n2 = 0; n2 < 128; n2 = n2 + 1) begin
                a2 = -2.0 * 3.14159265358979 * k * n2 / 128.0;
                sr  = sr  + fin_r[n2] * $cos(a2) - fin_i[n2] * $sin(a2);
                sim = sim + fin_r[n2] * $sin(a2) + fin_i[n2] * $cos(a2);
            end
            fo_d <= {24'($rtoi(sim)), 24'($rtoi(sr))}; fo_u <= k; fo_l <= (k == 127); fo_v <= 1;
            @(posedge clk); while (!fo_r) @(posedge clk);
            fo_v <= 0;
        end
        fft_busy <= 0;
    end

    // ---------------- eq_ctrl ----------------
    wire [31:0] a_d, b_d; wire [7:0] a_u; wire a_l, a_v, b_v; reg a_r, b_r; wire [4:0] shv;
    ofdm_eq_ctrl u_eq (.aclk(clk), .aresetn(rstn),
        .s_axis_tdata(fo_d), .s_axis_tuser(fo_u), .s_axis_tlast(fo_l), .s_axis_tvalid(fo_v),
        .s_axis_tready(fo_r),
        .m_axis_a_tdata(a_d), .m_axis_a_tuser(a_u), .m_axis_a_tlast(a_l), .m_axis_a_tvalid(a_v),
        .m_axis_a_tready(a_r),
        .m_axis_b_tdata(b_d), .m_axis_b_tvalid(b_v), .m_axis_b_tready(b_r), .dbg_shift(shv));

    // cmpy model: 16x16 -> full 33 bit -> keep 24 MSB (truncate), blocking join
    reg [47:0] z_d; reg [7:0] z_u; reg z_l, z_v = 0; wire z_r;
    reg signed [33:0] pr, pi_;
    always @* begin a_r = a_v && b_v && (!z_v || z_r); b_r = a_r; end
    always @(posedge clk) begin
        if (z_v && z_r) z_v <= 0;
        if (a_r) begin
            pr  = $signed(a_d[15:0]) * $signed(b_d[15:0]) - $signed(a_d[31:16]) * $signed(b_d[31:16]);
            pi_ = $signed(a_d[15:0]) * $signed(b_d[31:16]) + $signed(a_d[31:16]) * $signed(b_d[15:0]);
            z_d <= {pi_[32:9], pr[32:9]}; z_u <= a_u; z_l <= a_l; z_v <= 1;
        end
    end

    // ---------------- demap ----------------
    wire [31:0] w_d; wire w_l, w_v; wire [15:0] dfc;
    ofdm_qpsk_demap u_dem (.aclk(clk), .aresetn(rstn),
        .s_axis_tdata(z_d), .s_axis_tuser(z_u), .s_axis_tlast(z_l), .s_axis_tvalid(z_v),
        .s_axis_tready(z_r),
        .m_axis_tdata(w_d), .m_axis_tlast(w_l), .m_axis_tvalid(w_v), .m_axis_tready(1'b1),
        .frame_cnt(dfc));

    always @(posedge clk) if (u_sync.out_go) $display("t=%0t det d1=%0d d2=%0d n0=%0d ncnt=%0d phi=%0d Pb=%0d,%0d", $time, u_sync.d1, u_sync.d2, u_sync.n0, u_sync.ncnt, $signed(u_sync.cfo_phase), u_sync.Pbr, u_sync.Pbi);
    integer fo, nwords = 0;
    initial fo = $fopen("words_out.txt", "w");
    always @(posedge clk) if (w_v) begin
        $fwrite(fo, "%08x %0d\n", w_d, w_l); nwords = nwords + 1;
    end
endmodule
