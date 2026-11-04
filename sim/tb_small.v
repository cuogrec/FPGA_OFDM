`timescale 1ns/1ps
module tb_small;
  reg clk=0, rstn=0; always #5 clk=~clk;
  reg [31:0] d; reg [2:0] id; reg v=0; wire [31:0] o; wire ov; reg [3:0] g=0;
  i2s_ch_sel u(.aclk(clk),.aresetn(rstn),.s_axis_tdata(d),.s_axis_tid(id),.s_axis_tvalid(v),.s_axis_tready(),
     .m_axis_tdata(o),.m_axis_tvalid(ov),.m_axis_tready(1'b1),.gain_shift(g),.ch_sel(1'b0),.drop_cnt());
  wire [31:0] to; wire tl, tv; reg en=0; integer np=0, nl=0;
  axis_tap_pkt #(.PKT_LEN(8)) t(.aclk(clk),.aresetn(rstn),.s_axis_tdata(d),.s_axis_tvalid(v),.s_axis_tready(),
     .m_axis_tdata(to),.m_axis_tlast(tl),.m_axis_tvalid(tv),.m_axis_tready(1'b1),.enable(en),.drop_cnt());
  always @(posedge clk) if (tv) begin np=np+1; if (tl) nl=nl+1; end
  task send(input [23:0] s, input [2:0] c); begin @(posedge clk); d <= {4'h0, s, 4'h0}; id <= c; v <= 1; @(posedge clk); v <= 0;
     @(posedge clk); @(posedge clk); end endtask
  initial begin
    repeat(3) @(posedge clk); rstn=1;
    send(24'h123456,0); $display("L 0x123456 g0 -> %h (exp 1234)", o[15:0]);
    send(24'hABCDEF,1); $display("R ignored   -> %h (exp 1234)", o[15:0]);
    g=4; send(24'h012345,0); $display("g4 0x012345 -> %h (exp 1234)", o[15:0]);
    send(24'h123456,0); $display("g4 sat +    -> %h (exp 7fff)", o[15:0]);
    send(24'hF00000,0); $display("g4 sat -    -> %h (exp 8000)", o[15:0]);
    en=1; repeat(20) send(24'h1,0); en=0; repeat(20) send(24'h1,0);
    $display("tap: %0d samples, %0d tlast (exp 24,3)", np, nl); $finish;
  end
endmodule
