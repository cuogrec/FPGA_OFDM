# =============================================================================
#  ofdm33_rx_bd.tcl  --  Vivado 2024.2 block design: OFDM underwater RX, ZCU104
#
#  PCM1808 (96 kHz, I2S slave) -> I2S Receiver (master) -> i2s_ch_sel
#   -> [DDS Compiler -33 kHz] x [Complex Multiplier]  (digital down-conversion)
#   -> FIR Compiler (72 tap, decimate 12, 2 paths I/Q) -> 8 kHz complex baseband
#   -> AXIS Broadcaster -> (a) axis_tap_pkt -> FIFO -> AXI DMA (raw capture)
#                       -> (b) ofdm_sync_sc (Schmidl-Cox) <-> CORDIC translate (CFO)
#   -> CORDIC rotate (CFO correction) -> xFFT 128 (unscaled, natural order)
#   -> ofdm_eq_ctrl (LTF channel estimate) -> Complex Multiplier (Y*conj(H))
#   -> ofdm_qpsk_demap (pilot phase tracking, QPSK) -> FIFO -> AXI DMA -> DDR
#  Control: AXI GPIO (cfg) / AXI GPIO (status); IRQ: DMA x2 + I2S -> pl_ps_irq0
#
#  Usage (from build.tcl, project already created with ZCU104 board part):
#     source ofdm33_rx_bd.tcl
# =============================================================================

set design_name ofdm33_rx_bd
create_bd_design $design_name
current_bd_design $design_name

# ---------- helpers ----------------------------------------------------------
# newest VLNV of an IP in the installed catalog
proc ipv {name {lib ip}} {
    set l [lsort -dictionary [get_ipdefs -all -quiet xilinx.com:${lib}:${name}:*]]
    if {[llength $l] == 0} { error "IP xilinx.com:${lib}:${name} not found in catalog" }
    return [lindex $l end]
}
# create IP cell
proc mkip {name inst} { return [create_bd_cell -type ip -vlnv [ipv $name] $inst] }
# apply CONFIG.* properties; skip (with warning) names that this IP version
# does not have, so alternate spellings can be listed side by side
proc cfg {cell kv} {
    set have [list_property [get_bd_cells $cell]]
    set d {}
    foreach {k v} $kv {
        if {[lsearch -exact $have CONFIG.$k] >= 0} { lappend d CONFIG.$k $v } \
        else { puts "INFO: \[$cell\] has no CONFIG.$k (skipped)" }
    }
    if {[catch {set_property -dict $d [get_bd_cells $cell]} err]} {
        puts "WARNING: \[$cell\] dict apply failed ($err) -> applying one by one"
        foreach {k v} $d {
            if {[catch {set_property $k $v [get_bd_cells $cell]} e2]} {
                puts "WARNING: \[$cell\] $k = $v : $e2"
            }
        }
    }
}
proc slice {inst from to {w 32}} {
    set c [create_bd_cell -type ip -vlnv [ipv xlslice] $inst]
    set_property -dict [list CONFIG.DIN_WIDTH $w CONFIG.DIN_FROM $from CONFIG.DIN_TO $to] $c
    return $c
}
proc const {inst width val} {
    set c [create_bd_cell -type ip -vlnv [ipv xlconstant] $inst]
    set_property -dict [list CONFIG.CONST_WIDTH $width CONFIG.CONST_VAL $val] $c
    return $c
}

# =============================================================================
# 1. Zynq UltraScale+ MPSoC (ZCU104 preset)
# =============================================================================
set ps [mkip zynq_ultra_ps_e zynq_ps]
apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e \
    -config {apply_board_preset "1"} $ps
cfg zynq_ps {
    PSU__USE__M_AXI_GP0                  1
    PSU__USE__M_AXI_GP1                  0
    PSU__USE__M_AXI_GP2                  0
    PSU__MAXIGP0__DATA_WIDTH             32
    PSU__USE__S_AXI_GP2                  1
    PSU__SAXIGP2__DATA_WIDTH             128
    PSU__USE__IRQ0                       1
    PSU__FPGA_PL0_ENABLE                 1
    PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ  100
}

# =============================================================================
# 2. Clocks and resets
#    pl_clk0 100 MHz : AXI + all DSP
#    clk_out1 24.576 MHz : PCM1808 SCKI = 256 fs, I2S Receiver aud_mclk
#    MMCM D=9, M=88.750, O=40.125 -> 24.575978 MHz (-0.9 ppm)
# =============================================================================
set clkw [mkip clk_wiz clk_audio]
cfg clk_audio {
    PRIM_IN_FREQ                 100.000
    PRIM_SOURCE                  No_buffer
    CLKOUT1_REQUESTED_OUT_FREQ   24.576
    USE_LOCKED                   true
    USE_RESET                    true
    RESET_TYPE                   ACTIVE_LOW
}
# pin the fractional MMCM solution (ignored with a warning if the wizard re-derives it)
cfg clk_audio {
    MMCM_DIVCLK_DIVIDE      9
    MMCM_CLKFBOUT_MULT_F    88.750
    MMCM_CLKOUT0_DIVIDE_F   40.125
}

set rst_sys  [mkip proc_sys_reset rst_sys]
set rst_mclk [mkip proc_sys_reset rst_mclk]

connect_bd_net [get_bd_pins zynq_ps/pl_clk0] \
    [get_bd_pins zynq_ps/maxihpm0_fpd_aclk] [get_bd_pins zynq_ps/saxihp0_fpd_aclk] \
    [get_bd_pins clk_audio/clk_in1] [get_bd_pins rst_sys/slowest_sync_clk]
connect_bd_net [get_bd_pins zynq_ps/pl_resetn0] [get_bd_pins rst_sys/ext_reset_in] \
    [get_bd_pins clk_audio/resetn] [get_bd_pins rst_mclk/ext_reset_in]
connect_bd_net [get_bd_pins clk_audio/clk_out1] [get_bd_pins rst_mclk/slowest_sync_clk]
connect_bd_net [get_bd_pins clk_audio/locked]   [get_bd_pins rst_mclk/dcm_locked]

set CLK  [get_bd_pins zynq_ps/pl_clk0]
set RSTN [get_bd_pins rst_sys/peripheral_aresetn]

# =============================================================================
# 3. PCM1808 interface (external ports)
# =============================================================================
create_bd_port -dir O -type clk pcm_scki
create_bd_port -dir O pcm_bck
create_bd_port -dir O pcm_lrck
create_bd_port -dir I pcm_dout
connect_bd_net [get_bd_pins clk_audio/clk_out1] [get_bd_ports pcm_scki]
# MD0, MD1, FMT of the PCM1808 are tied to GND on the module (slave mode, I2S 24 bit)
# -> no FPGA pins / ports for them

# I2S Receiver (master). 32-bit LR clock -> BCK = 64 fs = 6.144 MHz, LRCK = fs
#   (PCM1808 slave accepts 64 BCK/frame with 256 fs SCKI; 48 BCK would need 384 fs)
#   C_NUM_CHANNELS = audio channels (valid 2/4/6/8); 2 = one I2S data line (L + R)
#   PS must write: 0x30 = 1 (channel 0/1), 0x20 = MCLK/(2*SCLK) = 2, 0x08 = 1 (enable)
mkip i2s_receiver i2s_rx
cfg i2s_rx {
    C_IS_MASTER     1
    C_NUM_CHANNELS  2
    C_DWIDTH        24
    C_32BIT_LR      1
    C_DEPTH         128
}
connect_bd_net [get_bd_pins clk_audio/clk_out1]        [get_bd_pins i2s_rx/aud_mclk]
connect_bd_net [get_bd_pins rst_mclk/peripheral_reset] [get_bd_pins i2s_rx/aud_mrst]
connect_bd_net $CLK  [get_bd_pins i2s_rx/s_axi_ctrl_aclk] [get_bd_pins i2s_rx/m_axis_aud_aclk]
connect_bd_net $RSTN [get_bd_pins i2s_rx/s_axi_ctrl_aresetn] [get_bd_pins i2s_rx/m_axis_aud_aresetn]
connect_bd_net [get_bd_ports pcm_dout] [get_bd_pins i2s_rx/sdata_0_in]
connect_bd_net [get_bd_pins i2s_rx/sclk_out]  [get_bd_ports pcm_bck]
connect_bd_net [get_bd_pins i2s_rx/lrclk_out] [get_bd_ports pcm_lrck]

# =============================================================================
# 4. Custom RTL blocks (module references)
# =============================================================================
create_bd_cell -type module -reference i2s_ch_sel      u_chsel
create_bd_cell -type module -reference axis_tap_pkt    u_tap
create_bd_cell -type module -reference ofdm_sync_sc    u_sync
create_bd_cell -type module -reference ofdm_eq_ctrl    u_eq
create_bd_cell -type module -reference ofdm_qpsk_demap u_demap

connect_bd_intf_net [get_bd_intf_pins i2s_rx/m_axis_aud] [get_bd_intf_pins u_chsel/s_axis]

# =============================================================================
# 5. Digital down-converter: DDS (-33 kHz @ fs) x Complex Multiplier, FIR /12
#    PINC = (1 - 33/96) * 2^16 = 43008 -> exp(-j*2*pi*33/96*n) exactly
#    DDS steps only when the multiplier consumes (TREADY) => locked to fs
# =============================================================================
mkip dds_compiler dds_lo
cfg dds_lo {
    PartsPresent            Phase_Generator_and_SIN_COS_LUT
    Parameter_Entry         Hardware_Parameters
    Phase_Width             16
    Output_Width            16
    Phase_Increment         Fixed
    Phase_offset            None
    PINC1                   1010100000000000
    Output_Selection        Sine_and_Cosine
    Amplitude_Mode          Full_Range
    Noise_Shaping           None
    Has_Phase_Out           false
    Has_TREADY              true
    DATA_Has_TLAST          Not_Required
    Has_ARESETn             false
    Latency_Configuration   Auto
}
connect_bd_net $CLK [get_bd_pins dds_lo/aclk]

mkip cmpy mix
cfg mix {
    APortWidth      16
    BPortWidth      16
    OutputWidth     16
    FlowControl     Blocking
    MultType        Use_Mults
    OptimizeGoal    Resources
    RoundMode       Truncate
    LatencyConfig   Automatic
}
connect_bd_net $CLK [get_bd_pins mix/aclk]
connect_bd_intf_net [get_bd_intf_pins u_chsel/m_axis]       [get_bd_intf_pins mix/S_AXIS_A]
connect_bd_intf_net [get_bd_intf_pins dds_lo/M_AXIS_DATA]   [get_bd_intf_pins mix/S_AXIS_B]

mkip fir_compiler fir_dec
cfg fir_dec {
    CoefficientSource       Vector
    Coefficient_Sets        1
    CoefficientVector       {0.000752807,0.000920901,0.001168781,0.001695484,0.001983058,0.002422575,0.002547267,0.002670707,0.002383598,0.001955289,0.001055015,-0.000056499,-0.001608554,-0.003312607,-0.005286226,-0.007198863,-0.009061883,-0.010515769,-0.011499762,-0.011664331,-0.010936769,-0.009035801,-0.005945730,-0.001512742,0.004162784,0.011066052,0.018953760,0.027638258,0.036750330,0.045971553,0.054865944,0.063064734,0.070154510,0.075814193,0.079747250,0.081767733,0.081767733,0.079747250,0.075814193,0.070154510,0.063064734,0.054865944,0.045971553,0.036750330,0.027638258,0.018953760,0.011066052,0.004162784,-0.001512742,-0.005945730,-0.009035801,-0.010936769,-0.011664331,-0.011499762,-0.010515769,-0.009061883,-0.007198863,-0.005286226,-0.003312607,-0.001608554,-0.000056499,0.001055015,0.001955289,0.002383598,0.002670707,0.002547267,0.002422575,0.001983058,0.001695484,0.001168781,0.000920901,0.000752807}
    Filter_Type             Decimation
    Decimation_Rate         12
    Interpolation_Rate      1
    Number_Channels         1
    Number_Paths            2
    Sample_Frequency        0.096
    Clock_Frequency         100
    Coefficient_Sign        Signed
    Coefficient_Width       16
    Quantization            Quantize_Only
    Coefficient_Structure   Inferred
    Data_Sign               Signed
    Data_Width              16
    Data_Fractional_Bits    0
    Output_Rounding_Mode    Truncate_LSBs
    Output_Width            16
    Has_ARESETn             true
    M_DATA_Has_TREADY       true
    S_DATA_Has_FIFO         true
}
connect_bd_net $CLK  [get_bd_pins fir_dec/aclk]
connect_bd_net $RSTN [get_bd_pins fir_dec/aresetn]
connect_bd_intf_net [get_bd_intf_pins mix/M_AXIS_DOUT] [get_bd_intf_pins fir_dec/S_AXIS_DATA]

# split baseband: sync + debug tap (both always ready, never stall the DDC)
mkip axis_broadcaster bb_split
cfg bb_split {
    NUM_MI             2
    S_TDATA_NUM_BYTES  4
    M_TDATA_NUM_BYTES  4
    M00_TDATA_REMAP    {tdata[31:0]}
    M01_TDATA_REMAP    {tdata[31:0]}
}
connect_bd_net $CLK  [get_bd_pins bb_split/aclk]
connect_bd_net $RSTN [get_bd_pins bb_split/aresetn]
connect_bd_intf_net [get_bd_intf_pins fir_dec/M_AXIS_DATA] [get_bd_intf_pins bb_split/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins bb_split/M00_AXIS]   [get_bd_intf_pins u_sync/s_axis]
connect_bd_intf_net [get_bd_intf_pins bb_split/M01_AXIS]   [get_bd_intf_pins u_tap/s_axis]

# =============================================================================
# 6. Synchronisation + CFO: CORDIC translate (angle of P), CORDIC rotate
#    Scaled radians, 16 bit (2^13 = pi); cartesian 16 bit (2^14 = 1.0)
# =============================================================================
mkip cordic cfo_atan
cfg cfo_atan {
    Functional_Selection        Translate
    Architectural_Configuration Word_Serial
    Pipelining_Mode             Optimal
    Data_Format                 SignedFraction
    Phase_Format                Scaled_Radians
    Input_Width                 16
    Output_Width                16
    Round_Mode                  Truncate
    Coarse_Rotation             true
    Compensation_Scaling        No_Scale_Compensation
    flow_control                Blocking
    out_tready                  true
}
mkip cordic cfo_rot
cfg cfo_rot {
    Functional_Selection        Rotate
    Architectural_Configuration Word_Serial
    Pipelining_Mode             Optimal
    Data_Format                 SignedFraction
    Phase_Format                Scaled_Radians
    Input_Width                 16
    Output_Width                16
    Round_Mode                  Truncate
    Coarse_Rotation             true
    Compensation_Scaling        Embedded_Multiplier
    flow_control                Blocking
    out_tready                  true
    cartesian_has_tlast         true
}
connect_bd_net $CLK [get_bd_pins cfo_atan/aclk] [get_bd_pins cfo_rot/aclk]
connect_bd_intf_net [get_bd_intf_pins u_sync/m_axis_cfo]     [get_bd_intf_pins cfo_atan/S_AXIS_CARTESIAN]
connect_bd_intf_net [get_bd_intf_pins cfo_atan/M_AXIS_DOUT]  [get_bd_intf_pins u_sync/s_axis_phi]
connect_bd_intf_net [get_bd_intf_pins u_sync/m_axis_cart]    [get_bd_intf_pins cfo_rot/S_AXIS_CARTESIAN]
connect_bd_intf_net [get_bd_intf_pins u_sync/m_axis_phase]   [get_bd_intf_pins cfo_rot/S_AXIS_PHASE]

# =============================================================================
# 7. FFT 128 (forward, unscaled -> 24 bit, natural order, xk_index in TUSER)
# =============================================================================
mkip xfft fft128
cfg fft128 {
    transform_length                        128
    implementation_options                  radix_2_lite_burst_io
    target_clock_frequency                  100
    run_time_configurable_transform_length  false
    data_format                             fixed_point
    input_width                             16
    phase_factor_width                      16
    scaling_options                         unscaled
    rounding_modes                          truncation
    output_ordering                         natural_order
    xk_index                                true
    throttle_scheme                         nonrealtime
    aresetn                                 true
    memory_options_data                     block_ram
    memory_options_phase_factors            block_ram
}
connect_bd_net $CLK  [get_bd_pins fft128/aclk]
connect_bd_net $RSTN [get_bd_pins fft128/aresetn]
connect_bd_intf_net [get_bd_intf_pins cfo_rot/M_AXIS_DOUT] [get_bd_intf_pins fft128/S_AXIS_DATA]
# config channel: FWD_INV = 1 (forward), held valid
const fft_cfg_d 8 1
const one1 1 1
connect_bd_net [get_bd_pins fft_cfg_d/dout] [get_bd_pins fft128/s_axis_config_tdata]
connect_bd_net [get_bd_pins one1/dout]      [get_bd_pins fft128/s_axis_config_tvalid]
connect_bd_intf_net [get_bd_intf_pins fft128/M_AXIS_DATA] [get_bd_intf_pins u_eq/s_axis]

# =============================================================================
# 8. Equaliser: Complex Multiplier Z = Y * conj(H)  (16x16 -> 24 bit MSBs)
# =============================================================================
mkip cmpy eq_mult
cfg eq_mult {
    APortWidth      16
    BPortWidth      16
    OutputWidth     24
    FlowControl     Blocking
    HasATLAST       true
    HasATUSER       true
    ATUSERWidth     8
    OutTLASTBehv    Pass_A_TLAST
    MultType        Use_Mults
    OptimizeGoal    Resources
    RoundMode       Truncate
    LatencyConfig   Automatic
}
connect_bd_net $CLK [get_bd_pins eq_mult/aclk]
connect_bd_intf_net [get_bd_intf_pins u_eq/m_axis_a]        [get_bd_intf_pins eq_mult/S_AXIS_A]
connect_bd_intf_net [get_bd_intf_pins u_eq/m_axis_b]        [get_bd_intf_pins eq_mult/S_AXIS_B]
connect_bd_intf_net [get_bd_intf_pins eq_mult/M_AXIS_DOUT]  [get_bd_intf_pins u_demap/s_axis]

# =============================================================================
# 9. Output FIFOs + AXI DMA (S2MM only) -> HP0 -> DDR
# =============================================================================
mkip axis_data_fifo fifo_bits
cfg fifo_bits { FIFO_DEPTH 1024 TDATA_NUM_BYTES 4 }
mkip axis_data_fifo fifo_raw
cfg fifo_raw  { FIFO_DEPTH 8192 TDATA_NUM_BYTES 4 }
connect_bd_net $CLK  [get_bd_pins fifo_bits/s_axis_aclk]    [get_bd_pins fifo_raw/s_axis_aclk]
connect_bd_net $RSTN [get_bd_pins fifo_bits/s_axis_aresetn] [get_bd_pins fifo_raw/s_axis_aresetn]
connect_bd_intf_net [get_bd_intf_pins u_demap/m_axis] [get_bd_intf_pins fifo_bits/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins u_tap/m_axis]   [get_bd_intf_pins fifo_raw/S_AXIS]

foreach d {dma_bits dma_raw} {
    mkip axi_dma $d
    cfg $d {
        c_include_sg              0
        c_include_mm2s            0
        c_include_s2mm            1
        c_sg_length_width         23
        c_addr_width              32
        c_s_axis_s2mm_tdata_width 32
        c_m_axi_s2mm_data_width   32
        c_s2mm_burst_size         16
    }
    connect_bd_net $CLK  [get_bd_pins $d/s_axi_lite_aclk] [get_bd_pins $d/m_axi_s2mm_aclk]
    connect_bd_net $RSTN [get_bd_pins $d/axi_resetn]
}
connect_bd_intf_net [get_bd_intf_pins fifo_bits/M_AXIS] [get_bd_intf_pins dma_bits/S_AXIS_S2MM]
connect_bd_intf_net [get_bd_intf_pins fifo_raw/M_AXIS]  [get_bd_intf_pins dma_raw/S_AXIS_S2MM]

# =============================================================================
# 10. Control / status GPIO
#   gpio_cfg  ch1: [3:0] gain_shift [4] ch_sel [5] sync_en [6] tap_en
#                  [15:8] thr  [23:16] toffs (signed)  [26:24] bb_shift
#             ch2: rmin
#   gpio_stat ch1: [15:0] sync frames  [31:16] CFO phase (2^13 = pi)
#             ch2: [15:0] demap frames [31:16] I2S drop count
# =============================================================================
mkip axi_gpio gpio_cfg
cfg gpio_cfg {
    C_ALL_OUTPUTS    1
    C_GPIO_WIDTH     32
    C_IS_DUAL        1
    C_ALL_OUTPUTS_2  1
    C_GPIO2_WIDTH    32
}
mkip axi_gpio gpio_stat
cfg gpio_stat {
    C_ALL_INPUTS     1
    C_GPIO_WIDTH     32
    C_IS_DUAL        1
    C_ALL_INPUTS_2   1
    C_GPIO2_WIDTH    32
}
connect_bd_net $CLK  [get_bd_pins gpio_cfg/s_axi_aclk]    [get_bd_pins gpio_stat/s_axi_aclk]
connect_bd_net $RSTN [get_bd_pins gpio_cfg/s_axi_aresetn] [get_bd_pins gpio_stat/s_axi_aresetn]

slice sl_gain   3  0
slice sl_chsel  4  4
slice sl_synen  5  5
slice sl_tapen  6  6
slice sl_thr   15  8
slice sl_toffs 23 16
slice sl_bbsh  26 24
foreach s {sl_gain sl_chsel sl_synen sl_tapen sl_thr sl_toffs sl_bbsh} {
    connect_bd_net [get_bd_pins gpio_cfg/gpio_io_o] [get_bd_pins $s/Din]
}
connect_bd_net [get_bd_pins sl_gain/Dout]  [get_bd_pins u_chsel/gain_shift]
connect_bd_net [get_bd_pins sl_chsel/Dout] [get_bd_pins u_chsel/ch_sel]
connect_bd_net [get_bd_pins sl_synen/Dout] [get_bd_pins u_sync/enable]
connect_bd_net [get_bd_pins sl_tapen/Dout] [get_bd_pins u_tap/enable]
connect_bd_net [get_bd_pins sl_thr/Dout]   [get_bd_pins u_sync/thr]
connect_bd_net [get_bd_pins sl_toffs/Dout] [get_bd_pins u_sync/toffs]
connect_bd_net [get_bd_pins sl_bbsh/Dout]  [get_bd_pins u_sync/bb_shift]
connect_bd_net [get_bd_pins gpio_cfg/gpio2_io_o] [get_bd_pins u_sync/rmin]

set cc1 [mkip xlconcat st_cat1]
set_property -dict [list CONFIG.NUM_PORTS 2 CONFIG.IN0_WIDTH 16 CONFIG.IN1_WIDTH 16] $cc1
connect_bd_net [get_bd_pins u_sync/frame_cnt] [get_bd_pins st_cat1/In0]
connect_bd_net [get_bd_pins u_sync/cfo_phase] [get_bd_pins st_cat1/In1]
connect_bd_net [get_bd_pins st_cat1/dout]     [get_bd_pins gpio_stat/gpio_io_i]
set cc2 [mkip xlconcat st_cat2]
set_property -dict [list CONFIG.NUM_PORTS 2 CONFIG.IN0_WIDTH 16 CONFIG.IN1_WIDTH 16] $cc2
connect_bd_net [get_bd_pins u_demap/frame_cnt] [get_bd_pins st_cat2/In0]
connect_bd_net [get_bd_pins u_chsel/drop_cnt]  [get_bd_pins st_cat2/In1]
connect_bd_net [get_bd_pins st_cat2/dout]      [get_bd_pins gpio_stat/gpio2_io_i]

# clocks/resets of the RTL blocks
foreach m {u_chsel u_tap u_sync u_eq u_demap} {
    connect_bd_net $CLK  [get_bd_pins $m/aclk]
    connect_bd_net $RSTN [get_bd_pins $m/aresetn]
}

# =============================================================================
# 11. AXI interconnect (SmartConnect) and interrupts
# =============================================================================
mkip smartconnect sc_ctrl
cfg sc_ctrl { NUM_SI 1 NUM_MI 5 }
mkip smartconnect sc_mem
cfg sc_mem  { NUM_SI 2 NUM_MI 1 }
connect_bd_net $CLK  [get_bd_pins sc_ctrl/aclk]    [get_bd_pins sc_mem/aclk]
connect_bd_net $RSTN [get_bd_pins sc_ctrl/aresetn] [get_bd_pins sc_mem/aresetn]

connect_bd_intf_net [get_bd_intf_pins zynq_ps/M_AXI_HPM0_FPD] [get_bd_intf_pins sc_ctrl/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M00_AXI] [get_bd_intf_pins i2s_rx/s_axi_ctrl]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M01_AXI] [get_bd_intf_pins dma_bits/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M02_AXI] [get_bd_intf_pins dma_raw/S_AXI_LITE]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M03_AXI] [get_bd_intf_pins gpio_cfg/S_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_ctrl/M04_AXI] [get_bd_intf_pins gpio_stat/S_AXI]

connect_bd_intf_net [get_bd_intf_pins dma_bits/M_AXI_S2MM] [get_bd_intf_pins sc_mem/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins dma_raw/M_AXI_S2MM]  [get_bd_intf_pins sc_mem/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins sc_mem/M00_AXI] [get_bd_intf_pins zynq_ps/S_AXI_HP0_FPD]

set irq [mkip xlconcat irq_cat]
set_property CONFIG.NUM_PORTS 3 $irq
connect_bd_net [get_bd_pins dma_bits/s2mm_introut] [get_bd_pins irq_cat/In0]
connect_bd_net [get_bd_pins dma_raw/s2mm_introut]  [get_bd_pins irq_cat/In1]
connect_bd_net [get_bd_pins i2s_rx/irq]            [get_bd_pins irq_cat/In2]
connect_bd_net [get_bd_pins irq_cat/dout]          [get_bd_pins zynq_ps/pl_ps_irq0]

# =============================================================================
# 12. Addresses, validate, save
# =============================================================================
assign_bd_address
regenerate_bd_layout
validate_bd_design
# report the MMCM solution actually chosen (must give CLKOUT1 ~ 24.576 MHz)
foreach pat {CONFIG.MMCM_DIVCLK* CONFIG.MMCM_CLKFBOUT_MULT_F CONFIG.MMCM_CLKOUT0_DIVIDE_F CONFIG.CLKOUT1_*FREQ* CONFIG.CLKOUT1_*ACTUAL*} {
    foreach pr [list_property [get_bd_cells clk_audio] $pat] {
        puts "INFO: clk_audio $pr = [get_property $pr [get_bd_cells clk_audio]]"
    }
}
save_bd_design
puts "INFO: $design_name created."
