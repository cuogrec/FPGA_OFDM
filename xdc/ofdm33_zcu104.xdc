# =============================================================================
#  PCM1808 on ZCU104 PMOD0 (J55, 3.3 V) -- wiring used on the bench
#     J55.1 -> G8 : BCK      J55.3 -> H8 : DOUT (PCM1808 OUT)
#     J55.5 -> G7 : LRCK     J55.7 -> H7 : SCKI (PCM1808 SCK / MCLK)
#  MD0, MD1, FMT are tied to GND on the module (slave, I2S 24 bit) -> not in this file.
#  (G8 H8 G7 H7 = PMOD0[0..3] in the ZCU104 constraints of the Xilinx PYNQ project)
# =============================================================================
set_property -dict {PACKAGE_PIN G8 IOSTANDARD LVCMOS33} [get_ports pcm_bck]
set_property -dict {PACKAGE_PIN H8 IOSTANDARD LVCMOS33} [get_ports pcm_dout]
set_property -dict {PACKAGE_PIN G7 IOSTANDARD LVCMOS33} [get_ports pcm_lrck]
set_property -dict {PACKAGE_PIN H7 IOSTANDARD LVCMOS33} [get_ports pcm_scki]

set_property SLEW SLOW  [get_ports {pcm_scki pcm_bck pcm_lrck}]
set_property DRIVE 8    [get_ports {pcm_scki pcm_bck pcm_lrck}]
set_property PULLDOWN true [get_ports pcm_dout]

# audio interface is slow (BCK 6.144 MHz, generated and sampled inside the
# aud_mclk domain of the I2S Receiver) -> no IO timing requirement
set_false_path -to   [get_ports {pcm_scki pcm_bck pcm_lrck}]
set_false_path -from [get_ports pcm_dout]