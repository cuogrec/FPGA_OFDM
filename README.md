# OFDM thủy âm 33 kHz – Bộ thu ZCU104 (Vivado 2024.2)

## 1. Tham số hệ thống

| Mục | Giá trị |
|---|---|
| ADC | PCM1808, fs = 96 kHz, chế độ slave, I2S 24 bit (MD1 MD0 = 00, FMT = 0) |
| SCKI | 24.576 MHz = 256·fs (MMCM D=9, M=88.75, O=40.125 → lệch −0.9 ppm) |
| Sóng mang | 33 kHz (cộng hưởng BII-7511H) |
| Băng tần chiếm | 31 – 35 kHz (64 subcarrier × 62.5 Hz) |
| Baseband | phức 8 kHz (fs/12) |
| FFT / CP | N = 128 (16 ms), CP = 32 (4 ms) |
| Subcarrier | f = −32…−1, +1…+32 (DC trống); pilot f = ±4, ±12, ±20, ±28 (giá trị +1) |
| Dữ liệu | 56 subcarrier QPSK Gray (b0 → dấu I, b1 → dấu Q) |
| Khung | [CP+SC preamble] [CP+LTF] [CP+DATA] × 16 = 2880 mẫu = 360 ms |
| SC preamble | QPSK trên subcarrier chẵn (2 nửa giống nhau, L = 64) |
| LTF | BPSK ±1 trên 64 subcarrier, PN9 (x⁹+x⁵+1, seed 0x1FF), `LTF_BITS = 64'hE78B72904CE8FBC1` |
| Payload | 1792 bit = 224 byte/khung (≈ 5 kbit/s thô) |

`model/ofdm_ref.py` là đặc tả tham chiếu (bộ phát + bảng LTF + hệ số FIR). Bộ phát MATLAB nằm trong `matlab/` (xem `matlab/README_TX.md`), khung giống hệt bản Python (sai khác < 1e-12).
Payload 224 byte: `seq | số khung | độ dài | text 219 | CRC-16/CCITT`, bit LSB trước, làm trắng PN9 seed 0x155 (FPGA xuất bit chưa bỏ làm trắng).

## 2. Chuỗi xử lý (PL)

```
PCM1808 ─I2S─► I2S Receiver(master) ─► i2s_ch_sel ─► Complex Multiplier ◄─ DDS Compiler (−33 kHz)
   ─► FIR Compiler (72 tap, ↓12, 2 path I/Q) ─► AXIS Broadcaster ─┬─► axis_tap_pkt ─► FIFO ─► AXI DMA (raw)
                                                                   └─► ofdm_sync_sc ◄─► CORDIC (translate: góc CFO)
   ─► CORDIC (rotate: bù CFO) ─► xFFT 128 ─► ofdm_eq_ctrl ─► Complex Multiplier (Y·H*) ─► ofdm_qpsk_demap ─► FIFO ─► AXI DMA (bit)
```

IP Vivado dùng: Zynq US+ MPSoC, Clocking Wizard, Processor System Reset ×2, I2S Receiver, DDS Compiler, Complex Multiplier ×2, FIR Compiler, AXIS Broadcaster, CORDIC ×2, Fast Fourier Transform, AXIS Data FIFO ×2, AXI DMA ×2, AXI GPIO ×2, SmartConnect ×2, Concat / Slice / Constant.

RTL tự viết (chỉ phần không có IP): `i2s_ch_sel`, `ofdm_sync_sc`, `ofdm_eq_ctrl`, `ofdm_qpsk_demap`, `axis_tap_pkt`.

## 3. Build

```
vivado -mode batch -source tcl/build.tcl              # project + block design
vivado -mode batch -source tcl/build.tcl -tclargs impl  # + bitstream + XSA
```
Cần board file ZCU104 (có sẵn trong 2024.2). Script tự chọn version IP mới nhất trong catalog; tham số IP nào không tồn tại sẽ được báo `INFO … skipped` thay vì dừng – hãy đọc log.

**Trước khi tạo bitstream:** kiểm tra lại chân PMOD0 (J55) trong `xdc/ofdm33_zcu104.xdc` với master XDC ZCU104.

## 4. Thanh ghi điều khiển (AXI GPIO)

`gpio_cfg` kênh 1:

| Bit | Ý nghĩa | Gợi ý |
|---|---|---|
| [3:0] | gain_shift (dịch trái mẫu ADC 24 bit trước khi lấy 16 bit) | sao cho đỉnh ≈ −6 dBFS |
| [4] | ch_sel: 0 = kênh L, 1 = kênh R | 0 |
| [5] | sync_enable | 1 |
| [6] | tap_enable (bắt baseband thô qua dma_raw, gói 4096 mẫu) | bật sau khi arm DMA |
| [15:8] | thr: ngưỡng metric, đơn vị 1/256 (2|P|/R) | 150 |
| [23:16] | toffs: dịch cửa sổ FFT (có dấu, mẫu); 0 = lùi 8 mẫu vào CP | 0 |
| [26:24] | bb_shift: gain trước CORDIC (bão hòa) | 2 |

`gpio_cfg` kênh 2: `rmin` – năng lượng tối thiểu R để cho phép phát hiện (đặt ≈ 4× giá trị R khi chỉ có nhiễu).

`gpio_stat` kênh 1: [15:0] số khung sync, [31:16] pha CFO (2¹³ = π, ε = phase/8192 subcarrier).
`gpio_stat` kênh 2: [15:0] số khung demap, [31:16] số mẫu I2S bị rơi.

## 5. Phần mềm PS và thử nghiệm

- `sw/ofdm33_app.c` là ứng dụng bare-metal có menu UART: đo nhiễu, đo mức tone, dump baseband, chạy bộ thu. Xem `sw/README_SW.md`.
- `TEST_LOOPBACK.md` mô tả cách nối thẳng Focusrite → PCM1808 → ZCU104 (bộ chia áp, chân MD/FMT nối GND, trình tự đo).
- I2S Receiver: `C_NUM_CHANNELS = 1`, `C_32BIT_LR = 1` → BCK = 64 fs (PCM1808 với SCKI 256 fs không nhận 48 fs). PS ghi `0x30 = 1`, `0x20 = 2`, `0x08 = 1`.

## 6. Kiểm chứng đã làm

Mô phỏng Icarus end-to-end RTL (sync → CORDIC → FFT → eq → cmpy → demap; IP thay bằng mô hình hành vi) với tín hiệu Python: TX 33 kHz → kênh 4 tia (trễ tới 1.56 ms) → AWGN → lệch đồng hồ → ADC 24 bit → mô hình DDC bit-true.

| SNR (băng 4 kHz) | Lệch clock | Lỗi bit / 1792 (3 khung) |
|---|---|---|
| 60 dB | −500 ppm | 0 / 0 / 0 |
| 30 dB | +300 ppm | 0 / 0 / 0 |
| 25 dB | 40 ppm | 0 / 0 / 0 |
| 20 dB | 100 ppm | 1 / 10 / 6 |
| 15 dB | 40 ppm | 35 / 77 / 43 (fading đa đường, chưa có FEC) |

Ước lượng CFO khớp lý thuyết (−500 ppm → −2162, lý thuyết −2163). Chạy lại: `cd sim; python3 gen_vectors.py <seed> <snr> <ppm>; iverilog -g2012 -o tb tb_rx.v ../rtl/ofdm_*.v; vvp tb; python3 check.py`.

Kiểm tra chéo toàn chuỗi: WAV do `matlab/run_tx.m` tạo → kênh 4 tia, 25 dB, 80 ppm → mô hình DDC → RTL → `ofdm_decode_dma`: 5/5 khung CRC OK, ra đúng văn bản.
Sửa ngày 29/10: `ofdm_qpsk_demap` không còn ngoại suy pilot ở mép băng (|f| > 28 dùng pilot gần nhất). Bản cũ có thể đảo dấu khi kênh fading mạnh.

Chưa kiểm được: chạy Vivado thật (tên tham số IP, timing). Tcl đã chạy khô với lệnh Vivado giả lập.
