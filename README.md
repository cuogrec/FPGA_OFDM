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

**Chân PMOD0 (J55):** `xdc/ofdm33_zcu104.xdc` theo đúng cách đấu trên bàn thử: BCK→G8, OUT→H8, LRCK→G7, SCK→H7. MD0/MD1/FMT nối GND trên module, không có trong XDC.

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
- I2S Receiver: `C_NUM_CHANNELS = 2` (1 đường dữ liệu I2S), `C_32BIT_LR = 1` → BCK = 64 fs (PCM1808 với SCKI 256 fs không nhận 48 fs). PS ghi `0x30 = 1`, `0x20 = 2`, `0x08 = 1`.

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

Đã build thật bằng Vivado/Vitis 2024.2 và chạy trên ZCU104 (xem mục 7).

## 7. FEC, công cụ đo và kết quả thử trong nước (xô, ~1 m)

**FEC (tùy chọn, lệnh `e` trên board, `S.fec = true` trong `run_tx.m`):** mã chập K = 7, R = 1/2 (133/171), xen kẽ khối 112 × 16, Viterbi quyết định cứng trên PS. 111 byte tin → 224 byte mã, 106 byte văn bản/khung (~1,66 kbit/s). MATLAB: `ofdm_fec_frame.m`, `ofdm_fec_unframe.m`, `ofdm_fec_payloads.m`.

**Công cụ đo (MATLAB):**

| File | Việc |
|---|---|
| `rx_log.m` | Điều khiển board qua COM (thay Tera Term): `g` thu và ghi CSV/log, `d` dump băng gốc, `m` theo dõi tone |
| `ofdm_channel_report.m` | Từ file dump: SNR từng sóng mang, đáp ứng tần số, trễ đa đường, độ ổn định kênh |
| `test_a1.m`, `test_a1b.m` | Thử dự trữ công suất: quét biên độ phát và `gain_shift`, có/không FEC |
| `test_s2.m` | Một buổi đo: báo động giả (A2), chạy dài 1000 khung (A3), đáy ở gain 2 (A1c) |
| `test_agc.m` | Kiểm tự động mức (app rev 3): quét biên độ mà không gửi lệnh gain hay rmin |

**Kết quả (10/11/2026, xô, test mode, 99 khung mỗi mức):**

- `gain_shift 4` cắt (bão hòa 16 bit trong `i2s_ch_sel`) khi đỉnh ADC > −24 dBFS (ngưỡng = −6·gain_shift dBFS). Ở biên độ phát 0,9: gain 4 → 1/99 khung đúng, gain 3 → 99/99 (29 bit kênh sai/khung), gain 2 → 99/99 (0,3 bit/khung). Phía phát (Focusrite, bộ khuếch đại, đầu dò) vẫn tuyến tính ở 0,9.
- Giới hạn dưới (gain 4): có FEC 99/99 tới biên độ 0,005 (−45 dB so với 0,9), 94/99 ở 0,003 (−49,5 dB). Không FEC: ≥ 96 % tới 0,012 (−37,5 dB), 78 % ở 0,008, 19 % ở 0,005.
- Lợi ích FEC ở ngưỡng 90 % khung đúng: ≥ 11 dB (gồm 3 dB do tốc độ mã 1/2). FEC còn sửa được các khung lỗi lẻ tẻ 1–50 bit do nhiễu xung.
- Ở mức thấp nhất, bộ dò bắt nhầm (rmin 3000) bắt đầu làm mất khung: đồng bộ thành giới hạn cùng với BER.

**Kết quả (12/11/2026, xô, `test_s2.m`, gain_shift 2):**

- Báo động giả (A2): không phát, bộ khuếch đại bật, rmin 3858, thr 180 → 0 khung trong 5 phút.
- Chạy dài (A3): 1000/1000 khung FEC đúng ở amp 0,5; 0 bit kênh sai trên 1 792 000 bit; CFO 0,17–0,30 Hz; không mất mẫu I2S.
- Văn bản nhiều khung (A4): 4 khung (khoảng 325 byte) ghép đúng `>>> message complete`. Ở amp 0,9 với gain 4, mỗi khung phải sửa 104–136 bit và một khung luôn hỏng; về gain 2 thì hết.
- Đáy ở gain 2 (A1c): khung thật vẫn 0 bit sai tới amp 0,012 (−37,5 dB), 9 bit sửa/khung ở 0,008, 76 ở 0,005 (69/83 khung đúng), hỏng hẳn ở 0,003. Cùng mức đó ở gain 4 chỉ 0,8 / 1,6 / 11,5 bit. Gain 2 kém gain 4 khoảng 8–10 dB ở đáy vì lượng tử hóa 16 bit.
- Trong A1c, sàn rmin 200 quá thấp: 15–60 % số lần bắt là bắt nhầm trong khoảng nghỉ (CFO lệch tới ±8 Hz, lệch nhịp 510 ms), mỗi lần làm mất một khung thật. Tỉ lệ khung đúng của A1c vì thế phản ánh bộ dò, không phản ánh giải mã.

Kết luận: không có một gain_shift cố định nào dùng được cho cả dải. Gain 4 bão hòa từ amp 0,35 trở lên, gain 2 mất khoảng 8–10 dB ở đáy. Cần AGC (tự chọn gain_shift theo đỉnh đo được) và rmin tự động có sàn theo nhiễu nền; đây là việc tiếp theo.

**App rev 3 (12/11/2026):** tự động mức (lệnh `a`, mặc định bật: AGC chọn `gain_shift`, `rmin` tự đặt theo mức đo được) và tự nhận FEC theo từng khung. Chi tiết trong `sw/README_SW.md`.

**Kết quả tự động mức (12/11/2026, xô, `test_agc.m`, không gửi lệnh gain hay rmin nào):**

| Biên độ phát | gain_shift app chọn | Khung đúng (FEC) | Bit kênh sửa TB |
|---|---|---|---|
| 0,5 | 1 | 99/99 | 0 |
| 0,9 | 0 | 99/99 | 0 |
| 0,25 | 1 | 99/99 | 0 |
| 0,12 | 2 | 99/99 | 0 |
| 0,05 | 3 | 99/99 | 0 |
| 0,02 | 5 | 99/99 | 0 |
| 0,008 | 6 | 99/99 | 0 |
| 0,005 | 6 | 99/99 | 0,01 |
| 0,003 | 6 | 70/99 | 3,8 |
| 0,9 (nhảy từ gain 6) | 0 | 96/99 | 4,4 |
| 0,05 (nhảy từ gain 0) | 3 | 98/98 | 0 |

- Từ 0,9 xuống 0,005 (45 dB) không mất khung nào và gần như không còn bit kênh sai, kể cả ở các mức mà gain cố định từng bão hòa (A1) hoặc thiếu độ phân giải (A1c).
- Ngưỡng bão hòa đo được khớp hằng số 2048: đỉnh không cắt tới 1712, đỉnh khi bão hòa 2400–3200 với hệ số đỉnh 1,3–2,3 (bình thường 2,9–3,1).
- Bản 3 mất 29 khung ở 0,003 (rmin bị đặt theo luật "im lặng" khi SNR thấp) và 3 khung khi nhảy từ gain 6 về 0,9.

**Đo lại với bản 3.1 (cùng ngày):** 0,5 → 0,02: 99/99 ở mọi mức; 0,008: 98/99; 0,005: 95/99; 0,003: 96/99 (3 khung hỏng do SNR, 49–65 bit sai); nhảy về 0,9: 98/99 (gain 6 → 2 → 0); nhảy về 0,05: 98/98.

- Phần mất còn lại ở 0,008 và 0,005 là do bắt nhầm (1 và 4 lần, CFO lệch, lệch nhịp): mỗi lần bắt nhầm chiếm bộ dò 360 ms và làm mất một khung thật. Bản 3.2 giữ rmin cao hơn ở tín hiệu yếu (chưa đo); sửa tận gốc cần khoảng chờ sau mỗi khung trong `ofdm_sync_sc` (chặng 3).
- Kết luận chặng 1: không cần chỉnh tay trên dải 45 dB (0,9 → 0,005) với ≥ 95 % khung đúng, 100 % từ 0,02 trở lên.

Cấu hình khuyến nghị hiện tại: FEC bật, `gain_shift` 2 khi biên độ phát ≥ 0,35, gain 4 khi biên độ ≤ 0,25, `thr` 180, `rmin` ≈ 200000·(amp/0,9)²·4^(gain−4).
