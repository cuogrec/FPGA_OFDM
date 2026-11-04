# Thử nối thẳng: Focusrite Solo → PCM1808 → ZCU104

Mục tiêu: kiểm tra toàn chuỗi điện (MATLAB → DAC Focusrite → ADC PCM1808 → FPGA) khi chưa có kênh nước. SNR rất cao, nên khung nào cũng phải ra CRC OK.

## 1. Đấu nối

### PCM1808 ↔ PMOD0 (J55) của ZCU104 (3.3 V)
| J55 | Tín hiệu FPGA | PCM1808 |
|---|---|---|
| 1 | `pcm_scki` (24.576 MHz) | SCKI |
| 2 | `pcm_bck` (6.144 MHz) | BCK |
| 3 | `pcm_lrck` (96 kHz) | LRCK |
| 4 | `pcm_dout` | DOUT |
| 5 / 11 | GND | DGND + AGND |
| 6 / 12 | 3.3 V | VDD (nếu module không tự có ổn áp 3.3 V) |

- **MD0, MD1, FMT: nối thẳng xuống GND trên module.** PCM1808 chốt chế độ lúc cấp nguồn. Lúc FPGA chưa nạp bitstream, chân PMOD có thể đang ở mức cao (có pull-up), làm PCM1808 khởi động ở chế độ master. Chân `pcm_md0/md1/fmt` của FPGA chỉ để dự phòng, bỏ trống cũng được.
- VCC (analog) của PCM1808 cấp **5 V** theo module (PMOD chỉ có 3.3 V). Nếu đổi chân MD/FMT thì phải tắt nguồn rồi bật lại PCM1808.

### Focusrite → PCM1808 (ngõ vào LIN)
- Dùng **Line Output 1 (Left)**, jack 6.35 mm TRS. Đầu Tip là tín hiệu, Sleeve nối GND. Có thể dùng cáp TS (mono).
- Ở 0 dBFS, line out của Solo Gen 3 lên tới +15.5 dBu (khoảng 13 Vpp cân bằng, khoảng 6.5 Vpp khi dùng một đầu). Full-scale của PCM1808 chỉ khoảng 3 Vpp (VCC 5 V). **Bắt buộc có bộ chia áp:**

```
Focusrite Tip ──[ 10 kΩ ]──┬──[ 1 µF ]──► LIN (PCM1808)
                           │
                        [ 2.2 kΩ ]
                           │
Focusrite Sleeve ──────────┴───────────► AGND
```

  Hệ số chia khoảng 0.18, nên 6.5 Vpp còn khoảng 1.2 Vpp. Tụ 1 µF có thể bỏ nếu module đã có tụ vào.
- Vặn núm Monitor của Solo về khoảng 9 giờ trước khi phát. Bật oscilloscope ở LIN để xem: đỉnh–đỉnh không được vượt 3 Vpp.

## 2. FPGA

1. Dùng gói mới (đã sửa I2S Receiver: `C_NUM_CHANNELS = 1`, `C_32BIT_LR = 1`, BCK = 64 fs). Bản cũ cho BCK = 48 fs nên PCM1808 sẽ không chạy.
2. `vivado -mode batch -source tcl/build.tcl -tclargs impl` tạo ra `vivado/ofdm33_rx.xsa`. Trước đó nên mở log kiểm tra các dòng `INFO … skipped` và timing.
3. Tạo Vitis app theo `sw/README_SW.md`, nạp qua JTAG, mở terminal 115200 baud.
4. Đo bằng oscilloscope: SCKI 24.576 MHz, BCK 6.144 MHz, LRCK 96 kHz, DOUT có dữ liệu.

## 3. Trình tự đo (gõ lệnh trên UART)

| Bước | Phía MATLAB (`run_tx.m`) | Lệnh UART | Cần thấy |
|---|---|---|---|
| a | không phát | `n` | rms nhiễu vài LSB, `rmin` được tự đặt |
| b | `S.mode='tone'`, `S.amp=0.3`, `S.tone_s=30` | `l` | ADC khoảng −12 dBFS (chỉnh núm Monitor / `S.amp` / `+` `-`), offset < ±5 Hz (lệch đồng hồ, cỡ chục ppm) |
| c | `S.mode='text'`, `S.repeat=20`, `S.loop=Inf` | `g` | `[OK ] seq 0/1 … "Hello ZCU104! …"`, CFO gần bằng offset của bước b |
| d | (nếu bước c lỗi) đang phát text | `d` (4 gói) | lưu log rồi chạy `read_uart_dump('log.txt')` trong MATLAB để xem phổ, chòm sao và kết quả của bộ thu tham chiếu |

`s` hiện bộ đếm khung sync và demap, số mẫu I2S bị rơi, CFO.

## 4. Xử lý sự cố

| Hiện tượng | Kiểm tra |
|---|---|
| `raw capture: DMA timeout` | I2S không có mẫu: SCKI/BCK/LRCK, I2S chưa enable (ctrl bit0), MCLK (`clk_audio` locked) |
| `l` báo offset vài trăm Hz đến kHz | Focusrite không chạy 96 kHz (Windows Sound → Advanced → 24 bit 96000 Hz), hoặc sai `S.fc` |
| Mức ADC không đổi khi vặn Monitor | sai kênh: dùng lệnh `c` (L/R), kiểm tra Line Out 1 → LIN |
| sync frames tăng, demap không tăng | kiểm tra chuỗi CORDIC/FFT (ILA) |
| sync frames không tăng | `thr` quá cao (thử 120), `rmin` quá lớn (`r` đặt 100), mức tín hiệu quá nhỏ |
| `[BAD]` liên tục | `d` + `read_uart_dump`. SNR tham chiếu cao mà FPGA vẫn lỗi thì gửi log cho Claude |
