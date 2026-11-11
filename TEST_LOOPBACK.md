# Thử nối thẳng: Focusrite Solo → PCM1808 → ZCU104

Mục tiêu: kiểm tra toàn chuỗi điện (MATLAB → DAC Focusrite → ADC PCM1808 → FPGA) khi chưa có kênh nước. SNR rất cao, nên khung nào cũng phải ra CRC OK.

## 1. Đấu nối

### PCM1808 ↔ PMOD0 (J55) của ZCU104 (3.3 V)
Đấu theo cách đánh số J55 của bạn (khớp `xdc/ofdm33_zcu104.xdc`):

| J55 | Chân FPGA | Tín hiệu | PCM1808 |
|---|---|---|---|
| 1 | G8 | `pcm_bck` (6.144 MHz) | BCK |
| 3 | H8 | `pcm_dout` | OUT |
| 5 | G7 | `pcm_lrck` (96 kHz) | LRC |
| 7 | H7 | `pcm_scki` (24.576 MHz) | SCK (MCLK) |

- **MD0, MD1, FMT nối chung xuống GND trên module** (slave, I2S 24 bit). Ba chân này không có trong XDC và không có cổng trong block design.
- GND và 3.3 V lấy từ J55 theo hàng chân nguồn của header, nguồn analog của PCM1808 theo hướng dẫn module. Nối GND của module PCM1808 chung với GND của Focusrite qua cáp tín hiệu.

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

1. Dùng gói mới (đã sửa I2S Receiver: `C_NUM_CHANNELS = 2` (một đường I2S), `C_32BIT_LR = 1`, BCK = 64 fs). Bản cũ cho BCK = 48 fs nên PCM1808 sẽ không chạy.
2. `vivado -mode batch -source tcl/build.tcl -tclargs impl` tạo ra `vivado/ofdm33_rx.xsa`. Trước đó nên mở log kiểm tra các dòng `INFO … skipped` và timing.
3. Tạo Vitis app theo `sw/README_SW.md`, nạp qua JTAG, mở terminal 115200 baud.
4. Đo bằng oscilloscope: SCKI 24.576 MHz, BCK 6.144 MHz, LRCK 96 kHz, DOUT có dữ liệu.

## 3. Trình tự đo (gõ lệnh trên UART)

| Bước | Phía MATLAB (`run_tx.m`) | Lệnh UART | Cần thấy |
|---|---|---|---|
| a | không phát | `n` | rms nhiễu vài LSB, `rmin` được tự đặt |
| b | `S.mode='tone'`, `S.amp=0.3`, `S.tone_s=30` | `l` | ADC trong khoảng −10 … −2 dBFS (chỉnh núm Monitor / `S.amp` / `+` `-`), offset < ±5 Hz (lệch đồng hồ, cỡ chục ppm). MATLAB chuẩn hóa đỉnh OFDM bằng `S.amp` giống tone, nên mức này cũng là mức đỉnh của OFDM |
| c | `S.mode='text'`, `S.repeat=20`, `S.loop=Inf`, giữ nguyên `S.amp` của bước b (tắt tone rồi gõ `n` trước) | `g` | `[OK ] seq 0/1 … "Hello ZCU104! …"`, CFO gần bằng offset của bước b |
| d | (nếu bước c lỗi) đang phát text | `d` (4 gói) | lưu log rồi chạy `read_uart_dump('log.txt')` trong MATLAB để xem phổ, chòm sao và kết quả của bộ thu tham chiếu |

Bộ phát hiện khung chỉ chạy trong lệnh `g` (từ bản app 30/10). Khi dừng `g`, app in số khung phát hiện, số khung demap và số mẫu I2S bị rơi trong lần chạy đó. Lệnh `n`, `l`, `d` mất thêm khoảng 0,8 s để xả gói đang dở, tránh trả về dữ liệu cũ.

## 4. Xử lý sự cố

| Hiện tượng | Kiểm tra |
|---|---|
| `raw capture: DMA timeout` | I2S không có mẫu: SCKI/BCK/LRCK, I2S chưa enable (ctrl bit0), MCLK (`clk_audio` locked) |
| `l` báo offset vài trăm Hz đến kHz | Focusrite không chạy 96 kHz (Windows Sound → Advanced → 24 bit 96000 Hz), hoặc sai `S.fc` |
| Mức ADC không đổi khi vặn Monitor | sai kênh: dùng lệnh `c` (L/R), kiểm tra Line Out 1 → LIN |
| `detected` tăng, `demapped` không tăng (dòng in khi dừng `g`) | kiểm tra chuỗi CORDIC/FFT (ILA) |
| `detected` = 0 | `thr` quá cao (thử 120), `rmin` quá lớn (`r` đặt 100), mức tín hiệu quá nhỏ |
| `[BAD]` liên tục | `d` + `read_uart_dump`. SNR tham chiếu cao mà FPGA vẫn lỗi thì gửi log cho Claude |
