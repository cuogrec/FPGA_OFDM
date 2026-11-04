# Ứng dụng PS (Vitis 2024.2, bare-metal)

`ofdm33_app.c` chỉ dùng truy cập thanh ghi trực tiếp (`Xil_In32/Out32`), không cần driver ID. Vì vậy chạy được cả với Vitis cũ lẫn Vitis Unified (SDT).

## Tạo project (Vitis Unified IDE 2024.2)
1. **File → New Component → Platform**: chọn `vivado/ofdm33_rx.xsa`, OS `standalone`, processor `psu_cortexa53_0`, giữ tick *Generate boot artifacts*. Bấm **Build**.
2. **File → New Component → Application**: tên `ofdm33_app`, chọn platform vừa tạo. Copy `ofdm33_app.c` vào thư mục `src/` và xóa file mẫu nếu có.
3. Link thư viện toán: mở `src/UserConfig.cmake`, thêm `m` vào `USER_LINK_LIBRARIES` (hoặc dùng mục *Libraries* trong Settings). Bấm Build.
   - Nếu `printf` không in được số thực, thêm linker flag `-u _printf_float`.
4. ZCU104: gạt **SW6 về JTAG** (cả 4 công tắc ON). Cắm micro-USB J164 (JTAG + UART), cấp nguồn.
5. **Run → Run (Launch Hardware)**: Vitis tự chạy psu_init, nạp bitstream và chạy app.
6. Mở terminal 115200 8N1 trên cổng COM UART0 của ZCU104. FT4232 tạo nhiều cổng COM, thường là cổng UART thứ nhất.

Lúc khởi động app in ra địa chỉ các khối và thanh ghi I2S. Kiểm tra `ctrl` có **bit3 = 1** (32-bit LRCLK) và `tim 2`.

## Lệnh
| Phím | Chức năng |
|---|---|
| `h` / `s` | trợ giúp / trạng thái (khung sync, demap, I2S drop, CFO, thiết lập) |
| `n` | đo nhiễu (không phát gì) và tự đặt `rmin` = 8 × R_noise |
| `l` | đo mức khi phát tone 33 kHz: mức ADC (dBFS, ±3 dB), lệch tần số (Hz, ppm) |
| `d` | bắt baseband thô N × 4096 mẫu, in `RAW_BEGIN…RAW_END` (MATLAB: `read_uart_dump.m`), hoặc dùng XSCT `mrd -bin` |
| `g` | chạy bộ thu: in từng khung `[OK]/[BAD]`, ghép văn bản nhiều khung; bấm phím bất kỳ để dừng |
| `+` `-` | gain_shift |
| `t` `o` `b` `r` `c` | ngưỡng thr, toffs, bb_shift, rmin, kênh ADC L/R |

## Khởi tạo I2S Receiver (PG308)
`0x08 = 0` → `0x30 = 1` (kênh 0/1) → `0x20 = 2` (SCLK = MCLK / (2·2) = 6.144 MHz = 64 fs) → `0x08 = 1`.
