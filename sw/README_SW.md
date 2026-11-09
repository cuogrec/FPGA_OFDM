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
| `h` / `s` | trợ giúp / trạng thái (bộ đếm khung, I2S drop, CFO, thiết lập) |
| `n` | đo nhiễu (không phát gì) và tự đặt `rmin` = 8 × R_noise. `+`/`-` tự nhân/chia `rmin` cho 4 |
| `l` | đo mức khi phát tone 33 kHz: mức ADC (dBFS, ±3 dB), lệch tần số (Hz, ppm). Mức tone = mức đỉnh OFDM với cùng `S.amp`; nên ở −10 … −2 dBFS |
| `d` | bắt baseband thô N × 4096 mẫu, in `RAW_BEGIN…RAW_END` (MATLAB: `read_uart_dump.m`), hoặc dùng XSCT `mrd -bin` |
| `g` | chạy bộ thu: xả khung cũ trong FIFO, bật bộ phát hiện, in từng khung `[OK]/[BAD]`, ghép văn bản; phím bất kỳ để dừng (in bộ đếm của lần chạy) |
| `+` `-` | gain_shift |
| `t` `o` `b` `r` `c` | ngưỡng thr, toffs, bb_shift, rmin, kênh ADC L/R |

## Khởi tạo I2S Receiver (PG308)
`0x08 = 0` → `0x30 = 1` (kênh 0/1) → `0x20 = 2` (SCLK = MCLK / (2·2) = 6.144 MHz = 64 fs) → `0x08 = 1`.

## Thay đổi bản 30/10/2026
- `n`/`l`/`d` không còn trả về gói cũ của lần đo trước (xả gói `axis_tap_pkt` đang dở, thêm khoảng 0,8 s mỗi lệnh).
- Bộ phát hiện khung (`sync_en`) chỉ bật trong `g`; đầu lệnh `g` xả FIFO bit nên không in khung cũ và vòng đệm đồng bộ không bị tràn khi ở menu.
- `rmin` tự đổi theo `gain_shift`, kẹp trong 32 bit. Khung có độ dài DMA sai được báo `[BAD]`.

## Đo đạc (bản 08/11/2026)

- Lệnh `k`: bật/tắt chế độ khung thử. MATLAB phát `run_tx.m` với `S.mode = 'test'` (nội dung biết trước, xem `matlab/ofdm_test_payloads.m`). Khi bật, `g` in mỗi khung một dòng `F,t_ms,seq,crc_ok,cfo_hz,bit_err,bytes` và khi dừng in tổng số bit lỗi / BER.
- Ở chế độ thường, dòng `[OK ]`/`[BAD]` có thêm thời điểm `t ... ms`.
- `matlab/rx_log.m` điều khiển app qua COM6 thay cho Tera Term và lưu CSV/log vào `data/`:
  `rx_log('k')`, `rx_log('g','N',200,'tag','2m','dist_m',2)`, `rx_log('d','npkt',16,'tag','2m')`, `rx_log('m','secs',120)`.
- `matlab/ofdm_channel_report.m` phân tích file dump: đáp ứng tần số, SNR theo sóng mang, trễ đa đường so với CP, độ biến đổi kênh.

## FEC (bản 09/11/2026)

- Lệnh `e`: bật/tắt giải mã FEC. MATLAB phát với `S.fec = true` trong `run_tx.m` (chế độ `text` hoặc `test`).
- Mã chập K = 7, tỉ lệ 1/2 (133/171 bát phân), xen kẽ khối 16 × 112, giải Viterbi quyết định cứng trên Cortex-A53. Mỗi khung mang 111 byte tin (106 byte văn bản).
- Ở chế độ khung thử, dòng `F` có thêm cột `raw_err` = số bit kênh đã được sửa; khi dừng in thêm BER kênh trước FEC.
- MATLAB: `ofdm_fec_frame.m`, `ofdm_fec_unframe.m`, `ofdm_fec_payloads.m`; `ofdm_channel_report.m` tự nhận khung FEC trong file dump.
