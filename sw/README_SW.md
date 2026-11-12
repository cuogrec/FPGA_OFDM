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

## Tự động mức và tự nhận FEC (bản 3, 12/11/2026)

Mặc định sau khi nạp: `gain_shift 2`, `thr 180`, `fec 1`, `agc 1`. Dòng chào in `rev 3` kèm ngày giờ build.

- Lệnh `a`: bật/tắt tự động mức. Khi bật, trong lúc chạy `g` app lấy liên tục các gói baseband thô 4096 mẫu (0,512 s) và từ mỗi gói:
  - **AGC**: có từ 8 mẫu sát mức bão hòa (|bb| ≥ 1800) thì giảm `gain_shift` 2 bậc ngay; có từ 8 mẫu trên 0,40 toàn thang (820), hoặc hệ số đỉnh của khung dưới 2,2, trong 2 gói liên tiếp thì giảm 1 bậc; có khung mà đỉnh dưới 0,15 toàn thang (307) trong 2 gói thì tăng 1 hoặc 2 bậc (tối đa 6). Khi im lặng thì giữ nguyên. Thay đổi được áp dụng ngay sau khi nhận xong một khung (trong khoảng nghỉ).
  - **rmin**: Rn = cửa sổ 128 mẫu yên nhất, Rs = cửa sổ ở phân vị 75. Có khung: `rmin = max(√(Rn·Rs), Rs/32, min(16·Rn, Rs/6))`; im lặng: `rmin = 6·Rs`; không bao giờ dưới 1500.
- Gõ tay `+`, `-`, `r` hoặc `n` thì tự động mức tắt (script cũ `test_a1.m`, `test_a1b.m`, `test_s2.m` vì thế chạy như trước). Gõ `a` để bật lại.
- **Tự nhận FEC**: mỗi khung được thử CRC theo kiểu `e` đang chọn trước, sai thì thử kiểu còn lại. Quên gõ `e` không còn làm hỏng khung; `e` chỉ quyết định kiểu thử trước.
- Dòng bắt đầu bằng `#` là thông tin (`# AGC gain_shift 4 to 2 ...`, `# rmin ...`, `# CLIP ...`); `rx_log.m` bỏ qua các dòng này.
- Khi dừng `g` có thêm dòng `level: last peak ..., max ... of 2048, crest ... | gain_shift ..., rmin ..., auto on, N gain change(s)`.
- Khi tự động mức tắt mà tín hiệu bão hòa, app in `# CLIP: input saturates ...` (tối đa 5 giây một lần).
- Kiểm bằng `matlab/test_agc.m`: quét biên độ 0,9 → 0,003 rồi nhảy lại 0,9 mà không gửi lệnh gain hay rmin nào.

### Bản 3.1 (12/11/2026, sau lần đo `test_agc.m` đầu tiên)

- Ngưỡng coi là có khung hạ từ Rs ≥ 4·Rn xuống Rs ≥ 2·Rn; luật "im lặng" chỉ được dùng khi đã 2 giây không có khung đúng. Trước đó ở biên độ 0,003 rmin bị nhảy lên mức im lặng và làm mất khung.
- rmin tăng khi mục tiêu lớn gấp 2, giảm ngay khi mục tiêu nhỏ hơn 1,3 lần.
- Bão hòa nặng (hệ số đỉnh dưới 1,6) thì hạ 4 bậc một lần thay vì 2.
- Dòng chào in `rev 3.1`.

### Bản 3.2 (12/11/2026, sau lần đo thứ hai)

- Ở tín hiệu yếu, rmin được giữ cao hơn: thành phần thứ ba đổi từ `min(4·Rn, Rs/2)` thành `min(16·Rn, Rs/6)`. Lần đo bản 3.1 có 4 khung bắt nhầm ở biên độ 0,005 khi rmin tụt xuống cỡ Rs/8 đến Rs/16; tín hiệu mạnh không đổi (vẫn Rs/32).
- Chưa đo trên board. Bắt nhầm ở mức rất yếu cần sửa tận gốc trong `ofdm_sync_sc` (khoảng chờ sau mỗi khung), thuộc chặng 3.
