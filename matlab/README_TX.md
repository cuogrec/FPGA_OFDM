# Bộ phát MATLAB – OFDM 33 kHz

Mở `run_tx.m`, sửa khối **THIẾT LẬP**, bấm Run. Không cần toolbox nào (Audio Toolbox là tùy chọn).

| Thiết lập | Ý nghĩa |
|---|---|
| `S.mode` | `'text'` gửi văn bản · `'tone'` sin 33 kHz (chỉnh mức, kiểm tra transducer) · `'sweep'` quét 25→41 kHz (đo đáp ứng) |
| `S.text` | văn bản (UTF-8, có dấu được). 219 byte/khung, tự chia nhiều khung |
| `S.repeat`, `S.gap_ms` | số lần lặp, khoảng lặng giữa khung (để vang trong xô tắt) |
| `S.amp` | biên độ đỉnh 0…1 |
| `S.papr_clip_db` | cắt đỉnh baseband (ví dụ 7) để tăng công suất trung bình; `Inf` = tắt |
| `S.play`, `S.loop` | phát qua Focusrite, số lần phát (`Inf` = liên tục, Ctrl+C dừng) |
| `S.device`, `S.driver`, `S.out_ch` | tên thiết bị (`'Focusrite'`), `'ASIO'` nếu dùng Audio Toolbox, Output 1/2 |
| `S.selftest` | trước khi phát, tự giải lại qua kênh xô nước giả lập (4 tia, nhiễu) và in số bit lỗi |

Mỗi lần chạy luôn ghi `ofdm_tx.wav` (96 kHz, 24 bit) để phát lại hoặc đưa vào mô phỏng.

## Focusrite Solo Gen 3 – lưu ý
- Đặt **96 kHz** trong Focusrite Control và Windows Sound → Focusrite USB → Advanced (24 bit, 96000 Hz). Nếu thiết bị chạy 44.1/48 kHz, sóng 33 kHz vượt Nyquist và sẽ **mất hẳn**.
- `audioplayer` (MATLAB cơ bản) dùng driver mặc định; với Audio Toolbox đặt `S.driver = 'ASIO'` để phát bit-perfect.
- Ngõ ra line của Solo chỉ vài Vpp: transducer BII-7511H thường cần thêm mạch khuếch đại công suất. Bắt đầu với `S.mode='tone'` để chỉnh mức trên ADC PCM1808 (đỉnh khoảng −6 dBFS, dùng `gain_shift`).

## Khung (khớp FPGA – không sửa `ofdm_params.m` nếu không sửa RTL)
`[CP|S&C][CP|LTF][CP|DATA]×16`, 360 ms, 224 byte: `seq | số khung | độ dài | text(219) | CRC-16/CCITT`.
Bit LSB trước, làm trắng bằng PN9 (seed 0x155) → QPSK Gray trên 56 subcarrier, 8 pilot.
FPGA trả về bit **chưa bỏ làm trắng**: dùng `ofdm_decode_dma('bits.bin')` (hoặc làm lại trong code PS).

## Các file
| File | Chức năng |
|---|---|
| `run_tx.m` | script chính |
| `rx_decode_wav.m` | giải WAV / bản ghi / dump `dma_raw` bằng bộ thu tham chiếu, vẽ metric + chòm sao |
| `ofdm_params.m` | tham số cố định (khớp RTL) |
| `ofdm_text_to_payloads.m`, `crc16_ccitt.m`, `ofdm_payload_to_bits.m` | văn bản → khung 224 byte → bit |
| `ofdm_build_frame.m`, `ofdm_upconvert.m`, `ofdm_tx_signal.m` | bit → OFDM baseband 8 kHz → sóng mang 33 kHz @ 96 kHz |
| `ofdm_play.m`, `ofdm_tone.m` | phát qua Focusrite, tín hiệu tone/sweep |
| `ofdm_rx_ref.m` | bộ thu tham chiếu (cùng thuật toán với FPGA) |
| `ofdm_decode_dma.m`, `ofdm_parse_payload.m`, `ofdm_join_text.m` | giải kết quả `dma_bits` của FPGA, ghép văn bản |
