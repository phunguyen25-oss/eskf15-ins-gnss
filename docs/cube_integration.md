# Ghép vào STM32CubeIDE

## 1. File cần thêm

Thêm toàn bộ `stm32/Inc/*.h` và `stm32/Src/*.c` vào project. Bật FPU cho Cortex-M4F, dùng `-std=c11` hoặc `gnu11`, và link `libm` nếu toolchain yêu cầu.

`eskf15.c` hoàn toàn không biết HAL. `mpu6050_adapter.c` chỉ chuyển đổi raw-to-SI. Bạn vẫn phải cung cấp driver I2C/SPI/DMA và calibration lấy từ Pha B.

## 2. Scheduling đề nghị

- Timer hoặc DRDY khởi tạo giao dịch DMA và chốt thời điểm lấy mẫu.
- Callback DMA đưa mẫu vào ring buffer cố định, không gọi ESKF nặng trong ISR.
- Task estimator lấy từng mẫu theo thứ tự timestamp và gọi `app_eskf_push_imu`.
- GPS/baro/mag được đẩy bằng event riêng. `app_eskf` giữ history cố định, tua về snapshot gần nhất không muộn hơn timestamp đo, update rồi repropagate chuỗi IMU. Mẫu cũ hơn buffer trả về `ESKF_LATE_MEASUREMENT`.

Xem `stm32/integration/cubemx_user_code_snippets.c`; chỉ sao chép phần tương ứng vào các vùng `USER CODE` để CubeMX không ghi đè.

## 3. Bộ nhớ và thời gian

Ma trận covariance 15×15 float chiếm 900 byte. Hai scratch matrix nằm trong `eskf15_t` để tránh heap/stack biến động. Mặc định `ESKF_HISTORY_LEN=64`; mỗi snapshot giữ nominal state, `P` và IMU nên dùng khoảng 62 KiB. Ở 500 Hz, 64 mẫu chỉ bao phủ xấp xỉ 128 ms. Có thể giảm/tăng macro theo RAM và độ trễ đã đo; kiểm tra map file và stack high-water mark. Không suy đoán rằng mọi biến thể F411 đều có cùng RAM/clock/ngoại vi.

## 4. Trình tự xác nhận

1. Build native test với GCC.
2. Chạy target không cảm biến bằng vector vàng.
3. Đọc MPU6050, chỉ log raw và SI; xác nhận axis/sign.
4. Bật predict, giữ update tắt; kiểm tra stationary behavior.
5. Bật từng sensor update một.
6. Replay/HIL với dropout và outlier.
7. Đo WCET bằng DWT cycle counter trên build Release.
