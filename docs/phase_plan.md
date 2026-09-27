# Lộ trình ba pha và cổng nghiệm thu

## Pha A — Mô hình, mô phỏng, kiểm thử

1. Khóa quy ước frame, quaternion, dấu gravity, đơn vị và timestamp.
2. Xác định trạng thái tối thiểu theo cảm biến thật và nhu cầu điều khiển.
3. Phân tích observability trên mô hình đã tuyến tính hóa dọc theo trajectory; không dùng một ma trận quan sát LTI đơn lẻ để kết luận cho mọi chuyển động phi tuyến.
4. Suy ra `F`, `G`, từng `H`; kiểm tra Jacobian giải tích bằng sai phân hữu hạn **offline**. Cấm sai phân hữu hạn trong vòng lặp nhúng, nhưng dùng nó làm oracle kiểm thử là đúng.
5. Mô phỏng đa tốc độ, bias, dropout, outlier và latency.
6. Chạy unit test, Monte Carlo, NIS/NEES; phân biệt lỗi mô hình với tuning.

**Cổng A:** mọi unit test đạt; quaternion norm ổn định; covariance đối xứng/positive definite trong test; RMSE và NIS nằm trong tiêu chí đã ghi trước.

## Pha B — Cảm biến thật và replay

1. Đo noise density, bias instability và temperature drift; không lấy phương sai mẫu tĩnh rồi gán trực tiếp cho mọi `R`/`Q`.
2. Hiệu chuẩn accelerometer sáu tư thế; gyro bias/scale; magnetometer hard/soft iron; kiểm tra residual sau calibration.
3. Ghi raw integer, dữ liệu SI, timestamp đo, timestamp nhận, sequence number, trạng thái driver và ground truth nếu có.
4. Replay chính cùng một thứ tự sự kiện như firmware; thực hiện fault injection có kiểm soát.
5. Tinh chỉnh bằng NIS/innovation whiteness; NEES chỉ khi có ground truth độc lập.

**Cổng B:** calibration residual đạt; không mất timestamp; replay xác định được; không divergence khi dropout/outlier đã định nghĩa; latency và lever-arm được mô hình hóa.

## Pha C — Nhúng, HIL và an toàn

1. Cross-compile lõi C với warning tối đa; chạy native test và so với vector vàng MATLAB.
2. Đo WCET, stack, RAM, jitter và deadline ở tần số IMU thực.
3. Tách acquisition khỏi estimation; DMA/ISR chỉ chốt timestamp và buffer, phép toán ma trận chạy trong task có deadline.
4. Bù delay bằng buffer state/IMU và repropagation; không update một GPS cũ vào trạng thái hiện tại.
5. Kiểm tra brownout, sensor reset, packet corruption, `NaN/Inf`, covariance hỏng, reject liên tiếp và timeout.
6. HIL trước bench test có actuator, sau đó mới test có ràng buộc và vùng an toàn.

**Cổng C:** không deadline miss; kết quả C bám vector vàng; health monitor chuyển mode đúng; reset/recovery xác định; fail-safe độc lập với EKF.

