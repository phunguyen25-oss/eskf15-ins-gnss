# Schema log tối thiểu

Ưu tiên binary record cố định trên firmware; chuyển sang CSV khi phân tích. Mỗi record phải có header chung:

| Trường | Kiểu gợi ý | Ý nghĩa |
|---|---:|---|
| `sensor_id` | uint8 | IMU/GPS/BARO/MAG/STATE |
| `sequence` | uint32 | phát hiện mất/gặp lại packet |
| `t_measure_us` | uint64 | thời điểm vật lý mẫu được đo |
| `t_receive_us` | uint64 | thời điểm estimator nhận |
| `status` | uint16 | overflow, saturation, fix type, CRC... |
| `payload` | fixed | raw integer trước, SI/calibrated sau |

IMU: `ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,temp_raw` và bản SI. GPS: NED position/velocity, covariance, fix type, satellite count. Baro: pressure, temperature, altitude và variance. Mag: raw/calibrated xyz. State: nominal state, diagonal `P`, accept/reject flags và health status.

Không dùng timestamp lúc ghi thẻ làm timestamp đo. Không ghi chỉ state đã lọc vì sẽ không thể sửa calibration hay replay thuật toán.
