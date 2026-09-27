# Khởi tạo và căn chỉnh ban đầu

ESKF không tự sửa được một khởi tạo vô lý. Trước khi bật controller:

1. Giữ hệ đứng yên trong một cửa sổ xác định. Kiểm tra chuẩn gyro, variance gyro và variance accelerometer; không chỉ dùng một timer cố định.
2. Lấy roll/pitch từ hướng gravity sau khi calibration accelerometer. Ở NED, accelerometer đứng yên đo gần `R_nb' * [0,0,-g]`.
3. Lấy yaw từ magnetometer đã hard/soft-iron calibration và tilt compensation, hoặc từ GNSS heading/dual antenna. Nếu không có nguồn yaw, đặt covariance yaw lớn và không tuyên bố yaw đã quan sát được.
4. Khởi tạo `b_g` bằng trung bình gyro đứng yên. Không thể tách hoàn toàn accelerometer bias khỏi sai số tilt chỉ bằng một tư thế; dùng calibration sáu tư thế.
5. Khởi tạo `p/v` từ GPS hợp lệ hoặc mốc ứng dụng. Chuyển lat/lon/alt sang local NED bằng một reference cố định; code mẫu nhận trực tiếp NED.
6. Đặt `P0` theo uncertainty thật, không đặt gần zero để “lọc mượt”. Một state sai nhưng covariance quá nhỏ sẽ chống lại measurement đúng.

Sau khởi tạo, chạy một khoảng estimator-only; chỉ arm controller khi health OK, IMU không timeout, attitude/position variance dưới giới hạn và sensor innovations không bị reject liên tiếp.

