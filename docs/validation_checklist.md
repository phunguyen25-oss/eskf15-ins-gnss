# Checklist nghiệm thu

## Toán học

- [ ] Frame và dấu gravity được kiểm bằng test đứng yên.
- [ ] `F/H` giải tích khớp numerical Jacobian offline trong tolerance.
- [ ] Update có inject và covariance reset.
- [ ] `P` dùng Joseph hoặc square-root method; symmetry được giám sát.
- [ ] Observability được đánh giá theo trajectory/motion excitation thực tế.

## Pha A

- [ ] `run_all_tests` đạt 100%.
- [ ] Monte Carlo dùng nhiều seed và báo confidence interval NIS/NEES.
- [ ] Có case bias, dropout, outlier, lệch timestamp và sai initial attitude.
- [ ] Tiêu chí RMSE được đặt trước khi tuning.

## Pha B

- [ ] Raw log không qua low-pass ẩn trước khi characterization.
- [ ] Timestamp đo và timestamp nhận tách riêng.
- [ ] Accelerometer/magnetometer calibration được xác nhận trên dataset khác.
- [ ] Allan deviation có đủ thời lượng để nhận biết vùng slope.
- [ ] Lever arm và độ trễ được đo, không đoán.

## Pha C

- [ ] Không `malloc/calloc/free` trong đường chạy estimator.
- [ ] ISR không chạy toàn bộ ESKF.
- [ ] WCET < 50% budget ở build release tệ nhất đã đo.
- [ ] Stack high-water mark và RAM còn margin.
- [ ] Mỗi sensor có timeout, gate, reject counter và recovery policy.
- [ ] Periodic Cholesky/LDLT hoặc health check xác nhận `P` hợp lệ.
- [ ] HIL tái hiện log lỗi và lệnh fail-safe đúng.
- [ ] Controller kiểm tra validity flag; không dùng state lỗi một cách im lặng.

## Tiêu chí dừng thử nghiệm

Chuyển ngay sang degraded/manual mode khi có `NaN/Inf`, quaternion norm không thể phục hồi, covariance không positive definite, IMU timeout, estimator deadline miss liên tiếp hoặc uncertainty vượt giới hạn của controller. `trace(P)` chỉ là một chỉ báo; ngưỡng riêng từng state có ý nghĩa vật lý hơn.

