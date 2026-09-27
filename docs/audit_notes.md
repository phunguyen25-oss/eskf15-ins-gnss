# Những điểm đã sửa so với lộ trình nháp

Gói này giữ cấu trúc tốt của lộ trình bảy giai đoạn nhưng sửa các chỗ dễ gây lỗi khi triển khai:

- Tách thành ba cổng A/B/C để không đưa một mô hình chưa được chứng minh thẳng xuống MCU.
- Observability được phân tích sau khi có `F/H` và theo trajectory; không kết luận tuyệt đối từ một ma trận LTI chung chung.
- Bổ sung đầy đủ covariance predict, measurement update, inject và reset — bốn phần cốt lõi thường bị lộ trình mô tả bỏ qua.
- Sửa thứ tự tích phân vị trí để không sinh hệ số `1.5*a*dt²`.
- Phân biệt numerical differentiation dùng làm test offline với numerical differentiation trong runtime.
- Không bắt buộc RK4 một cách máy móc. Midpoint quaternion + kiểm soát dt có thể phù hợp hơn trên MCU; lựa chọn integrator phải dựa vào error budget/WCET.
- Phân biệt sample variance, noise density, bias instability và bias random walk; Allan deviation không biến mọi cực tiểu thành một tham số duy nhất.
- NIS/NEES dùng phép giải tuyến tính, không dùng `inv`; NEES chỉ dùng khi có ground truth.
- Bổ sung initialization, lever arm, timestamp đo/nhận, out-of-sequence measurement và history repropagation.
- Symmetrize/diagonal floor chỉ là guard số học, không phải chứng minh positive definiteness; thêm Cholesky health check.
- Health không chỉ dựa vào `trace(P)` mà kiểm tra uncertainty theo nhóm state, finite, timeout, reject và deadline.

