# Quy ước và phương trình cốt lõi

## 1. Trạng thái

Trạng thái danh định được lưu bằng 16 số:

\[
x = [p_n, v_n, q_{nb}, b_a, b_g]
\]

trong đó `q_nb` quay một vector từ Body sang NED. Sai số tối thiểu có 15 số:

\[
\delta x=[\delta p,\delta v,\delta\theta,\delta b_a,\delta b_g].
\]

Sai số attitude nằm bên phải:

\[
q_{true}=q_{nom}\otimes\operatorname{Exp}(\delta\theta).
\]

## 2. Lan truyền danh định

Với dữ liệu IMU đã đổi sang SI:

\[
a_b=a_m-b_a,\qquad \omega_b=\omega_m-b_g,
\]

\[
a_n=R(q_{nb})a_b+g_n,\qquad g_n=[0,0,g]^T.
\]

Mã dùng orientation giữa bước để giảm sai số tích phân:

\[
p^+=p+v\Delta t+\tfrac12 a_n\Delta t^2,\quad
v^+=v+a_n\Delta t,\quad
q^+=q\otimes\operatorname{Exp}(\omega_b\Delta t).
\]

Vị trí bắt buộc dùng vận tốc **trước** cập nhật. Dùng `v_new` trong công thức trên tạo hệ số gia tốc sai thành `1.5*a*dt²`.

## 3. Động học sai số liên tục

Với thứ tự block `[dp,dv,dtheta,dba,dbg]`:

\[
F=
\begin{bmatrix}
0&I&0&0&0\\
0&0&-R[a_b]_\times&-R&0\\
0&0&-[\omega_b]_\times&0&-I\\
0&0&0&0&0\\
0&0&0&0&0
\end{bmatrix}.
\]

MATLAB có chế độ Van Loan để tính `Phi,Qd` làm tham chiếu (`cfg.use_van_loan=true`). Demo/Monte Carlo mặc định dùng cùng xấp xỉ nhanh như bản nhúng: `Phi ≈ I + Fdt + 0.5F²dt²`, `Qd ≈ GQcG' dt`. Phải so sánh hai chế độ bằng log ở đúng tần số chạy thật.

## 4. Update, inject và reset

\[
y=z-h(x),\; S=HPH^T+R,\; K=PH^TS^{-1},\;\delta x=Ky.
\]

Update covariance dùng Joseph:

\[
P^+=(I-KH)P^-(I-KH)^T+KRK^T.
\]

Sau đó inject `dp,dv,dba,dbg` bằng cộng và attitude bằng quaternion. Vì tọa độ sai số attitude thay đổi sau inject, covariance được reset với block:

\[
G_{reset,\theta}\approx I-\tfrac12[\delta\theta]_\times,
\qquad P\leftarrow G_{reset}PG_{reset}^T.
\]

Quaternion được chuẩn hóa sau predict và inject; chuẩn hóa không thay thế việc kiểm tra `NaN`, chuẩn gần zero hoặc bước thời gian hỏng.

## 5. Gating và tính nhất quán

Mẫu đo m chiều có NIS:

\[
\epsilon_\nu=y^TS^{-1}y.
\]

Ngưỡng phải lấy từ phân phối chi-square với đúng số bậc tự do. Lõi nhúng dùng scalar sequential update nên mỗi phép gate có một bậc tự do; các ngưỡng mặc định 3.84 (95%) hoặc 6.63 (99%). Khi dùng gate chung cho vector 3D, không được dùng lại ngưỡng 1D.

NEES chỉ hợp lệ khi có ground truth và được tính từng thời điểm bằng `e'*(P\e)`, không dùng `inv(P)`. NIS dùng được với log thật nhưng chỉ đánh giá phần không gian đo.
