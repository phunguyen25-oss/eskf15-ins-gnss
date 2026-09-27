# Tài liệu nền đã dùng để tổng hợp

1. Joan Solà, *Quaternion kinematics for the error-state Kalman filter*, 2017: https://arxiv.org/abs/1711.02508
2. N. Trawny và S. I. Roumeliotis, *Indirect Kalman Filter for 3D Attitude Estimation*, University of Minnesota TR-2005-002: https://mediawiki.isr.tecnico.ulisboa.pt/images/d/db/Indirect_Kalman_Filter_for_3D_Attitude_Estimation.pdf
3. C. F. Van Loan, *Computing Integrals Involving the Matrix Exponential*, IEEE TAC, 1978: https://doi.org/10.1109/TAC.1978.1101743
4. W. J. Riley, *Handbook of Frequency Stability Analysis*, NIST SP 1065: https://www.nist.gov/publications/handbook-frequency-stability-analysis
5. Arm, *CMSIS-DSP Matrix Functions*: https://arm-software.github.io/CMSIS-DSP/main/group__groupMatrix.html
6. STMicroelectronics, *STM32F411 product documentation*: https://www.st.com/en/microcontrollers-microprocessors/stm32f411.html
7. TDK InvenSense, *MPU-6050 product/documentation page*: https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/

Các tài liệu trên cung cấp quy ước/quaternion, error-state injection, discretization, noise characterization và giới hạn phần cứng. Thông số noise cụ thể trong code chỉ là giá trị khởi đầu mô phỏng; phải thay bằng kết quả đo của đúng cụm cảm biến.
