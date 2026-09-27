function report = analyze_static_imu(csv_file,fs)
% CSV columns must include ax,ay,az,gx,gy,gz in SI units.
T=readtable(csv_file);
required={'ax','ay','az','gx','gy','gz'};
assert(all(ismember(required,T.Properties.VariableNames)),'Missing required columns.');
A=T{:,{'ax','ay','az'}}; G=T{:,{'gx','gy','gz'}};
report.acc_mean=mean(A,1); report.acc_cov=cov(A);
report.gyro_mean=mean(G,1); report.gyro_cov=cov(G);
[report.tau,report.acc_adev]=allan_deviation(A,fs);
[~,report.gyro_adev]=allan_deviation(G,fs);
figure('Name','Static IMU characterization');
subplot(2,1,1); loglog(report.tau,report.acc_adev); grid on; ylabel('acc ADEV');
subplot(2,1,2); loglog(report.tau,report.gyro_adev); grid on; ylabel('gyro ADEV'); xlabel('tau (s)');
end

