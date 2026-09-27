function c = eskf_default_config()
c.g = 9.80665;
% Continuous-time noise densities / random walks (SI/sqrt(Hz)).
c.sigma_acc = 0.08;
c.sigma_gyro = deg2rad(0.35);
c.sigma_ba = 0.002;
c.sigma_bg = deg2rad(0.015);
c.gate_1d = 6.63;             % chi-square, 1 DoF, 99 percent
c.min_variance = 1e-10;
c.max_dt = 0.05;
c.use_van_loan = false;        % fast second-order Phi; enable for offline reference
end
