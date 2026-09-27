function stats = run_monte_carlo(runs)
if nargin<1, runs=20; end
stats=zeros(runs,4);
for k=1:runs
    o=run_phase_a_demo(false,k,false);
    stats(k,:)=[o.rmse_position,o.rmse_velocity,o.mean_nees,o.nis_accept_rate];
end
fprintf('Mean [pos_RMSE vel_RMSE NEES NIS_pass] =\n'); disp(mean(stats,1));
end
