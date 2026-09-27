function out = run_phase_a_demo(do_plot,seed,use_van_loan)
if nargin<1, do_plot=true; end
if nargin<2, seed=11; end
if nargin<3, use_van_loan=false; end
dt=0.01; tr=generate_truth(40,dt); d=simulate_sensors(tr,dt,seed);
c=eskf_default_config(); c.use_van_loan=use_van_loan;
q0=quat_mul(tr.q(:,1),quat_exp(deg2rad([3;-2;5])));
P0=diag([ones(1,3),ones(1,3),repmat(deg2rad(8)^2,1,3), ...
         repmat(0.1^2,1,3),repmat(deg2rad(1)^2,1,3)]);
s=eskf_init(d.gps_p(:,1),d.gps_v(:,1),q0,zeros(3,1),zeros(3,1),P0,c);
n=numel(tr.t); est=zeros(16,n); nees=nan(1,n); nis=[];
for k=1:n
    if k>1, s=eskf_predict(s,d.acc(:,k-1),d.gyro(:,k-1),dt); end
    if d.gps_mask(k)
        [s,ii]=eskf_update_gps(s,d.gps_p(:,k),d.gps_v(:,k),d.Rp,d.Rv,zeros(3,1));
        nis=[nis;ii.nis(:)]; %#ok<AGROW>
    end
    if d.baro_mask(k)
        [s,ii]=eskf_update_baro(s,d.baro(k),d.Rbaro); nis=[nis;ii.nis(:)]; %#ok<AGROW>
    end
    if d.mag_mask(k)
        [s,ii]=eskf_update_mag(s,d.mag(:,k),d.mag_n,d.Rmag); nis=[nis;ii.nis(:)]; %#ok<AGROW>
    end
    est(:,k)=[s.p;s.v;s.q;s.ba;s.bg];
    e=state_error(s,tr,k); nees(k)=e'*(s.P\e);
end
pos_err=est(1:3,:)-tr.p; vel_err=est(4:6,:)-tr.v;
out.rmse_position=sqrt(mean(sum(pos_err.^2,1)));
out.rmse_velocity=sqrt(mean(sum(vel_err.^2,1)));
out.mean_nees=mean(nees(isfinite(nees)));
out.nis_accept_rate=mean(nis<=c.gate_1d);
out.accepted=s.accepted; out.rejected=s.rejected;
fprintf('3D position RMSE: %.3f m\n',out.rmse_position);
fprintf('3D velocity RMSE: %.3f m/s\n',out.rmse_velocity);
fprintf('Mean NEES (diagnostic): %.2f; scalar gate pass: %.1f %%\n', ...
    out.mean_nees,100*out.nis_accept_rate);
fprintf('Accepted/rejected scalar measurements: %d/%d\n',s.accepted,s.rejected);
if do_plot
    figure('Name','ESKF Phase A');
    subplot(2,1,1); plot(tr.t,pos_err'); grid on; ylabel('position error (m)'); legend('N','E','D');
    subplot(2,1,2); plot(tr.t,vel_err'); grid on; ylabel('velocity error (m/s)'); xlabel('time (s)');
end
end
