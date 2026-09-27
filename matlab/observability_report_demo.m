function report = observability_report_demo()
% Finite-horizon local observability along the simulated trajectory.
dt=0.01; tr=generate_truth(20,dt); c=eskf_default_config();
s=eskf_init(tr.p(:,1),tr.v(:,1),tr.q(:,1),tr.ba(:,1),tr.bg(:,1),eye(15),c);
T=eye(15); W=zeros(15); mn=[0.42;0.08;0.90]; mn=mn/norm(mn);
for k=1:numel(tr.t)
    s.q=tr.q(:,k); s.ba=tr.ba(:,k); s.bg=tr.bg(:,k);
    [F,G,Qc]=eskf_FG(s,tr.f(:,k),tr.omega(:,k),quat_to_rot(s.q)); %#ok<ASGLU>
    [Phi,~]=eskf_discretize(F,G,Qc,dt,false); T=Phi*T;
    H=[];
    if mod(k-1,20)==0
        Hg=zeros(6,15); Hg(1:3,1:3)=eye(3); Hg(4:6,4:6)=eye(3); H=[H;Hg]; %#ok<AGROW>
    end
    if mod(k-1,5)==0
        Hb=zeros(1,15); Hb(3)=-1;
        pred=quat_to_rot(s.q)'*mn; Hm=zeros(3,15); Hm(:,7:9)=skew3(pred);
        H=[H;Hb;Hm]; %#ok<AGROW>
    end
    if ~isempty(H), W=W+T'*(H'*H)*T; end
end
sv=svd(0.5*(W+W')); tol=max(sv)*1e-8;
report.singular_values=sv; report.numerical_rank=sum(sv>tol);
report.condition=max(sv)/max(min(sv),eps);
fprintf('Finite-horizon local observability rank: %d/15\n',report.numerical_rank);
fprintf('Condition indicator: %.3e (large means weak directions)\n',report.condition);
semilogy(sv,'o-'); grid on; xlabel('mode'); ylabel('Gramian singular value');
title('Trajectory-dependent local observability');
end

