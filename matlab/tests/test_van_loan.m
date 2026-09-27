function test_van_loan()
c=eskf_default_config(); s=eskf_init(zeros(3,1),zeros(3,1),[1;0;0;0],zeros(3,1),zeros(3,1),eye(15),c);
[F,G,Qc]=eskf_FG(s,[0;0;-c.g],[0.1;0.2;-0.1],eye(3));
[Phi,Qd]=eskf_discretize(F,G,Qc,0.01,true);
assert(all(isfinite(Phi(:)))&&norm(Qd-Qd','fro')<1e-10);
assert(min(eig(Qd))>-1e-10);
end
