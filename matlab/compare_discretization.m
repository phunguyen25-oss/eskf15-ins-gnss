function report = compare_discretization()
% Quantify embedded approximation against Van Loan across representative dt.
c=eskf_default_config(); s=eskf_init(zeros(3,1),zeros(3,1),[1;0;0;0],zeros(3,1),zeros(3,1),eye(15),c);
[F,G,Qc]=eskf_FG(s,[1.2;-0.6;-9.5],[0.8;-0.4;0.3],eye(3));
dts=[0.001,0.002,0.005,0.01,0.02]; report=zeros(numel(dts),3);
for k=1:numel(dts)
    [pe,qe]=eskf_discretize(F,G,Qc,dts(k),false);
    [pr,qr]=eskf_discretize(F,G,Qc,dts(k),true);
    report(k,:)=[dts(k),norm(pe-pr,'fro'),norm(qe-qr,'fro')];
end
fprintf(' dt(s)      ||dPhi||F      ||dQ||F\n'); disp(report);
end
