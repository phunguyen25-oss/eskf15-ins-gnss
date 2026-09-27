function [Phi,Qd] = eskf_discretize(F,G,Qc,dt,use_van_loan)
n=size(F,1); L=G*Qc*G';
if use_van_loan
    M=[-F,L; zeros(n),F']*dt;
    E=expm(M);
    Phi=E(n+1:end,n+1:end)';
    Qd=Phi*E(1:n,n+1:end);
else
    Fdt=F*dt;
    Phi=eye(n)+Fdt+0.5*(Fdt*Fdt);
    Qd=L*dt;
end
Qd=0.5*(Qd+Qd');
end

