function [s,info] = eskf_update_sequential(s,residual,H,Rdiag,gate)
% Sequential scalar update. R must be diagonal/uncorrelated.
if nargin<5, gate=s.cfg.gate_1d; end
m=numel(residual); dx=zeros(15,1);
accepted=false(m,1); nis=nan(m,1);
for k=1:m
    h=H(k,:); r=Rdiag(k);
    innov=residual(k)-h*dx;
    PHt=s.P*h'; S=h*PHt+r;
    nis(k)=innov*innov/S;
    if isfinite(S) && S>0 && nis(k)<=gate
        K=PHt/S;
        A=eye(15)-K*h;
        s.P=A*s.P*A'+(K*r)*K';
        dx=dx+K*innov;
        accepted(k)=true; s.accepted=s.accepted+1;
    else
        s.rejected=s.rejected+1;
    end
end
s.P=0.5*(s.P+s.P');
if any(accepted), s=eskf_inject_reset(s,dx); end
info.accepted=accepted; info.nis=nis; info.dx=dx;
end

