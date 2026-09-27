function [F,G,Qc] = eskf_FG(s,a_corr,w_corr,Rnb)
F=zeros(15);
F(1:3,4:6)=eye(3);
F(4:6,7:9)=-Rnb*skew3(a_corr);
F(4:6,10:12)=-Rnb;
F(7:9,7:9)=-skew3(w_corr);
F(7:9,13:15)=-eye(3);
G=zeros(15,12);
G(4:6,1:3)=Rnb;
G(7:9,4:6)=-eye(3);
G(10:12,7:9)=eye(3);
G(13:15,10:12)=eye(3);
c=s.cfg;
Qc=diag([repmat(c.sigma_acc^2,1,3), ...
         repmat(c.sigma_gyro^2,1,3), ...
         repmat(c.sigma_ba^2,1,3), ...
         repmat(c.sigma_bg^2,1,3)]);
end

