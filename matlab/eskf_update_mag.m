function [s,info] = eskf_update_mag(s,mag_b,mag_n,Rmag)
pred=quat_to_rot(s.q)'*mag_n(:);
H=zeros(3,15); H(:,7:9)=skew3(pred);
[s,info]=eskf_update_sequential(s,mag_b(:)-pred,H,diag(Rmag));
end
