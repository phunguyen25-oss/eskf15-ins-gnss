function cal = mag_covariance_calibration(raw,target_norm)
% Quick symmetric hard/soft-iron estimate; raw must cover orientations uniformly.
assert(size(raw,2)==3 && size(raw,1)>30,'raw must be Nx3.');
if nargin<2, target_norm=1; end
cal.bias=mean(raw,1)'; centered=raw-cal.bias';
C=(centered'*centered)/size(centered,1);
[V,D]=eig(0.5*(C+C'));
assert(all(diag(D)>0),'Degenerate orientation coverage.');
W=V*diag(1./sqrt(diag(D)))*V';
tmp=(W*centered')'; scale=target_norm/mean(sqrt(sum(tmp.^2,2)));
cal.M=scale*W;
cal.apply=@(x) (cal.M*(x'-cal.bias))';
cal.residual_rms=sqrt(mean((sqrt(sum(cal.apply(raw).^2,2))-target_norm).^2));
cal.note='Validate on a separate dataset; covariance whitening assumes approximately uniform orientation coverage.';
end

