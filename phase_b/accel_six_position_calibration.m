function cal = accel_six_position_calibration(means6,g)
% Rows: +X,-X,+Y,-Y,+Z,-Z stationary means, in m/s^2.
if nargin<2, g=9.80665; end
assert(isequal(size(means6),[6,3]),'means6 must be 6x3.');
plus=[means6(1,1);means6(3,2);means6(5,3)];
minus=[means6(2,1);means6(4,2);means6(6,3)];
cal.bias=0.5*(plus+minus);
gain=(plus-minus)/(2*g);
cal.M=diag(1./gain);
cal.apply=@(raw) (cal.M*(raw'-cal.bias))';
cal.note='Diagonal six-position model; use ellipsoid/non-orthogonality fit when cross-axis error matters.';
end

