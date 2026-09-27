function q = quat_exp(phi)
% Exp map from a rotation vector to a unit quaternion.
phi=phi(:); a=norm(phi);
if a < 1e-8
    q=[1-0.125*a*a; 0.5*phi];
else
    q=[cos(0.5*a); sin(0.5*a)*phi/a];
end
q=q/norm(q);
end

