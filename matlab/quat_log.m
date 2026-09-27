function phi = quat_log(q)
q=q(:)/norm(q);
if q(1)<0, q=-q; end
s=norm(q(2:4));
if s<1e-10, phi=2*q(2:4); else, phi=2*atan2(s,q(1))*q(2:4)/s; end
end

