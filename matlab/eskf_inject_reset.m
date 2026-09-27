function s = eskf_inject_reset(s,dx)
dx=dx(:);
s.p=s.p+dx(1:3); s.v=s.v+dx(4:6);
s.q=quat_mul(s.q,quat_exp(dx(7:9))); s.q=s.q/norm(s.q);
s.ba=s.ba+dx(10:12); s.bg=s.bg+dx(13:15);
J=eye(15); J(7:9,7:9)=eye(3)-0.5*skew3(dx(7:9));
s.P=J*s.P*J'; s.P=0.5*(s.P+s.P');
end

