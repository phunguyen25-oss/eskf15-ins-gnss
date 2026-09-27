function e = state_error(s,truth,k)
dq=quat_mul(quat_conj(s.q),truth.q(:,k));
e=[truth.p(:,k)-s.p; truth.v(:,k)-s.v; quat_log(dq); ...
   truth.ba(:,k)-s.ba; truth.bg(:,k)-s.bg];
end

