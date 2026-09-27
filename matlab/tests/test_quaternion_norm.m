function test_quaternion_norm()
c=eskf_default_config(); s=eskf_init(zeros(3,1),zeros(3,1),[1;0;0;0],zeros(3,1),zeros(3,1),eye(15),c);
for k=1:1000, s=eskf_predict(s,[0;0;-c.g],[0.4;-0.2;0.1],0.002); end
assert(abs(norm(s.q)-1)<1e-12);
end

