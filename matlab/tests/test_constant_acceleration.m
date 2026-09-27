function test_constant_acceleration()
c=eskf_default_config(); c.sigma_acc=0; c.sigma_gyro=0; c.sigma_ba=0; c.sigma_bg=0;
s=eskf_init(zeros(3,1),zeros(3,1),[1;0;0;0],zeros(3,1),zeros(3,1),eye(15),c);
dt=0.01; s=eskf_predict(s,[1;0;-c.g],[0;0;0],dt);
assert(abs(s.p(1)-0.5*dt^2)<1e-12); assert(abs(s.v(1)-dt)<1e-12);
end

