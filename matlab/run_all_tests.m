function run_all_tests()
root=fileparts(mfilename('fullpath')); addpath(root,fullfile(root,'tests'));
names={'test_constant_acceleration','test_quaternion_norm', ...
       'test_covariance_health','test_mag_jacobian','test_van_loan'};
for k=1:numel(names)
    feval(names{k}); fprintf('[PASS] %s\n',names{k});
end
fprintf('All %d MATLAB tests passed.\n',numel(names));
end
