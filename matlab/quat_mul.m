function q = quat_mul(a,b)
% Hamilton product, scalar first.
a=a(:); b=b(:);
q = [a(1)*b(1)-dot(a(2:4),b(2:4)); ...
     a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end

