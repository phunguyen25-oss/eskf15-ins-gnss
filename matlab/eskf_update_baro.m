function [s,info] = eskf_update_baro(s,altitude,variance)
H=zeros(1,15); H(3)=-1;
[s,info]=eskf_update_sequential(s,altitude-(-s.p(3)),H,variance);
end

