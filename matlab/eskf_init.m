function s = eskf_init(p,v,q,ba,bg,P0,cfg)
if nargin < 7, cfg=eskf_default_config(); end
s.p=p(:); s.v=v(:); s.q=q(:)/norm(q); s.ba=ba(:); s.bg=bg(:);
s.P=0.5*(P0+P0');
s.omega=zeros(3,1);
s.cfg=cfg;
s.accepted=0; s.rejected=0;
end

