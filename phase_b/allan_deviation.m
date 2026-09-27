function [tau,adev] = allan_deviation(rate,fs)
% Overlapping Allan deviation for uniformly sampled rate data (NxC).
if isvector(rate), rate=rate(:); end
N=size(rate,1); maxm=floor((N-1)/2);
m=unique(max(1,round(logspace(0,log10(maxm),50))));
cs=[zeros(1,size(rate,2));cumsum(rate,1)];
tau=zeros(numel(m),1); adev=zeros(numel(m),size(rate,2));
for j=1:numel(m)
    mj=m(j); avg=(cs(1+mj:end,:)-cs(1:end-mj,:))/mj;
    delta=avg(1+mj:end,:)-avg(1:end-mj,:);
    adev(j,:)=sqrt(0.5*mean(delta.^2,1)); tau(j)=mj/fs;
end
end

