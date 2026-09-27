function q = euler_zyx_to_quat(roll,pitch,yaw)
cr=cos(roll/2); sr=sin(roll/2);
cp=cos(pitch/2); sp=sin(pitch/2);
cy=cos(yaw/2); sy=sin(yaw/2);
q=[cr*cp*cy+sr*sp*sy; sr*cp*cy-cr*sp*sy; ...
   cr*sp*cy+sr*cp*sy; cr*cp*sy-sr*sp*cy];
q=q/norm(q);
end

