function q= setQuanternionHeading(q,yaw)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
q.X=0;
q.Y=0;
q.Z=sin(yaw/2);
q.W=cos(yaw/2);
end