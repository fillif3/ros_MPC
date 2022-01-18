function yaw= getHeading(q)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
yaw = atan2(2.0*(q.X * q.Y + q.W * q.Z), q.W*q.W + q.X*q.X - q.Y*q.Y - q.Z*q.Z);
end