function [trajectory,horizon] = getTrajectoryGivenTime(fullTrajectory,currentTime,fullHorizon,timeStep)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
fullTimeOfMission = timeStep*length(fullTrajectory);
horizon = min(floor((fullTimeOfMission-currentTime)/timeStep),fullHorizon);
weightForInterpolation = 1-mod(currentTime,timeStep)/timeStep;
index= floor(currentTime/timeStep);
trajectory=cell(1,horizon);
position = zeros(1,3);
for i=1:horizon
    position(1:2) =weightForInterpolation*fullTrajectory(index+i,1:2)+...
        (1-weightForInterpolation)*fullTrajectory(index+i+1,1:2);
    position(3) = fullTrajectory(index+i,3); %It can be made better
    trajectory{i}=position;
end