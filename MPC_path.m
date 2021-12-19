function controlInputs= MPC_path(currentState,fullTrajectory,currentTime,timeStep,weights,...
    fullHorizon,minimumInput,maximumInput,model,map)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
distFunction = @(trajectory,states) distance_function(trajectory,states,weights);
[trajectory,horizon] =getTrajectoryGivenTime(fullTrajectory,currentTime,fullHorizon,timeStep);
costFunction = @(x) cost_function(x,currentState,trajectory,model,horizon,distFunction,timeStep,minimumInput,maximumInput,map);
[controlInputs,fval,a,b] = fminunc(costFunction,zeros(1,length(maximumInput)*horizon));
for i=1:(2*horizon)
    j= mod(i+1,2)+1;
    controlInputs(i)=sin(controlInputs(i))*(maximumInput(j)-minimumInput(j))/2+(maximumInput(j)+minimumInput(j))/2;
end
end