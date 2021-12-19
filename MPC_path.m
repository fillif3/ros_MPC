function controlInputs= MPC_path(currentState,fullTrajectory,currentTime,timeStep,weights,fullTimeOfMission,...
    fullHorizon,minimumInput,maximumInput,model,amp)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
distFunction = @(trajectory,states) distance_function(trajectory,states,weights);
[trajectory,horizon] =getTrajectoryGivenTime(fullTrajectory,currentTime,fullHorizon,fullTimeOfMission,timeStep);
costFunction = @(x) cost_function(x,currentState,trajectory,model,horizon,distFunction,timeStep,minimumInput,maximumInput,map);
controlInputs = fminunc(costFunction,zeros(size(maximumInput)));

end