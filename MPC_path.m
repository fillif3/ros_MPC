function controlInputs= MPC_path(currentState,destination,timeStep,...
    fullHorizon,minimumInput,maximumInput,model,map)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%distFunction = @(trajectory,states) distance_function(trajectory,states,weights);
%[trajectory,horizon] =getTrajectoryGivenTime(fullTrajectory,currentTime,fullHorizon,timeStep);
lb=repmat(minimumInput,1,fullHorizon);
ub=repmat(maximumInput,1,fullHorizon);
costFunction = @(x) cost_function(x,currentState,model,fullHorizon,destination,timeStep);
nonlocon = @(x) map_checker(x,currentState,model,fullHorizon,timeStep,map);
options = optimoptions('ga','Display','off','MaxGenerations',2);
[controlInputs,fval,a,b] = ga(costFunction,length(maximumInput)*fullHorizon,[],[]...
    ,[],[],lb,ub,nonlocon,options);

end