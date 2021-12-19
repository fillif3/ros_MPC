function pthObj = global_planner(starting_pose,destination,map,pathLength)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%costmap = vehicleCostmap(map);
rng default
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.05;

planner = plannerRRT(ss,stateValidator);
planner.MaxConnectionDistance = 2.0;
planner.MaxIterations = 30000;

planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

[pthObj, ~] = plan(planner,starting_pose,destination);
interpolate(pthObj,pathLength)


function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
    isReached = false;
    threshold = 0.01;
    if planner.StateSpace.distance(newState, goalState) < threshold
        isReached = true;
    end
end

end