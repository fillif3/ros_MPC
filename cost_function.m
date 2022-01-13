function cost= cost_function(inputs,currentState,model,horizon,destination,...
    timeStep)
numberOfInputs= length(inputs)/horizon;
cost=0;
for i=1:horizon
    current_inputs = inputs((numberOfInputs*(i-1)+1):(numberOfInputs*i));
    currentState=model(current_inputs,currentState,timeStep);
    %if getOccupancy(map,currentState(1:2))
    %    cost =realmax;
    %    return 
    %end
    cost=cost+norm(currentState(1:2) - destination);
end
end