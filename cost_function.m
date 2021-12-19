function cost= cost_function(inputs,currentState,trajectory,model,horizon,distanceFunction,timeStep,minimumInput,maximumInput,map)
numberOfInputs= length(inputs)/horizon;
predictedStates=cell(1,horizon);
for i=1:horizon
    current_inputs = inputs((numberOfInputs*(i-1)+1):(numberOfInputs*i));
    for j=1:numberOfInputs
        current_inputs(j)=sin(current_inputs(j))*(maximumInput(j)-minimumInput(j))/2+(maximumInput(j)+minimumInput(j))/2;
    end
    currentState=model(current_inputs,currentState,timeStep);
    %if getOccupancy(map,currentState(1:2))
    %    cost =realmax;
    %    return 
    %end
    predictedStates{i}=currentState;
end
cost=distanceFunction(trajectory,predictedStates);
end