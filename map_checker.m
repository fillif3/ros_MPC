function [c,ceq]= map_checker(inputs,currentState,model,horizon,...
    timeStep,map)
numberOfInputs= length(inputs)/horizon;
c=0;
ceq=0;
for i=1:horizon
    current_inputs = inputs((numberOfInputs*(i-1)+1):(numberOfInputs*i));
    currentState=model(current_inputs,currentState,timeStep);
    if getOccupancy(map,currentState(1:2))
        ceq =-1;
        return 
    end
end
end