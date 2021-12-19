function distance= distance_function(trajectory,predictedStates,weights)
distance=0;
for i=1:length(trajectory)
    trajectoryPosition=trajectory{i};
    statePosition=predictedStates{i};
    distance=distance+weights(1)*norm(trajectoryPosition(1:2) - statePosition(1:2));
    distance_radians = abs(trajectoryPosition(3)-statePosition(3));
    distance_radians=mod(distance_radians,2*pi);
    distance_radians=min(distance_radians,2*pi-distance_radians);
    distance=distance+distance_radians*weights(2);
end