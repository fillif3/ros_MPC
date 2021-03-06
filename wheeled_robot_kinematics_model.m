function state = wheeled_robot_kinematics_model(input,state,timeStep)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
previous_state=state;
if input(2)==0
    state(1)=state(1)+input(1)*cos(state(3))*timeStep;
    state(2)=state(2)+input(1)*sin(state(3))*timeStep;
elseif input(1)==0
    state(3)=state(3)+input(2)*timeStep;
elseif sign(input(2))*sign(input(1))>0
    beta = state(3)+pi/2;
    R = input(1)/input(2);
    circleX = state(1)+cos(beta)*R;
    circleY = state(2)+sin(beta)*R;
    delta_theta= input(2)*timeStep;
    beta=beta+delta_theta;
    state(3)=state(3)+delta_theta;
    state(1)=circleX-cos(beta)*R;
    state(2)=circleY-sin(beta)*R;
else
    beta = state(3)-pi/2;
    R = input(1)/input(2);
    circleX = state(1)+cos(beta)*R;
    circleY = state(2)+sin(beta)*R;
    delta_theta= input(2)*timeStep;
    beta=beta-delta_theta;
    state(3)=state(3)+delta_theta;
    state(1)=circleX-cos(beta)*R;
    state(2)=circleY-sin(beta)*R;
end