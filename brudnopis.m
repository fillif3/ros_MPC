load('matlab.mat')
timeStep=0.5;
%r = rosrate(1/timeStep);
maxHorizon=3;
lb=[-0.22,-2.84];
ub=[0.22,2.84];

currentState(1) = startingPose.Pose.Pose.Position.X;
currentState(2) = startingPose.Pose.Pose.Position.Y;
q = startingPose.Pose.Pose.Orientation;
currentState(3) = atan2(2.0*(q.Y*q.X + q.W*q.Z), q.W*q.W - q.Z*q.Z - q.Y*q.Y + q.X*q.X);
destinationState(1) = destination.Pose.Position.X;
destinationState(2) = destination.Pose.Position.Y;
q = destination.Pose.Orientation;
destinationState(3) = atan2(2.0*(q.Y*q.X + q.W*q.Z), q.W*q.W - q.Z*q.Z - q.Y*q.Y + q.X*q.X);
show(map)
inflate(map,0.1)

hold on
plot(currentState(1),currentState(2),'*')
plot(destinationState(1),destinationState(2),'*')
realStates=currentState;
tic
for i=1:20
    inputs = MPC_path(currentState,destinationState(1:2),timeStep,...
            maxHorizon,lb,ub,@wheeled_robot_kinematics_model,map);
    currentState = wheeled_robot_kinematics_model(inputs(1:2),currentState,timeStep);
    realStates(i+1,:)= currentState;
    disp(inputs(1:2))
    %currentTime=currentTime+timeStep;
end
toc
plot(realStates(:,1),realStates(:,2))
hold on
