%% MATLAB Template: Simple Loop
% Copyright 2017 The MathWorks, Inc.

%% SETUP
% Start or connect to ROS master
rosshutdown;
ipAddr = '';
rosinit(ipAddr)

% Create ROS subscribers
startingPoseSub = rossubscriber('/initialpose','geometry_msgs/PoseWithCovarianceStamped');
destinationSub=rossubscriber('/move_base_simple/goal','geometry_msgs/PoseStamped');
mapSub = rossubscriber('/map','nav_msgs/OccupancyGrid');
PoseSub = rossubscriber('/amcl_pose','geometry_msgs/PoseWithCovarianceStamped');
disp('Wait for map')
mapMsg = receive(mapSub);
map = readBinaryOccupancyGrid(mapMsg);
inflate(map,0.1)
disp('Wait for start')
startingPose = receive(startingPoseSub);
disp('Wait for destination')
destination = receive(destinationSub);
disp('got everything')
% Create ROS publishers
[velPub,velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
% Preparnig tools for debugging
%[debPubState,debMsgState] = rospublisher('/deb_pose','geometry_msgs/PoseStamped');
%[debPubStateNext,debMsgStateNext] = rospublisher('/deb_pose_next','geometry_msgs/PoseStamped');
%[debPubTarget,debMsgTarget] = rospublisher('/deb_target','geometry_msgs/PoseStamped');
%debMsgStateNext.Header=destination.Header;
 
% Start visualization

% Load parameters
timeStep=0.5;
r = rosrate(1/timeStep);
maxHorizon=3;
lb=[-0.22,-2.84];
ub=[0.22,2.84];


%Compute path

currentState(1) = startingPose.Pose.Pose.Position.X;
currentState(2) = startingPose.Pose.Pose.Position.Y;
q = startingPose.Pose.Pose.Orientation;
currentState(3) = atan2(2.0*(q.X * q.Y + q.W * q.Z), q.W*q.W + q.X*q.X - q.Y*q.Y - q.Z*q.Z);
destinationState(1) = destination.Pose.Position.X;
destinationState(2) = destination.Pose.Position.Y;
q = destination.Pose.Orientation;
destinationState(3) = atan2(2.0*(q.X * q.Y + q.W * q.Z), q.W*q.W + q.X*q.X - q.Y*q.Y - q.Z*q.Z);
lastMsgId=0;
%% CONTROL LOOP
% Start while-loop, which runs indefinitely and as quickly as possible.
firstTimeLoop=true;
reset(r)
realStates=currentState;
msgs={};
for i=1:100
    %% 1: SENSE
    % Get latest data from ROS subscribers
%     if firstTimeLoop
%         firstTimeLoop=false;
%         index=1;
%     else
%         receivedMsg = PoseSub.LatestMessage;
%         if isempty(receivedMsg)
%            index=index+1; % If message is empty, next step
%            disp('empty')
%         elseif lastMsgId==receivedMsg.Header.Seq
%             index=index+1;
%             disp('same')
%         else
%             disp('mext')
%             currentState(1) = receivedMsg.Pose.Pose.Position.X;
%             currentState(2) = receivedMsg.Pose.Pose.Position.Y;
%             q = startingPose.Pose.Pose.Orientation;
%             currentState(3) = atan2(2.0*(q.Y*q.Y + q.W*q.Z), q.W*q.W - q.Z*q.Z - q.Y*q.Y + q.X*q.X);
%             lastMsgId=receivedMsg.Header.Seq;
%             index=1;
%         end
%     end

    %% 2: PROCESS
    % Run perception and control algorithms, which use received data and 
    % control parameters to produce some output.
    controlTrajectory=MPC_path(currentState,destinationState(1:2),timeStep,...
        maxHorizon,lb,ub,@wheeled_robot_kinematics_model,map);
    currentState=wheeled_robot_kinematics_model(controlTrajectory(1:2),currentState,timeStep);
    realStates(i+1,:)= currentState;

    
%     disp('input')
%     disp(controlTrajectory(2*index-1:2*index))
%     disp('next state')
%     disp(currentState)
%     currentState = wheeled_robot_kinematics_model(controlTrajectory(2*index-1:2*index),currentState,timeStep);
%     debMsgStateNext.Pose.Position.X=currentState(1);
%     debMsgStateNext.Pose.Position.Y=currentState(2);
%     debMsgStateNext.Pose.Orientation.Z=currentState(3);
%     send(debPubStateNext,debMsgStateNext);

    
    %% 3: CONTROL
    % Package and send control outputs as ROS messages
    velMsg.Linear.X = controlTrajectory(1);
    velMsg.Angular.Z = controlTrajectory(2);
    send(velPub,velMsg);
    msgs{i}=[controlTrajectory(1);controlTrajectory(2)];
    waitfor(r);
% End while-loop
end