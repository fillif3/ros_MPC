%% MATLAB Template: Simple Loop
% Copyright 2017 The MathWorks, Inc.

%% SETUP
% Start or connect to ROS master
rosshutdown;
ipAddr = '';
rosinit(ipAddr)

% Create ROS subscribers
startingPoseSub = rossubscriber('/initialpose','geometry_msgs/PoseWithCovarianceStamped');
destinationSub=rossubscriber('/move_base_simple/goal','geometry_msgs/PoseStamped',@update_destination);
mapSub = rossubscriber('/map','nav_msgs/OccupancyGrid');
PoseSub = rossubscriber('/amcl_pose','geometry_msgs/PoseWithCovarianceStamped',@update_state);
disp('Wait for map')
mapMsg = receive(mapSub);
map = readBinaryOccupancyGrid(mapMsg);
inflate(map,0.3)
disp('Wait for start')
startingPose = receive(startingPoseSub);
disp('Wait for destination')
destination = receive(destinationSub);
disp('got everything')
% Create ROS publishers
[velPub,velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
[destinationPub,destinationMsg] = rospublisher('/destination','geometry_msgs/Pose');
[destinationMapPub,destinationMapMsg] = rospublisher('/destinationMap','geometry_msgs/PoseStamped');
destinationMapMsg.Header=startingPose.Header;

% Preparnig tools for debugging
%[debPubState,debMsgState] = rospublisher('/deb_pose','geometry_msgs/PoseStamped');
%[debPubStateNext,debMsgStateNext] = rospublisher('/deb_pose_next','geometry_msgs/PoseStamped');
%[debPubTarget,debMsgTarget] = rospublisher('/deb_target','geometry_msgs/PoseStamped');
%debMsgStateNext.Header=destination.Header;
 
% Start visualization

% Load parameters
timeStep=1;
delay=1;
r = rosrate(1/timeStep);
maxHorizon=3;
lb=[-0.22,-2.84];
ub=[0.22,2.84];


%Compute path

currentState(1) = startingPose.Pose.Pose.Position.X;
currentState(2) = startingPose.Pose.Pose.Position.Y;
currentState(3) = getHeading(startingPose.Pose.Pose.Orientation);
destinationState(1) = destination.Pose.Position.X;
destinationState(2) = destination.Pose.Position.Y;
destinationState(3) = getHeading(destination.Pose.Orientation);
%% CONTROL LOOP
% Start while-loop, which runs indefinitely and as quickly as possible.
reset(r)
realStates=currentState;
msgs={};
controlTrajectory=[0,0];
tic
while true
   
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
    %future_state=wheeled_robot_kinematics_model(controlTrajectory(1:2),currentState,delay);
    placehorderState=currentState;
    controlTrajectory=MPC_path(placehorderState,destinationState(1:2),timeStep,...
        maxHorizon,lb,ub,@wheeled_robot_kinematics_model,map);
    placehorderState=wheeled_robot_kinematics_model(controlTrajectory(1:2),placehorderState,timeStep);
    currentState=placehorderState;
    futureState=wheeled_robot_kinematics_model(controlTrajectory(3:4),placehorderState,timeStep);
    futureState=wheeled_robot_kinematics_model(controlTrajectory(5:6),futureState,timeStep);
    %futureState=wheeled_robot_kinematics_model(controlTrajectory(7:8),futureState,timeStep);
    %futureState=wheeled_robot_kinematics_model(controlTrajectory(9:10),futureState,timeStep);
    %disp('created control input')
    %disp(controlTrajectory(1:2))
    %disp(currentState)

    
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
    %velMsg.Linear.X = controlTrajectory(1);
    %velMsg.Angular.Z = controlTrajectory(2);
    %send(velPub,velMsg);
    %msgs{i}=[controlTrajectory(1);controlTrajectory(2)];
    destinationMsg.Position.X=futureState(1);
    destinationMsg.Position.Y=futureState(2);
    destinationMsg.Orientation=setQuanternionHeading(destination.Pose.Orientation,futureState(3));
    send(destinationPub,destinationMsg);
    destinationMapMsg.Pose=destinationMsg;
    send(destinationMapPub,destinationMapMsg);
    waitfor(r);
    %disp('after waiting')
    %toc
% End while-loop
end

function update_state(~,msg)
    currentState(1) = msg.Pose.Pose.Position.X;
    currentState(2) = msg.Pose.Pose.Position.Y;
    q = msg.Pose.Pose.Orientation;
    currentState(3) = atan2(2.0*(q.X * q.Y + q.W * q.Z), q.W*q.W + q.X*q.X - q.Y*q.Y - q.Z*q.Z);
    assignin('base','currentState',currentState)
    %disp('GET STATE')
    %disp(currentState)
end

function update_destination(~,msg)
    destinationState(1) = msg.Pose.Position.X;
    destinationState(2) = msg.Pose.Position.Y;
    q = msg.Pose.Orientation;
    destinationState(3) = atan2(2.0*(q.X * q.Y + q.W * q.Z), q.W*q.W + q.X*q.X - q.Y*q.Y - q.Z*q.Z);
    assignin('base','destinationState',destinationState)
    disp('GET destination')
    disp(destinationState)
end