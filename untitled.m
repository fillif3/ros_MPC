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
PoseSub = rossubscriber('/acml_pose','geometry_msgs/PoseWithCovarianceStamped');
mapMsg = receive(mapSub);
map = readBinaryOccupancyGrid(mapMsg);
startingPose = receive(startingPoseSub);
destination = receive(destinationSub);
% Create ROS publishers
[velPub,velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
[pathPub,pathMsg] = rospublisher('/localpath','nav_msgs/Path');
 
% Start visualization

% Load parameters
pathLength=100;
timeStep=0.2;
weights=[1,0];
maxHorizon=10;
lb=[-0.22,-2.84];
ub=[0.22,2.84];


%Compute path
currentState(1) = startingPose.Pose.Pose.Position.X;
currentState(2) = startingPose.Pose.Pose.Position.Y;
q = startingPose.Pose.Pose.Orientation;
currentState(3) = atan2(2.0*(q.Y*q.Z + q.W*q.X), q.W*q.W - q.X*q.X - q.Y*q.Y + q.Z*q.Z);
destinationState(1) = destination.Pose.Position.X;
destinationState(2) = destination.Pose.Position.Y;
q = destination.Pose.Orientation;
destinationState(3) = atan2(2.0*(q.Y*q.Z + q.W*q.X), q.W*q.W - q.X*q.X - q.Y*q.Y + q.Z*q.Z);
path = global_planner(currentState,destinationState,map,pathLength);
for i=1:pathLength
    poseMsg = rosmessage("geometry_msgs/PoseStamped");
    poseMsg.Pose.Position.X=path.States(i,1);
    poseMsg.Pose.Position.Y=path.States(i,2);
    pathMsg.Poses(i)=poseMsg;
end
pathMsg.Header.FrameId = startingPose.Header.FrameId;
send(pathPub,pathMsg);
k=1;
%% CONTROL LOOP
% Start while-loop, which runs indefinitely and as quickly as possible.
firstTimeLoop=true;
currentTime=0;
for i=1:pathLength
    %% 1: SENSE
    % Get latest data from ROS subscribers
    if firstTimeLoop
        firstTimeLoop=false;
        positionTimeStamp=currentTime;
        index=1;
    else
        receivedMsg = PoseSub.LatestMessage;
        if isempty(receivedMsg)
           index=index+1; % If message is empty, next step
        elseif lastMsgId==receivedMsg.Header.seq
            index=index+1;
        else
            currentState(1) = startingPose.PoseWithCovariance.Pose.Point.x;
            currentState(2) = startingPose.PoseWithCovariance.Pose.Point.y;
            q = startingPose.PoseWithCovariance.Pose.Quaternion;
            currentState(3) = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
            lastMsgId=receivedMsg.Header.seq;
            index=1;
        end
    end

    %% 2: PROCESS
    % Run perception and control algorithms, which use received data and 
    % control parameters to produce some output.
    if index==1
        controlTrajectory=MPC_path(currentState,path.States,currentTime,timeStep,weights,...
            maxHorizon,lb,ub,@wheeled_robot_kinematics_model,map);
    
    end
    
    %% 3: CONTROL
    % Package and send control outputs as ROS messages
    velMsg.linear = controlTrajectory(2*index-1);
    velMsg.angular = controlTrajectory(2*index);
    send(velPub,velMsg);
    currentTime=currentTime+timeStep;

    
% End while-loop
end