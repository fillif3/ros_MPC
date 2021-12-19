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
particleCloudSub = rossubscriber('/particlecloud','geometry_msgs/PoseArray');
mapMsg = receive(mapSub);
map = readBinaryOccupancyGrid(mapMsg);
startingPose = receive(startingPoseSub);
destination = receive(destinationSub);
% Create ROS publishers
[velPub,velMsg] = rospublisher('/cmd_vel','geometry_msgs/Twist');
[pathPub,pathMsg] = rospublisher('/localpath','nav_msgs/Path');
 
% Start visualization

% Load parameters
path_length=300;


%% CONTROL LOOP
% Start while-loop, which runs indefinitely and as quickly as possible.

while(currentTime < 10)
    %% 1: SENSE
    % Get latest data from ROS subscribers
    receivedMsg = mySub.LatestMessage;
    if isempty(receivedMsg)
       receivedData = rand; % If message is empty, assign random number
    else
       receivedData = receivedMsg.Z;
    end

    %% 2: PROCESS
    % Run perception and control algorithms, which use received data and 
    % control parameters to produce some output.
    ctrlInput  = myPerceptionAlgorithm(receivedData,perceptionParam);
    ctrlOutput = myControlAlgorithm(ctrlInput,controlParams);
    
    %% 3: CONTROL
    % Package and send control outputs as ROS messages
    pubMsg.X = ctrlOutput;
    send(myPub,pubMsg);
    
    %% 4: VISUALIZE
    % (Optional) Visualize data as the algorithm is running
    currentTime = toc;
    plot(myViz,currentTime,ctrlOutput,'bo','MarkerSize',5)
    drawnow

    % (Optional) Pause execution to add delay between iterations
    pause(0.1)
    
% End while-loop
end