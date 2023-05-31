clc;
close all;
clear all;


room_size = [10,10];
% Simulation Params
sim_t = 50;
% Init state.
x0 = [1; 1];
% Target position
% x0 = [0; 0; 0];
% Target position
params.p_d_f1 = [10; 8];
params.p_d_t1 = [1; 1];
params.v_d1 = [1;1];

params.state1 = [x0', 0];

params.p_d_f2 = [10; 1];
params.p_d_t2 = [1; 4];
params.v_d2 = [1;-1];

params.x_dim = 3;
params.u_dim = 2;

dt = 0.1;
% sim_t = 30;

params.cbf_gamma0 = 1;

params.u_max = 1;
params.u_min  = -1;

params.pid_p=0.5;
params.pV = 5; %5
params.m = 0;
params.safety_distance = 0.5;

params.clf.rate_max = 1;
params.clf.rate_min = 0.01;
params.cbf.rate_max = 1.5;
params.cbf.rate_min = 1;

params.d_cluster = 0.5; 
params.d_plane = 0.05;

params.weight.slack = 1;
params.weight.input = 5;

total_k = ceil(sim_t / dt);
x = x0;
t = 0;   

% initialize traces.
xs = zeros(total_k, params.x_dim);
% ts = zeros(total_k, 1);
% us = zeros(total_k-1, dynsys.udim);
% hs = zeros(total_k-1, 1);
% Vs = zeros(total_k-1, 1);
% xs(1, :) = x0';
% ts(1) = t;
% u_prev = [0;0];

% creat indoor scenario
indoor_scenario = robotScenario(UpdateRate=1/dt,StopTime=sim_t);
add_wall(indoor_scenario,[0.8 0.8 0.8])
% robot platform 1
robot1 = robotPlatform("robot1",indoor_scenario,"InitialBasePosition",[x0' 0],"ReferenceFrame","ENU","IsBinaryOccupied",1, "Collision","mesh");
AzimuthResolution = 0.16;
ElevationResolution = 1.25;
MaxRange = 7;
AzimuthLimits = [-180 180];
% AzimuthLimits = [-90 90];
% ElevationLimits = [-15 15];
theta=20;
ElevationLimits = [-10 10];
LidarMode = robotLidarPointCloudGenerator("UpdateRate",10, ...
    "MaxRange",MaxRange, ...
    "RangeAccuracy",3, ...
    "AzimuthResolution",AzimuthResolution,...
    "ElevationResolution",ElevationResolution, ...
    "AzimuthLimits",AzimuthLimits, ...
    "ElevationLimits",ElevationLimits, ...
    "HasOrganizedOutput",1, ...
    "HasNoise",0);
mountp = [0 0 1.1820]; mounttheta = [0 0 0]; % tand(20)*2/2+1
lidar1 = robotSensor("Lidar1",robot1,LidarMode, "MountingLocation",mountp, ...
    "MountingAngles",mounttheta);
roi1=[-MaxRange MaxRange -MaxRange MaxRange -mountp(3)+0.1 sind(theta)*MaxRange];
% roi1=[-MaxRange MaxRange -MaxRange MaxRange -sind(theta)*MaxRange sind(theta)*MaxRange];
updateMesh(robot1,"Cuboid","Size",[1 0.5 1],"Color",[0 0 1], "Collision","mesh");
TLidar2robot1 = [eul2rotm(mounttheta,'ZYX') mountp'; 0 0 0 1];

% robot platform 2
robot2 = robotPlatform("robot2",indoor_scenario,"InitialBasePosition",[params.p_d_t2' 0],"ReferenceFrame","ENU","IsBinaryOccupied",1, "Collision","mesh");
mountp2 = [0 0 1.1820]; mounttheta2 = [0 0 0];
roi2=[-MaxRange MaxRange -MaxRange MaxRange -mountp2(3)+0.1 sind(theta)*MaxRange];
lidar2 = robotSensor("Lidar2",robot2,LidarMode, "MountingLocation",mountp2, ...
    "MountingAngles",mounttheta2);
TLidar2robot2 = [eul2rotm(mounttheta2,'ZYX') mountp2'; 0 0 0 1];
updateMesh(robot2,"Cuboid","Size",[2 1 1],"Color",[0 1 1], "Collision","mesh");
%


[ax,plotFrames] = show3D(indoor_scenario,"View","Top");ax.ZLim =[0 5];ax.XLim = [-2 18];ax.YLim=[-2 18];
% [ax,plotFrames] = show3D(indoor_scenario,"View","Top");zlim([0 5]);xlim([-2 18]);ylim([-2 18]);
hold on

% bep = birdsEyePlot('Xlimits',[-5 5],'YLimits',[-5 5]);
% plotter = pointCloudPlotter(bep);
c=winter;
player1 = pcplayer([-2 18],[-2 18],[0 5],BackgroundColor=[1 1 1],ViewPlane="XY",Projection="orthographic",ColorSource="Z");
player2 = pcplayer([-2 18],[-2 18],[0 5],BackgroundColor=[1 1 1],ViewPlane="XY",Projection="orthographic",ColorSource="Z");
paramCCCs=[];
paramCCC2s=[];
setup(indoor_scenario)

idx=1;xs1(idx,:) = x0';
xs2(idx,:) = params.p_d_t2';

% 3d map & planner
res = 10; % cells/meter
map3d = occupancyMap3D(res);
map3d.FreeThreshold=0.5;
% map3d.OccupiedThreshold = 0.8; % default 0.65
ax2 = show(map3d);ax2.ZLim =[-0.1 5];ax2.XLim = [-2 18];ax2.YLim=[-2 18];

% generate rrt planner
ss1 = stateSpaceSE3([0.5 10; 0.5 10; 0 0; inf inf; inf inf; inf inf; inf inf]);
sv1 = validatorOccupancyMap3D(ss1,"Map",map3d,"ValidationDistance",0.1);
planner1 = plannerRRTStar(ss1,sv1,"GoalBias",0.1,"MaxConnectionDistance",2);
[pthObj, solnInfo] = plan(planner1, [params.state1(1:2) 0 1 0 0 0], [params.p_d_f1' 0 1 0 0 0]);
% rng(1, 'twister');
waypoints = pthObj.States(:,1:2);
controller1 = controllerPurePursuit("Waypoints",waypoints,"DesiredLinearVelocity",3,"MaxAngularVelocity",3*pi);
unicycle = unicycleKinematics("VehicleInputs","VehicleSpeedHeadingRate");

% 2d map & planner
ss1_2d = stateSpaceSE2([0.5 10; 0.5 10; -pi pi]);
sv1_2d = validatorOccupancyMap(ss1_2d);
map2d = occupancyMap(12, 12, res);
map2d.DefaultValue = 0.5;
sv1_2d.Map = map2d;
sv1_2d.ValidationDistance = 0.01;

planner1 = plannerRRT(ss1_2d,sv1_2d,'MaxConnectionDistance',0.3,'GoalBias',1);

figure_map2d=figure(10);
figure_map3d=figure(9);
figure_scene=figure(1);

localMapN=3;

while advance(indoor_scenario)
    localMapCnt=0;
    ts(idx) = indoor_scenario.CurrentTime;

    % update sensor 1 & current motion of robot 1 & map
    [isUpdated1, ~, sensorReadings1(idx)] = lidar1.read();
    motion1 = read(robot1); p1(idx,:) = motion1(1:2); orientation1(idx,:) = motion1(10:13);% quat2rotm(orientation(idx,:));
    [yaw, ~, ~] = quat2angle(motion1(10:13));
    params.state1 = [p1(idx,:) yaw];
    

    sign_temp = ((params.p_d_f1 - params.p_d_t1).*params.v_d1)>= 0;
%     params.p_d_t1 = params.p_d_t1.*sign_temp + params.p_d_f1.* not(sign_temp);
    
    % update sensor 2 & current motion of robot 2
    [isUpdated2, ~, sensorReadings2(idx)] = lidar2.read();
    motion2 = read(robot2); p2(idx,:) = motion2(1:2); orientation2(idx,:) = motion2(10:13);% quat2rotm(orientation(idx,:));
    params.p_d_t2 = params.p_d_t2 + dt*params.v_d2;
    sign_temp = ((params.p_d_f2 - params.p_d_t2).*params.v_d2)>= 0;
    params.p_d_t2 = params.p_d_t2.*sign_temp + params.p_d_f2.* not(sign_temp);

    % plot the Lidar reading of robot 1 (after removing ground via roi)
    T1 =[quat2rotm(orientation1(idx,:)) [p1(idx,:) 0]';0 0 0 1] * TLidar2robot1;
    indices1 = findPointsInROI( sensorReadings1(idx),roi1 );
    ptCloudB1 = select(sensorReadings1(idx),indices1);
    Ptcloud1 = pctransform(removeInvalidPoints(ptCloudB1),rigidtform3d(T1));
    view(player1,Ptcloud1);
    
    % plot the Lidar reading of robot 2 (after removing ground via roi)
    T2 =[quat2rotm(orientation2(idx,:)) [p2(idx,:) 0]';0 0 0 1] * TLidar2robot2;
    indices2 = findPointsInROI( sensorReadings2(idx),roi2 );
    ptCloudB2 = select(sensorReadings2(idx),indices2);
    Ptcloud2 = pctransform(removeInvalidPoints(ptCloudB2),rigidtform3d(T2));
    view(player2,Ptcloud2);

    % update the map and the map in the plannner 1
    insertPointCloud(map3d, [0 0 0 1 0 0 0],...
        pctransform(removeInvalidPoints(sensorReadings1(idx)),rigidtform3d(T1)), MaxRange);
    sv1.Map = map3d;

    [ranges, angles] = pointCloudToPolar(removeInvalidPoints(sensorReadings1(idx)));

    % Insert the ray into the map
    eul=quat2eul(orientation1(idx,:));
    T1_2d=[T1(1,4) T1(2,4) eul(1)];
    % Calculate the end points of each range
    endpoints_x = T1_2d(1) + ranges .* cos(angles);
    endpoints_y = T1_2d(2) + ranges .* sin(angles);
    
    % Get the size of the map
    map_x_limits = map2d.XWorldLimits;
    map_y_limits = map2d.YWorldLimits;
    
    % Find which endpoints are within the map limits
    valid = (endpoints_x >= map_x_limits(1)) & (endpoints_x <= map_x_limits(2)) ...
          & (endpoints_y >= map_y_limits(1)) & (endpoints_y <= map_y_limits(2));
    
    % Filter the ranges and angles based on this
    valid_ranges = ranges(valid);
    valid_angles = angles(valid);
    
    % Then call insertRay with the filtered data
    insertRay(map2d, T1_2d, valid_ranges, valid_angles, MaxRange);

    if localMapCnt >= localMapN
        % Reset all cells to be unoccupied
        setOccupancy(map2d, [0 0], 0, 'grid')
        localMapCnt=0;
    else
        localMapCnt=localMapCnt+1;
    end
    % Reset all cells to be unoccupied
    sv1_2d.Map=map2d;
    
    % if idx == 1
    % % init map
    %     [pthObj,solnInfo] = planner1.plan([params.state1(1:2) 0], [params.p_d_f1' 0]);
    %     waypoints = pthObj.States(:,1:3);
    % end

    % check if the past path is valid for the current map 
    isPathValid = isStateValid(sv1,pthObj.States);

    if sum(isPathValid) ~= pthObj.NumStates
        % replanning and update the target
       [pthObj, solnInfo] = plan(planner1,[motion1(1:3) motion1(10:13)],[params.p_d_f1' 0 1 0 0 0]);       
    end
%     params.p_d_t1 = pthObj.States(2,1:2)';

    figure(10);
    subplot(1,2,1); % create a subplot in the left side
    show(map2d);
    title('2D Map');
    
    subplot(1,2,2); % create a subplot in the right side
    show(map3d);
    title('3D Map');


    figure(figure_scene);
    show(map3d,"Parent",ax2)
    refreshdata % Refresh all plot data and visualize.
    drawnow limitrate
  
    % plot the scenario and platform
    show3D(indoor_scenario,FastUpdate=true,Parent=ax);  % Use fast update to move platform visualization frames.
    refreshdata % Refresh all plot data and visualize.
    drawnow limitrate

    % if p reach target positon break
    if norm(p1(idx,:)- params.p_d_f1') <= 0.05 & norm(p2(idx,:)- params.p_d_f2') <= 0.05
        break
    else
    % else update controller

        % 1. pointcloud -> CBFs 
        paramCCC = CBF_estimation(Ptcloud1,T1,params);
        paramCCCs=[paramCCCs;paramCCC];
        % 2. conpute u
%         tic
%         [u, slack, h, V(idx), clf,cbf, ~] = CbfClfQP(paramCCC, p1(idx,:),params,params.p_d_t1'-p1(idx,:));
%         toc
        controller1.Waypoints   = pthObj.States(:,1:2);
        u = exampleHelperMobileRobotController(controller1,[p1(idx,:) theta],[pthObj.States(2,1:2) 0],0.1);
        % 3. update robot state
        [ts_temp, xs_temp] = ode45(@(t,s)derivative(unicycle,s,u),[t t+dt],params.state1);

%         [ts_temp, xs_temp] = ode45(@(t, s) odefcn(t, s, u'), [t t+dt], p1(idx,:)');
%         xs1(idx+1,:) = xs_temp(end, :)';
xs(idx+1,:) = xs_temp(end, :);
        u_prev = u;
        % 3. update robot1 motion
        motion = [xs_temp(end, 1:2) 0 0 0 0 0 0 0 angle2quat( 0, 0, xs_temp(end, 3), 'YXZ' ) 0 0 0];
        move(robot1,"base", motion);

        % update robot2 motion
        paramCCC2 = CBF_estimation(Ptcloud2,T2,params);
        paramCCC2s=[paramCCC2s;paramCCC2];
        [u, slack, h, V(idx), clf,cbf, ~] = CbfClfQP(paramCCC2,p2(idx,:),params,params.p_d_t2'-p2(idx,:));
        [ts_temp, xs_temp] = ode45(@(t, s) odefcn(t, s, u'), [t t+dt], p2(idx,:)');
        motion = [ xs_temp(end, :) 0 u 0 0 0 0 angle2quat( 0, 0, 0, 'YXZ' ) 0 0 0];
        xs2(idx+1,:) = xs_temp(end, :)';
        move(robot2,"base", motion);


    end
    % update
    updateSensors(indoor_scenario)
    idx = idx+1;

end
hold off


    occupancyMap = binaryOccupancyMap(indoor_scenario,MapHeightLimits=[-1 1], ...
                                  GridOriginInLocal=[-1 -1],MapSize=[room_size(1)+2 room_size(2)+2]);
    figure(3);show(occupancyMap);grid on;yticks(-1:0.25:room_size(1)+2);xticks(-1:0.25:room_size(2)+2);hold on
    plot(xs1(:,1),xs1(:,2),'r-')
    plot(xs2(:,1),xs2(:,2),'r-')
% figure(1);
% ax = show3D(indoor_scenario);
% view(-65,45);zlim([0 5]);xlim([-2 18]);ylim([-2 18]);
% light
% grid on


%%

function add_wall(indoor_scenario,wallColor)
room_size = [12,10];
    wallHeight = 5;
    wallWidth = 0.25*2;
    wallLength = 10;
    % addMesh(indoor_scenario,"Cylinder","Position",[3 5 2.5],"Size",[2 5],"Color",[0.8 0.8 0.8]);
    addMesh(indoor_scenario,"Plane","Position",[room_size(1)/2 room_size(2)/2 0],"Size",[room_size(1)+2 room_size(2)+2],"Color",[0.8 0.8 0.8],"Collision","mesh");
    % Add outer walls.
%    addMesh(indoor_scenario,"Box",Position=[0, room_size(2)/2, wallHeight/2],...
%         Size=[wallWidth, room_size(2)+wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true); %left
   addMesh(indoor_scenario,"Box",Position=[room_size(1), room_size(2)/2, wallHeight/2],...
        Size=[wallWidth, room_size(2)+wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true,Collision="mesh");%right
%    addMesh(indoor_scenario,"Box",Position=[room_size(1)/2, 0, wallHeight/2],...
%         Size=[room_size(1)+wallWidth, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true); %bottom
   addMesh(indoor_scenario,"Box",Position=[room_size(1)/2, room_size(2), wallHeight/2],...
       Size=[room_size(1)+wallWidth, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true,Collision="mesh"); %upper
   
%    addMesh(indoor_scenario,"Box",Position=[room_size(1)*3/8, room_size(2)/2, wallHeight/2],...
%         Size=[wallWidth*2, wallWidth*2, wallHeight],Color=wallColor,IsBinaryOccupied=true); % centre

    % Add inner walls.
    addMesh(indoor_scenario,"Box",Position=[room_size(1)/8, room_size(2)/2, wallHeight/2],...
        Size=[room_size(1)/4, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true,Collision="mesh");
    addMesh(indoor_scenario,"Box",Position=[room_size(1)/4, room_size(2)/2, wallHeight/2],...
        Size=[wallWidth, room_size(2)/3,  wallHeight],Color=wallColor,IsBinaryOccupied=true,Collision="mesh");
    addMesh(indoor_scenario,"Box",Position=[room_size(1)*6/8, room_size(2)/2, wallHeight/2],...
        Size=[room_size(1)/2, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true,Collision="mesh");
    addMesh(indoor_scenario,"Box",Position=[room_size(1)/2, room_size(2)/2, wallHeight/2],...
        Size=[wallWidth, room_size(2)/3, wallHeight],Color=wallColor,IsBinaryOccupied=true,Collision="mesh");
%     addMesh(indoor_scenario,"Box",Position=[wallLength/8, wallLength/3, wallHeight/2],...
%         Size=[wallLength/4, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
%     addMesh(indoor_scenario,"Box",Position=[wallLength/4, wallLength/3, wallHeight/2],...
%         Size=[wallWidth, wallLength/6,  wallHeight],Color=wallColor,IsBinaryOccupied=true);
%     addMesh(indoor_scenario,"Box",Position=[(wallLength-wallLength/4), wallLength/2, wallHeight/2],...
%         Size=[wallLength/2, wallWidth, wallHeight],Color=wallColor,IsBinaryOccupied=true);
%     addMesh(indoor_scenario,"Box",Position=[wallLength/2, wallLength/2, wallHeight/2],...
%         Size=[wallWidth, wallLength/3, wallHeight],Color=wallColor,IsBinaryOccupied=true);
end

function dx = odefcn(t, x, u)
    dx = zeros(2,2)*x+ u;
end

function robotRefState = exampleHelperMobileRobotController(purePursuitObj, robotPose, robotGoal, goalRadius)
% This function is for internal use only and may be removed in a future
% release.
%exampleHelperMobileRobotController Convert outputs of pure pursuit controller to a single output
%   This function accepts a pure pursuit controller object, PUREPURSUITOBJ,
%   and the current pose of the robot under control. The function combines
%   the two outputs of the controller into a single 2x1 ouput,
%   ROBOTREFSTATE. This allows the controller to be easily used with ode
%   solvers.

%   Copyright 2019 The MathWorks, Inc.

% Get controller output
controller = purePursuitObj;
[vRef, wRef] = controller(robotPose);

% Stop controller when goal is reached
distanceToGoal = norm(robotPose(1:2) - robotGoal(:));
if distanceToGoal < goalRadius
    vRef = 0;
    wRef = 0;
end

robotRefState = [vRef; wRef];
end