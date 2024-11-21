%% Template Exercises Manipulation - Cooperative Robotics a.y. 24-25
addpath('./simulation_scripts');
clc;
clear;
close all
real_robot = false;

%% Initialization - DON'T CHANGE ANYTHING from HERE ... 
% Simulation variables (integration and final time)
dt = 0.005;
Tf = 15; %simulation time
loop = 1;
maxloops = ceil(Tf/dt);
mission.phase = 1;
mission.phase_time = 0;
model = load("panda.mat");

% UDP Connection with Franka Interface
if real_robot == true
    hudprLeft = dsp.UDPReceiver('LocalIPPort',1501,'MaximumMessageLength',255);
    hudprRight = dsp.UDPReceiver('LocalIPPort',1503,'MaximumMessageLength',255);
    hudpsLeft = dsp.UDPSender('RemoteIPPort',1500);
    hudpsLeft.RemoteIPAddress = '127.0.0.1';
    hudpsRight = dsp.UDPSender('RemoteIPPort',1502);
    hudpsRight.RemoteIPAddress = '127.0.0.1';
else
    hudps = dsp.UDPSender('RemoteIPPort',1505);
    hudps.RemoteIPAddress = '127.0.0.1';
end
%% ... to HERE.
% Init robot model

wTb_left = eye(4); % fixed transformation word -> base1
wTb_right = [rotation(0,0,pi),[1.06 -0.01 0]'; 0 0 0 1]; % fixed transformation word -> base2

eRt=rotation(0,0, deg2rad(-44.9949));
%%% General R-L
eOt=[0 0 0.2104]';

%%%Left tool

bRe_left=pandaArm.ArmL.bTe[1:3,1:3];
bRt_left=bRe_left*eRt;
bOe_left=pandaArm.ArmL.bTe[1:3,4];
bOt_left=bOe+eOt;
bTt_left=[bRt_left, bOt_left;zeros(1,3), 1];
%%%right tool
bRe_right=pandaArm.ArmR.bTe[1:3,1:3];
bRt_right=bRe_right*eRt;
bOe_right=pandaArm.ArmR.bTe[1:3,4];
bOt_right=bOe+eOt;
bTt_right=[bRt_right, bOt_right;zeros(1,3), 1];

plt = InitDataPlot(maxloops);
pandaArms = InitRobot(model,wTb1,wTb2);
% Init object and tools frames
obj_length = 0.12;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = [rotation(0,0,0)];
pandaArms.ArmL.wTo = [rotation(0,deg2rad(30),0) [0.5 0 0.59]';zeros(1,3) 1];
pandaArms.ArmR.wTo = [rotation(0,deg2rad(30),0) [0.5 0 0.59]';zeros(1,3) 1];

theta = -44.9949;% FIXED ANGLE BETWEEN EE AND TOOL 
tool_length = 0.2124;% FIXED DISTANCE BETWEEN EE AND TOOL
% Define trasnformation matrix from ee to tool.

pandaArms.ArmL.eTt = [eRt eOt;zeros(1,3) 1];
pandaArms.ArmR.eTt =[eRt eOt;zeros(1,3) 1];

% Transformation matrix from <t> to <w>
%%Trasformation left manipulator
wRt_left=pandaArm.ArmL.wTe[1:3,1:3]*eRt;
wOt_left=pandaArm.ArmL.wTe[1:3,4]+eOt;
pandaArms.ArmL.wTt =[wRt_left wOt_left;zeros(1,3) 1];
%%Trasformation right manipulator
wRt_right=pandaArm.ArmL.wTe[1:3,1:3]*eRt;
wOt_right=pandaArm.ArmL.wTe[1:3,4]+eOt;
pandaArms.ArmR.wTt =[wRt_right wOt_right ;zeros(1,3) 1];

%% Defines the goal position for the end-effector/tool position task
% First goal reach the grasping points.
pandaArms.ArmL.wTg =  [rotation(0,deg2rad(30),0) [0.5-0.12/2 0 0.59]';zeros(1,3) 1];
pandaArms.ArmR.wTg =  [rotation(0,deg2rad(30),0) [0.5+0.12/2 0 0.59]';zeros(1,3) 1];

% Second goal move the object
pandaArms.wTog = [rotation(0,deg2rad(30),0) [0.65 -0.35 0.28]';zeros(1,3) 1];

%% Mission configuration

mission.prev_action = "go_to";
mission.current_action = "go_to";

mission.phase = 1;
mission.phase_time = 0;
% Define the active tasks for each phase of the mission
% Suggested Name for the task
% T = move tool task
% JL = joint limits task
% MA = minimum altitude task
% RC = rigid constraint task
mission.actions.go_to.tasks = [JL MA T];
mission.actions.coop_manip.tasks = [...];
mission.actions.end_motion.tasks = [...];

%% CONTROL LOOP
disp('STARTED THE SIMULATION');
for t = 0:dt:Tf
    % Receive UDP packets - DO NOT EDIT
    if real_robot == true
        dataLeft = step(hudprLeft);
        dataRight = step(hudprRight);
        % wait for data (to decide)
         if t == 0
             while(isempty(dataLeft))
                 dataLeft = step(hudprLeft);
                 pause(dt);
             end
             while(isempty(dataRight))
                 dataRight = step(hudprRight);
                 pause(dt);
             end
         end
        qL = typecast(dataLeft, 'double');
        qR = typecast(dataRight, 'double');
        pandaArms.ArmL.q = qL;
        pandaArms.ArmR.q = qR;
    end
    
    % update all the involved variables
    pandaArms = UpdateTransforms(pandaArms, mission);
    pandaArms = ComputeJacobians(pandaArms, mission);
    pandaArms = ComputeActivationFunctions(pandaArms,mission);
    pandaArms = ComputeTaskReferences(pandaArms,mission);

    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(14,1);
    Qp = eye(14);

    % Used by the Move-To task
    tool_jacobian_L = zeros(6, 7);
    tool_jacobian_R = zeros(6, 7);
    if (mission.phase == 1)
        % In this phase the tool frame coincide with the center of the
        % gripper
        tool_jacobian_L = pandaArms.ArmL.wJt;
        tool_jacobian_R = pandaArms.ArmR.wJt;
    elseif(mission.phase == 2)
        % In this phase the tool frame coincide with the object frame
        tool_jacobian_L = pandaArms.ArmL.wJo;
        tool_jacobian_R = pandaArms.ArmR.wJo;
    end

    % ADD minimum distance from table
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    
    % Bimanual system TPIK
    % ...
    % Task: Tool Move-To
    
    [Qp, ydotbar] = iCAT_task(eye(14),     eye(14),    ...
        Qp, ydotbar, zeros(14,1),  ...
        0.0001,   0.01, 10);    % this task should be the last one

    % get the two variables for integration
    pandaArms.ArmL.q_dot = ydotbar(1:7);
    pandaArms.ArmR.q_dot = ydotbar(8:14);

    pandaArms.ArmL.x = tool_jacobian_L * pandaArms.ArmL.q_dot;
    pandaArms.ArmR.x = tool_jacobian_R * pandaArms.ArmR.q_dot;
    % Integration
	pandaArms.ArmL.q = pandaArms.ArmL.q(1:7) + pandaArms.ArmL.q_dot*dt;    
    pandaArms.ArmR.q = pandaArms.ArmR.q(1:7) + pandaArms.ArmR.q_dot*dt;
    %Send udp packets [q_dot1, ..., q_dot7] DO NOT CHANGE
    if real_robot == false
        pandaArms.ArmL.q = pandaArms.ArmL.q(1:7) + pandaArms.ArmL.q_dot*dt; 
        pandaArms.ArmR.q = pandaArms.ArmR.q(1:7) + pandaArms.ArmR.q_dot*dt; 
    end
    %Send udp packets [q_dot1, ..., q_dot7]
    if real_robot == true
        step(hudpsLeft,[t;pandaArms.ArmL.q_dot]);
        step(hudpsRight,[t;pandaArms.ArmR.q_dot]);
    else 
        step(hudps,[pandaArms.ArmL.q',pandaArms.ArmR.q'])
    end
    
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + dt;
    [pandaArms,mission] = UpdateMissionPhase(pandaArms, mission);
   
    % Update data plot
    plt = UpdateDataPlot(plt,pandaArms,t,loop, mission);
    loop = loop + 1;
    % add debug prints here
    if (mod(t,0.1) == 0)
        t 
        phase = mission.phase
        if (mission.phase == 1)
            %add debug prints phase 1 here
        elseif (mission.phase == 2)
            %add debug prints phase 2 here
        end
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    % WARNING: MUST BE ENABLED IF CONTROLLING REAL ROBOT !
    SlowdownToRealtime(dt);
    
end

PrintPlot(plt, pandaArms);
end
