function [pandaArms] = InitRobot(model,wTb_left,wTb_right)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArms containing the two robot
% model
pandaArms.ArmL = model;
pandaArms.ArmR = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArms.ArmL.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArms.ArmR.q = pandaArms.ArmL.q;
pandaArms.ArmL.q_dot = [0 0 0 0 0 0 0]';
pandaArms.ArmR.q_dot = [0 0 0 0 0 0 0]';
pandaArms.ArmL.bTe = getTransform(pandaArms.ArmL.franka,[pandaArms.ArmL.q',0,0],'panda_link7');
pandaArms.ArmR.bTe = getTransform(pandaArms.ArmR.franka,[pandaArms.ArmR.q',0,0],'panda_link7');
pandaArms.ArmL.wTb = wTb_left;
pandaArms.ArmR.wTb = wTb_right;
pandaArms.ArmL.wTe = pandaArms.ArmL.wTb*pandaArms.ArmL.bTe;
pandaArms.ArmR.wTe = pandaArms.ArmR.wTb*pandaArms.ArmR.bTe;

% joint limits corresponding to the actual Panda by Franka arm configuration
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% Init relevance Jacobians
pandaArms.ArmL.bJe = eye(6,7);
pandaArms.ArmR.bJe = eye(6,7);
pandaArms.Jjl = [ ];

%% ... to HERE.
% Init robot model

% wTb_left = eye(4); % fixed transformation word -> base1
% wTb_right = [rotation(0,0,pi),[1.06 -0.01 0]'; 0 0 0 1]; % fixed transformation word -> base2

pandaArms.eRt=rotation(0,0, deg2rad(-44.9949));
%%% General R-L
pandaArms.eOt=pandaArms.eRt*[0 0 0.2104]'; % tool vector position projected on world-frame

%%%Left tool

pandaArms.ArmL.bRe=pandaArms.ArmL.bTe(1:3,1:3);
pandaArms.ArmL.bRt=pandaArms.ArmL.bRe*pandaArms.eRt;
pandaArms.ArmL.bOe=pandaArms.ArmL.bTe(1:3,4);
pandaArms.ArmL.bOt=pandaArms.ArmL.bOe+pandaArms.eOt;
pandaArms.ArmL.bTt=[pandaArms.ArmL.bRt, pandaArms.ArmL.bOt;zeros(1,3), 1];
%%%right tool
pandaArms.ArmR.bRe=pandaArms.ArmR.bTe(1:3,1:3);
pandaArms.ArmR.bRt=pandaArms.ArmR.bRe*pandaArms.eRt;
pandaArms.ArmR.bOe=pandaArms.ArmR.bTe(1:3,4);
pandaArms.ArmR.bOt=pandaArms.ArmR.bOe+pandaArms.eOt;
pandaArms.ArmR.bTt=[pandaArms.ArmR.bRt, pandaArms.ArmR.bOt;zeros(1,3), 1];
% Define trasnformation matrix from ee to tool.
pandaArms.ArmL.eTt = [pandaArms.eRt pandaArms.eOt;zeros(1,3) 1];
pandaArms.ArmR.eTt =[pandaArms.eRt pandaArms.eOt;zeros(1,3) 1];
% Transformation matrix from <t> to <w>
%%Trasformation left manipulator
pandaArms.ArmL.wRt=pandaArms.ArmL.wTe(1:3,1:3)*pandaArms.eRt;
pandaArms.ArmL.wOt=pandaArms.ArmL.wTe(1:3,4)+pandaArms.eOt;
pandaArms.ArmL.wTt =[pandaArms.ArmL.wRt pandaArms.ArmL.wOt;zeros(1,3) 1];
%%Trasformation right manipulator
pandaArms.ArmR.wRt=pandaArms.ArmR.wTe(1:3,1:3)*pandaArms.eRt;
pandaArms.ArmR.wOt=pandaArms.ArmR.wTe(1:3,4)+pandaArms.eOt;
pandaArms.ArmR.wTt=[pandaArms.ArmR.wRt pandaArms.ArmR.wOt ;zeros(1,3) 1];

%% ... TO HERE
% Init Task Reference vectors

% Init Activation function for activate or deactivate tasks




end

