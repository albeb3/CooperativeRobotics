function [pandaArm] = InitRobot(model,wTb_left,wTb_right)

%% DO NOT CHANGE FROM HERE ...
% Init two field of the main structure pandaArm containing the two robot
% model
pandaArm.ArmL = model;
pandaArm.ArmR = model;
% Init robot basic informations (q_init, transformation matrices ...)
pandaArm.ArmL.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
pandaArm.ArmR.q = pandaArm.ArmL.q;
pandaArm.ArmL.q_dot = [0 0 0 0 0 0 0]';
pandaArm.ArmR.q_dot = [0 0 0 0 0 0 0]';
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka,[pandaArm.ArmL.q',0,0],'panda_link7');
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka,[pandaArm.ArmR.q',0,0],'panda_link7');
pandaArm.ArmL.wTb = wTb_left;
pandaArm.ArmR.wTb = wTb_right;
pandaArm.ArmL.wTe = pandaArm.ArmL.wTb*pandaArm.ArmL.bTe;
pandaArm.ArmR.wTe = pandaArm.ArmR.wTb*pandaArm.ArmR.bTe;

% joint limits corresponding to the actual Panda by Franka arm configuration
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% Init relevance Jacobians
pandaArm.ArmL.bJe = eye(6,7);
pandaArm.ArmR.bJe = eye(6,7);
pandaArm.Jjl = [ ];

%% ... to HERE.
% Init robot model

wTb_left = eye(4); % fixed transformation word -> base1
wTb_right = [rotation(0,0,pi),[1.06 -0.01 0]'; 0 0 0 1]; % fixed transformation word -> base2

pandaArm.eRt=rotation(0,0, deg2rad(-44.9949));
%%% General R-L
pandaArm.eOt=[0 0 0.2104]';

%%%Left tool

pandaArm.ArmL.bRe=pandaArm.ArmL.bTe[1:3,1:3];
pandaArm.ArmL.bRt=pandaArm.ArmL.bRe*pandaArm.eRt;
pandaArm.ArmL.bOe=pandaArm.ArmL.bTe[1:3,4];
pandaArm.ArmL.bOt=pandaArm.ArmL.bOe+pandaArm.eOt;
pandaArm.ArmL.bTt=[pandaArm.ArmL.bRt, pandaArm.ArmL.bOt;zeros(1,3), 1];
%%%right tool
pandaArm.ArmR.bRe=pandaArm.ArmR.bTe[1:3,1:3];
pandaArm.ArmR.bRt=pandaArm.ArmR.bRe*pandaArm.eRt;
pandaArm.ArmR.bOe=pandaArm.ArmR.bTe[1:3,4];
pandaArm.ArmR.bOt=pandaArm.ArmR.bOe+pandaArm.eOt;
pandaArm.ArmR.bTt=[pandaArm.ArmR.bRt, pandaArm.ArmR.bOt;zeros(1,3), 1];
% Define trasnformation matrix from ee to tool.
pandaArms.ArmL.eTt = [eRt eOt;zeros(1,3) 1];
pandaArms.ArmR.eTt =[eRt eOt;zeros(1,3) 1];
% Transformation matrix from <t> to <w>
%%Trasformation left manipulator
pandaArms.ArmL.wRt=pandaArm.ArmL.wTe[1:3,1:3]*pandaArm.eRt;
pandaArms.ArmL.wOt=pandaArm.ArmL.wTe[1:3,4]+pandaArm.eOt;
pandaArms.ArmL.wTt =[pandaArms.ArmL.wRt pandaArms.ArmL.wOt;zeros(1,3) 1];
%%Trasformation right manipulator
pandaArms.ArmR.wRt=pandaArm.ArmR.wTe[1:3,1:3]*pandaArm.eRt;
pandaArms.ArmR.wOt=pandaArm.ArmR.wTe[1:3,4]+pandaArm.eOt;
pandaArms.ArmR.wTt=[pandaArms.ArmR.wRt pandaArms.ArmR.wOt ;zeros(1,3) 1];

%% ... TO HERE
% Init Task Reference vectors

% Init Activation function for activate or deactivate tasks




end

