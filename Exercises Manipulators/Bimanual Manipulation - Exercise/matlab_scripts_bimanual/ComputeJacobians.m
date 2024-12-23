function [pandaArms] = ComputeJacobians(pandaArms,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, 
% -0.0305686, 1.53975, 0.753872] ) 
%
% remember: the control vector is:
% [q_dot] 
% [qdot_1, qdot_2, ..., qdot_7]
%
% therefore all task jacobians should be of dimensions
% m x 14top
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]

% Left Arm base to ee Jacobian
pandaArms.ArmL.bJe = geometricJacobian(pandaArms.ArmL.franka, ...
    [pandaArms.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
% Right Arm base to ee Jacobian
pandaArms.ArmR.bJe = geometricJacobian(pandaArms.ArmR.franka, ...
    [pandaArms.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

%%%%%%%%%%%%%%AGGIUNTO IO %%%%%%%%%%%%%%%%%%%%%%%%%%
pandaArms.ArmL.bJe = pandaArms.ArmL.bJe (:,1:7);
pandaArms.ArmR.bJe = pandaArms.ArmR.bJe(:,1:7);

% Ste is the rigid body transformation from end-effector frame to 
% tool-frame frame projected on <w>
pandaArms.ArmL.Ste = [eye(3) zeros(3);  skew(pandaArms.ArmL.wTe(1:3,1:3)*pandaArms.ArmL.eTt(1:3,4))' eye(3)];
pandaArms.ArmR.Ste = [eye(3) zeros(3);  skew(pandaArms.ArmR.wTe(1:3,1:3)*pandaArms.ArmR.eTt(1:3,4))' eye(3)];

% Top three rows are angular velocities, bottom three linear velocities
pandaArms.ArmL.wJt   = pandaArms.ArmL.Ste*[pandaArms.ArmL.wTb(1:3,1:3) zeros(3,3) ; zeros(3,3) pandaArms.ArmL.wTb(1:3,1:3)]*pandaArms.ArmL.bJe;
pandaArms.ArmR.wJt  =pandaArms.ArmR.Ste*[pandaArms.ArmR.wTb(1:3,1:3) zeros(3,3) ; zeros(3,3) pandaArms.ArmR.wTb(1:3,1:3)]*pandaArms.ArmR.bJe;
pandaArms.wJt = [pandaArms.ArmL.wJt pandaArms.ArmR.wJt ]

% Stw_ma is the rigid body tranformation from tool frame to its projection
% on the plane of the world
projection_tool_on_plane_L=pandaArms.ArmL.wTe(1:3,1:3)*pandaArms.ArmL.eTt(1:3,4);
projection_tool_on_plane_L(3,1)=0;
pandaArms.ArmL.Stw_ma = [eye(3) zeros(3);  -skew(projection_tool_on_plane_L) eye(3)];
projection_tool_on_plane_R=pandaArms.ArmR.wTe(1:3,1:3)*pandaArms.ArmR.eTt(1:3,4)
projection_tool_on_plane_R(3,1)=0;
pandaArms.ArmR.Stw_ma = [eye(3) zeros(3);  -skew(projection_tool_on_plane_R) eye(3)];
%
pandaArms.ArmL.Jw_w=pandaArms.ArmL.Stw_ma*pandaArms.ArmL.wJt;
pandaArms.ArmR.Jw_w=pandaArms.ArmR.Stw_ma*pandaArms.ArmR.wJt;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if (mission.phase == 2)
%     pandaArms.ArmL.wJo = ...; 
%     pandaArms.ArmR.wJo = ...;

% Common JacobianS
pandaArms.Jjl = eye(14,14);

%pandaArms.Jma = [[zeros(3,7); pandaArms.ArmL.Jw_w(4:6,1:7)] zeros(6,7); zeros(6,7) [zeros(3,7); pandaArms.ArmR.Jw_w(4:6,1:7)]];
pandaArms.Jma_L = [pandaArms.ArmL.Jw_w(6,1:7) zeros(1,7)];
pandaArms.Jma_R =[zeros(1,7) pandaArms.ArmR.Jw_w(6,1:7)];
end