function [pandaArm] = UpdateTransforms(pandaArm, mission)
% the function updates all the transformations

% Left arm transformations
pandaArm.ArmL.bTe = getTransform(pandaArm.ArmL.franka, ...
    [pandaArm.ArmL.q',0,0],'panda_link7');%DO NOT EDIT

% Right arm transformations
pandaArm.ArmR.bTe = getTransform(pandaArm.ArmR.franka, ...
    [pandaArm.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

% <e> to <w>
pandaArm.ArmL.wTe = pandaArm.ArmL.bTe;
pandaArm.ArmR.wTe = [rotation(0,0,pi),[1.06 -0.01 0]'; 0 0 0 1]*pandaArm.ArmR.bTe;

% Transformation matrix from <t> to <w>
pandaArm.ArmL.wTt = pandaArm.ArmL.wTe*pandaArm.ArmL.eTt;
pandaArm.ArmR.wTt = pandaArm.ArmR.wTe*pandaArm.ArmR.eTt;

% <o> to <w> : ASSUME <t> = <g> during entire cooperation phase
% if (mission.phase == 2)
%     pandaArm.ArmL.wTo = ...; 
%     pandaArm.ArmR.wTo = ...;
% end
