function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        activationfunctionstool=eye(2)
        [err_rho_L,err_pos_L]=CartError(pandaArms.ArmL.wTg,pandaArms.ArmL.wTt);
         activationfunctionstool(1,1)=DecreasingBellShapedFunction(0.001,0.1,0,1, norm(err_pos_L) );
         [err_rho_R,err_pos_R]=CartError(pandaArms.ArmR.wTg,pandaArms.ArmR.wTt);
         activationfunctionstool(2,2)=DecreasingBellShapedFunction(0.001,0.1,0,1, norm(err_pos_R) );
         pandaArm.A.tool = activationfunctionstool .* ActionTransition("T",.... , ....., mission.phase_time);
    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        
        % Move-To
        
    case 3 % STOP any motion 
        
end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
activationfunctions_ma=eye(2);
dist_maL=0.15-pandaArm.ArmL.wTt(3,4);
activationfunctions_ma(1,1)= IncreasingBellShapedFunction(0.001 , 0.1,0,1, dist_maL);
dist_maR=0.15-pandaArm.ArmR.wTt(3,4);
activationfunctions_ma(2,2)= IncreasingBellShapedFunction(0.001 , 0.1,0,1, dist_maR);
pandaArm.A.ma = activationfunctions_ma .* ActionTransition("MA",.... , ....., mission.phase_time);

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
activationfunctionsjl=eye(14)
for i=1:size(jlim)
    activationfunctionsjl(i,i) = IncreasingBellShapedFunction(jlmax(i)-0.1,jlmax(i),0,1, pandaArms.ArmL.q(i) )+ decreasingBellShapedFunction(jlmin(i)-0.1,jlmin(i),0,1, pandaArms.ArmL.q(i) );
    activationfunctionsjl(i+7,i+7) = IncreasingBellShapedFunction(jlmax(i)-0.1,jlmax(i),0,1, pandaArms.ArmR.q(i))+decreasingBellShapedFunction(jlmin(i)-0.1,jlmin(i),0,1, pandaArms.ArmR.q(i) ); 
end
pandaArm.A.jl = activationfunctionsjl .* ActionTransition("JL",.... , ....., mission.phase_time);

end
