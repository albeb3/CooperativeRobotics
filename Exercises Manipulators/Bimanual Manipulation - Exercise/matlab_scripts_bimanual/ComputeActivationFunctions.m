function [pandaArms] = ComputeActivationFunctions(pandaArms, mission)

if mission.phase == 1
    prev_action = mission.actions.go_to.tasks;
    current_action = mission.actions.go_to.tasks;
elseif mission.phase == 2
    prev_action = mission.actions.go_to.tasks;
    current_action = mission.actions.coop_manip.tasks;
end

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
        activationfunctionstool_L=eye(6);
        
         err_rho_L,err_pos_L]=CartError(pandaArms.ArmL.wTg,pandaArms.ArmL.wTt);
         for i=1:6
            activationfunctionstool(i,i)=DecreasingBellShapedFunction(0.001,0.1,0,1, norm(err_pos_L) );
         end 
         [err_rho_R,err_pos_R]=CartError(pandaArms.ArmR.wTg,pandaArms.ArmR.wTt);
         for i=1:6
            activationfunctionstool(i,i)=DecreasingBellShapedFunction(0.001,0.1,0,1, norm(err_pos_R) );
         end    
         pandaArms.A.tool = activationfunctionstool .* ActionTransition("T",prev_action , current_action, mission.phase_time);
         
    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        
        % Move-To
        
    case 3 % STOP any motion 
        
end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )

dist_maL=0.15-pandaArms.ArmL.wTt(3,4);
activationfunctions_ma_L= IncreasingBellShapedFunction(0.001 , 0.1,0,1, dist_maL);
pandaArms.A.ma_L = activationfunctions_ma_L .* ActionTransition("MA",prev_action , current_action, mission.phase_time)

dist_maR=0.15-pandaArms.ArmR.wTt(3,4);
activationfunctions_ma_R= IncreasingBellShapedFunction(0.001 , 0.1,0,1, dist_maR);
pandaArms.A.ma_R = activationfunctions_ma_R .* ActionTransition("MA",prev_action , current_action, mission.phase_time)

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
activationfunctionsjl=eye(14);
for i=1:size(jlmin,1)
    activationfunctionsjl(i,i) = IncreasingBellShapedFunction(jlmax(i)-0.1,jlmax(i),0,1, pandaArms.ArmL.q(i) )+ DecreasingBellShapedFunction(jlmin(i)-0.1,jlmin(i),0,1, pandaArms.ArmL.q(i) );
    activationfunctionsjl(i+7,i+7) = IncreasingBellShapedFunction(jlmax(i)-0.1,jlmax(i),0,1, pandaArms.ArmR.q(i))+DecreasingBellShapedFunction(jlmin(i)-0.1,jlmin(i),0,1, pandaArms.ArmR.q(i) ); 
end
pandaArms.A.jl = activationfunctionsjl .* ActionTransition("JL",prev_action , current_action, mission.phase_time);

end