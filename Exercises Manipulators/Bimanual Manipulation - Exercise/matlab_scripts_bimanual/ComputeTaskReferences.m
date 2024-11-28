function [pandaArms] = ComputeTaskReferences(pandaArms,mission)
% Compute distance between tools for plotting
pandaArms.dist_tools = norm(pandaArms.ArmL.wTt(1:3, 4) - pandaArms.ArmR.wTt(1:3, 4));
% Compute minimum altitude reference ALWAYS

pandaArms.ArmL.xdot.alt = 0.2 * (0.15-pandaArms.ArmL.wTt(3,4)) ;
pandaArms.ArmR.xdot.alt = 0.2 * (0.15-pandaArms.ArmR.wTt(3,4)) ;

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
% reference for the joint limits task:
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
pandaArms.ArmL.xdot.jl = zeros(7, 1);
pandaArms.ArmR.xdot.jl = zeros(7, 1);
% Computing of velocity joint limit LEFT ARM
for i = 1:size(jlmin)
    mean(i) = (jlmin(i) + jlmax(i))/2;
    if pandaArms.ArmL.q(i) <= mean(i)
        pandaArms.ArmL.xdot.jl(i) = 0.2 * (jlmin(i) - pandaArms.ArmL.q(i) +0.01);
    else
        pandaArms.ArmL.xdot.jl(i) = 0.2 * (jlmax(i) - pandaArms.ArmL.q(i) -0.01);
    end
end
% Computing of velocity joint limit RIGHT ARM
for i = 1:size(jlmin)
    mean(i) = (jlmin(i) + jlmax(i))/2;
    if (pandaArms.ArmR.q(i) <= mean(i))
        pandaArms.ArmR.xdot.jl(i) = 0.2 * (jlmin(i) - pandaArms.ArmR.q(i) +0.01);
    else
        pandaArms.ArmR.xdot.jl(i) = 0.2 * (jlmax(i) - pandaArms.ArmR.q(i) -0.01);
    end
end
% Assembling of velocities joint limit
pandaArms.xdot.jl=[pandaArms.ArmL.xdot.jl ; pandaArms.ArmR.xdot.jl];

switch mission.phase
    case 1
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
       % [ang, lin] = CartError();
       
        %pandaArms.ArmL.xdot.tool = ...;
        
        % limit the requested velocities...
       % pandaArms.ArmL.xdot.tool(1:3) = Saturate();
        %pandaArms.ArmL.xdot.tool(4:6) = Saturate();

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
       % [ang, lin] = CartError();
       
        %pandaArms.ArmR.xdot.tool = ...;
        % limit the requested velocities...
        %pandaArms.ArmR.xdot.tool(1:3) = Saturate();
        %pandaArms.ArmR.xdot.tool(4:6) = Saturate();
    case 2
        % Perform the rigid grasp of the object and move it

        % COMMON
        % -----------------------------------------------------------------
        % Rigid Grasp Constraint
       % pandaArms.xdot.rc = ...;

        % LEFT ARM
        % -----------------------------------------------------------------        
        % Object position and orientation task reference
        %[ang, lin] = CartError();
        %pandaArms.ArmL.xdot.tool = ...;
        % limit the requested velocities...
       % pandaArms.ArmL.xdot.tool(1:3) = Saturate();
        %pandaArms.ArmL.xdot.tool(4:6) = Saturate();

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Object position and orientation task reference
        %[ang, lin] = CartError();
        %pandaArms.ArmR.xdot.tool = ...;
        % limit the requested velocities...
        %pandaArms.ArmR.xdot.tool(1:3) = Saturate();
        %pandaArms.ArmR.xdot.tool(4:6) = Saturate();
    case 3
        % Stop any motions
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
       % pandaArms.ArmL.xdot.tool(1:3) = ...;
        %pandaArms.ArmL.xdot.tool(4:6) = ...;

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
      %  pandaArms.ArmR.xdot.tool(1:3) = ...;
        %pandaArms.ArmR.xdot.tool(4:6) = ...;
end


