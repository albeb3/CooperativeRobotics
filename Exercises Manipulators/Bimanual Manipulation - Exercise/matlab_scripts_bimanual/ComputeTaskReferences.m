function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Compute distance between tools for plotting
pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));
% Compute minimum altitude reference ALWAYS

pandaArm.ArmL.xdot.alt = 0.2 * (0.15-pandaArm.ArmL.wTt(3,4)) ;
pandaArm.ArmR.xdot.alt = 0.2 * (0.15-pandaArm.ArmR.wTt(3,4)) ;
pandaArm.xdot.alt =[pandaArm.ArmL.xdot.alt ; pandaArm.ArmR.xdot.alt ];
% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
% reference for the joint limits task:
jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
pandaArm.ArmL.xdot.jl = zeros(7, 1);
pandaArm.ArmR.xdot.jl = zeros(7, 1);
% Computing of velocity joint limit LEFT ARM
for i = 1:size(jlmin)
    mean(i) = (jlmin(i) + jlmax(i))/2;
    if pandaArm.ArmL.q(i) <= mean(i)
        pandaArm.ArmL.xdot.jl(i) = 0.2 * (jlmin(i) - pandaArm.ArmL.q(i) +0.01);
    else
        pandaArm.ArmL.xdot.jl(i) = 0.2 * (jlmax(i) - pandaArm.ArmL.q(i) -0.01);
    end
end
% Computing of velocity joint limit RIGHT ARM
for i = 1:size(jlmin)
    mean(i) = (jlmin(i) + jlmax(i))/2;
    if pandaArm.ArmR.(i) <= mean(i)
        pandaArm.ArmR.xdot.jl(i) = 0.2 * (jlmin(i) - pandaArm.ArmR.q(i) +0.01);
    else
        pandaArm.ArmR.xdot.jl(i) = 0.2 * (jlmax(i) - pandaArm.ArmR.q(i) -0.01);
    end
end
% Assembling of velocities joint limit
pandaArm.xdot.jl=[pandaArm.ArmL.xdot.jl ; pandaArm.ArmR.xdot.jl];

switch mission.phase
    case 1
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang, lin] = CartError();
       
        pandaArm.ArmL.xdot.tool = ...;
        % limit the requested velocities...
        pandaArm.ArmL.xdot.tool(1:3) = Saturate();
        pandaArm.ArmL.xdot.tool(4:6) = Saturate();

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang, lin] = CartError();
       
        pandaArm.ArmR.xdot.tool = ...;
        % limit the requested velocities...
        pandaArm.ArmR.xdot.tool(1:3) = Saturate();
        pandaArm.ArmR.xdot.tool(4:6) = Saturate();
    case 2
        % Perform the rigid grasp of the object and move it

        % COMMON
        % -----------------------------------------------------------------
        % Rigid Grasp Constraint
        pandaArm.xdot.rc = ...;

        % LEFT ARM
        % -----------------------------------------------------------------        
        % Object position and orientation task reference
        [ang, lin] = CartError();
        pandaArm.ArmL.xdot.tool = ...;
        % limit the requested velocities...
        pandaArm.ArmL.xdot.tool(1:3) = Saturate();
        pandaArm.ArmL.xdot.tool(4:6) = Saturate();

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Object position and orientation task reference
        [ang, lin] = CartError();
        pandaArm.ArmR.xdot.tool = ...;
        % limit the requested velocities...
        pandaArm.ArmR.xdot.tool(1:3) = Saturate();
        pandaArm.ArmR.xdot.tool(4:6) = Saturate();
    case 3
        % Stop any motions
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.ArmL.xdot.tool(1:3) = ...;
        pandaArm.ArmL.xdot.tool(4:6) = ...;

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.ArmR.xdot.tool(1:3) = ...;
        pandaArm.ArmR.xdot.tool(4:6) = ...;
end


