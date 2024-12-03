function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
switch mission.phase
            case 1  %Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                [err_rho_L, err_pos_L] =CartError(pandaArm.ArmL.wTg,pandaArm.ArmL.wTt);
                [err_rho_R,err_pos_R] =CartError(pandaArm.ArmR.wTg,pandaArm.ArmR.wTt);
                % max error: 1/10 cm and 1deg
                if (norm(err_pos_R)<0.1 && err_rho_R<deg2rad(1) && norm(err_pos_L)<0.1 && err_rho_L<deg2rad(1)) 
                    mission.phase = 2;
                end
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task

                % max error: 1 cm and 3deg
               
            case 3 % Finish motion
                
        end
end

