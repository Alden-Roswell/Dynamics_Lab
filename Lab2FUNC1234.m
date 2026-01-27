clear;
close all;
clc;

eul = deg2rad([50, 80, 40]);
DCM = RotationMatrix321(eul);
vEC = rad2deg(EulerAngles321(DCM));

eul2 = deg2rad([10, 0, 10]);
DCM2 = RotationMatrix313(eul2);
vEC2 = rad2deg(EulerAngles313(DCM2));
vEC2




function [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename)
end

function DCM = RotationMatrix321(attitude321)
alpha = attitude321(1);
    R1 = [1,0,0;
      0, cos(alpha), sin(alpha);
      0, -sin(alpha), cos(alpha)];
    beta = attitude321(2);
    R2 = [cos(beta), 0, -sin(beta);
          0, 1, 0;
          sin(beta), 0, cos(beta)];
    gamma = attitude321(3);
    R3 = [cos(gamma), sin(gamma),0;
          -sin(gamma), cos(gamma),0;
          0, 0, 1];
    DCM = R1*R2*R3;
end

function DCM = RotationMatrix313(attitude321)
    alpha = attitude321(1);
    R3 = [cos(alpha), sin(alpha),0;
          -sin(alpha), cos(alpha),0;
          0, 0, 1];
    
    beta = attitude321(2);
    R1 = [1,0,0;
      0, cos(beta), sin(beta);
      0, -sin(beta), cos(beta)];
    
    gamma = attitude321(3);
    R3 = [cos(gamma), sin(gamma),0;
          -sin(gamma), cos(gamma),0;
          0, 0, 1];
    DCM = R3*R1*R3;
end

function attitude321 = EulerAngles321(DCM)

attitude321(1) = atan2(DCM(2,3),DCM(3,3));
attitude321(2) = -asin(DCM(1,3));
attitude321(3) = atan2(DCM(1,2),DCM(1,1));

attitude321 = attitude321';

end

function attitude313 = EulerAngles313(DCM)

attitude313(1) = atan2(DCM(3,1),-DCM(3,2));
attitude313(2) = acos(DCM(3,3));
attitude313(3) = atan2(DCM(1,3),DCM(2,3));

attitude313 = attitude313';

end