clear;
close all;
clc;

eul = deg2rad([50, 80, 40]);
DCM = RotationMatrix321(eul);
vEC = rad2deg(EulerAngles321(DCM));

eul2 = deg2rad([14, 2, 5]);
DCM2 = RotationMatrix313(eul2);
vEC2 = rad2deg(EulerAngles313(DCM2));
vEC2;

[t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData("3801_Sec001_Test1.csv");

figure()
hold on;
set(gca,'Zdir','reverse','Ydir','reverse')
scatter3(av_pos_inert(1,:),av_pos_inert(2,:),av_pos_inert(3,:))
  % tar_pos_inert(1,:),tar_pos_inert(2,:),tar_pos_inert(3,:),color = "blue",color = "red")
scatter3(tar_pos_inert(1,:),tar_pos_inert(2,:),tar_pos_inert(3,:), color = "blue")

xlabel("X");
ylabel("Y");
zlabel("Z");
axis equal;


function [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename)
dat = readmatrix(filename, Range = 1);
%dat = rmmissing(dat,1); 

%Main Vector
framerate = dat(2,1)
dat(:,2) = [];
dat(1:5,:) = [];
t_vec = dat(:,1)' ./framerate;

pos_av_aspen = dat(:,11:13)';

att_av_aspen = dat(:,8:10)';

pos_tar_aspen = dat(:,5:7)';

att_tar_aspen = dat(:,2:4)';


[av_pos_inert, av_att, tar_pos_inert, tar_att] =  ...
ConvertASPENData(pos_av_aspen, att_av_aspen,  pos_tar_aspen, att_tar_aspen);

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
    R1 = [cos(alpha), sin(alpha),0;
          -sin(alpha), cos(alpha),0;
          0, 0, 1];
    
    beta = attitude321(2);
    R2 = [1,0,0;
      0, cos(beta), sin(beta);
      0, -sin(beta), cos(beta)];
    
    gamma = attitude321(3);
    R3 = [cos(gamma), sin(gamma),0;
          -sin(gamma), cos(gamma),0;
          0, 0, 1];
    DCM = R3*R2*R1;
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