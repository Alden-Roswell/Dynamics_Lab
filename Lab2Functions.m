clear;
close all;
clc;

eul = deg2rad([50, 80, 40])';
DCM = RotationMatrix321(eul);
vEC = rad2deg(EulerAngles321(DCM));

eul2 = deg2rad([14, 2, 5])';
DCM2 = RotationMatrix313(eul2);
vEC2 = rad2deg(EulerAngles313(DCM2));
vEC2;

[t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData("3801_Sec001_Test1.csv");

%% Question 3
% On the same figure, plot the 3D position of both objects in frame 𝑁𝑁; draw the aerospace vehicle's path
% in solid blue and the target's path in dashed red. Label all axes and include a legend. Be sure to follow
% the plotting best practices presented in Lab 1.
figure()
hold on;
grid on;
set(gca,'Zdir','reverse','Ydir','reverse')
scatter3(av_pos_inert(1,:),av_pos_inert(2,:),av_pos_inert(3,:))
  % tar_pos_inert(1,:),tar_pos_inert(2,:),tar_pos_inert(3,:),color = "blue",color = "red")
scatter3(tar_pos_inert(1,:),tar_pos_inert(2,:),tar_pos_inert(3,:), color = "blue")

xlabel("X (mm)");
ylabel("Y (mm)");
zlabel("Z (mm)");
axis equal;
legend("Aircraft", "Target")
title("Position of Aircraft and Target")


%% Question 4
% Create two figures. On the first figure, create three subplots with each subplot displaying one
% component of the position vector in frame 𝐸𝐸 as a function of time for each object (aerospace vehicle in
% blue, target in red). On the second figure, create three subplots with each subplot displaying one of the
% 3-2-1 Euler angles (in degrees) as a function of time for each object relative to frame 𝐸𝐸.
figure()
hold on;
subplot(3,1,1)
plot(t_vec, av_pos_inert(1,:),t_vec, tar_pos_inert(1,:), "LineWidth", 2)
ylabel("X")
grid on;
title("Position of Aircraft and Target Over Time")
subplot(3,1,2)
plot(t_vec, av_pos_inert(2,:),t_vec, tar_pos_inert(2,:), "LineWidth", 2)
ylabel("Y")
grid on;
subplot(3,1,3)
plot(t_vec, av_pos_inert(3,:),t_vec, tar_pos_inert(3,:), "LineWidth", 2)
ylabel("Z")
xlabel("time (s)")
grid on;
legend("Aircraft", "Target")



figure()
hold on;
subplot(3,1,1)
plot(t_vec, av_att(1,:),t_vec, tar_att(1,:), "LineWidth", 2)
ylabel("\alpha")
grid on;
title("Attitude of Aircraft and Target")
subplot(3,1,2)
plot(t_vec, av_att(2,:),t_vec, tar_att(2,:), "LineWidth", 2)
ylabel("\beta")
grid on;
subplot(3,1,3)
plot(t_vec, av_att(3,:),t_vec, tar_att(3,:), "LineWidth", 2)
ylabel("\gamma")
xlabel("time (s)")
grid on;
legend("Aircraft", "Target", "Location","best")

%% Question 5
% For both objects, calculate the 3-1-3 Euler angles that describe the orientation of the object relative to
% frame 𝐸𝐸 over time. To calculate the 3-1-3 Euler angles (of either the aerospace vehicle or target), first use
% the 3-2-1 Euler angles to calculate the direction cosine matrix (DCM) that describes the body-fixed frame
% (of either the aerospace vehicle or target) relative to frame 𝐸𝐸. Then, determine the 3-1-3 Euler angles (of
% either the aerospace vehicle or target) from the associated DCM. After these calculations, create a figure
% with three subplots. Each subplot displays each Euler angle (in degrees) as a function of time for the two
% objects (aerospace vehicle in blue, target in red).
av_att313 = zeros(size(av_att));
tar_att313 = zeros(size(av_att));
for i = 1:length(av_att)
av_att313(:,i) =  EulerAngles313(RotationMatrix321(av_att(:,i)));
tar_att313(:,i) = EulerAngles313(RotationMatrix321(tar_att(:,i)));
end

figure()
hold on;
subplot(3,1,1)
plot(t_vec, av_att313(1,:),t_vec, tar_att313(1,:), "LineWidth", 2)
ylabel("\alpha")
grid on;
title("Attitude of Aircraft and Target 313 Euler Angles")
subplot(3,1,2)
plot(t_vec, av_att313(2,:),t_vec, tar_att313(2,:), "LineWidth", 2)
ylabel("\beta")
grid on;
subplot(3,1,3)
plot(t_vec, av_att313(3,:),t_vec, tar_att313(3,:), "LineWidth", 2)
ylabel("\gamma")
xlabel("time (s)")
grid on;
legend("Aircraft", "Target", "Location","best")


%% Question 6
% Calculate the position vector of the target relative to the aerospace vehicle, expressed in the axes of
% Frame 𝐸𝐸. Plot each component of the relative position vector as a function of time as a subplot on a
% single figure.
tar_pos_rel = tar_pos_inert - av_pos_inert;
figure()
hold on;
subplot(3,1,1)
plot(t_vec, tar_pos_rel(1,:), "LineWidth", 2)
ylabel("X")
grid on;
title("Position of Aircraft and Target Over Time")
subplot(3,1,2)
plot(t_vec, tar_pos_rel(2,:), "LineWidth", 2)
ylabel("Y")
grid on;
subplot(3,1,3)
plot(t_vec, tar_pos_rel(3,:), "LineWidth", 2)
ylabel("Z")
xlabel("time (s)")
grid on;
legend("Relative Position")



%% Question 7
% Determine the position vector of the target relative to the aerospace vehicle, expressed in the body
% coordinates of the aerospace vehicle. In other words, calculate the position vector of the target in Frame
% 𝐵𝐵. Plot each component of the relative position vector as a function of time as a subplot in a single figure.

function [t_vec, av_pos_inert, av_att, tar_pos_inert, tar_att] = LoadASPENData(filename)
dat = readmatrix(filename, Range = 1);
dat = rmmissing(dat,1); 

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

function [DCM] = RotationMatrix321(attitude321)
alpha = attitude321(1,:);
    R1 = [1,0,0;
      0, cos(alpha), sin(alpha);
      0, -sin(alpha), cos(alpha)];
    beta = attitude321(2);
    R2 = [cos(beta), 0, -sin(beta);
          0, 1, 0;
          sin(beta), 0, cos(beta)];
    gamma = attitude321(3,:);
    R3 = [cos(gamma), sin(gamma),0;
          -sin(gamma), cos(gamma),0;
          0, 0, 1];
    DCM = R1*R2*R3;
end

function [DCM] = RotationMatrix313(attitude321)
    alpha = attitude321(1,:);
    R1 = [cos(alpha), sin(alpha),0;
          -sin(alpha), cos(alpha),0;
          0, 0, 1];
    
    beta = attitude321(2,:);
    R2 = [1,0,0;
      0, cos(beta), sin(beta);
      0, -sin(beta), cos(beta)];
    
    gamma = attitude321(3,:);
    R3 = [cos(gamma), sin(gamma),0;
          -sin(gamma), cos(gamma),0;
          0, 0, 1];
    DCM = R3*R2*R1;
end

function [attitude321] = EulerAngles321(DCM)

attitude321(1,:) = atan2(DCM(2,3),DCM(3,3));
attitude321(2,:) = -asin(DCM(1,3));
attitude321(3,:) = atan2(DCM(1,2),DCM(1,1));

attitude321 = attitude321';

end

function [attitude313] = EulerAngles313(DCM)

attitude313(1,:) = atan2(DCM(3,1),-DCM(3,2));
attitude313(2,:) = acos(DCM(3,3));
attitude313(3,:) = atan2(DCM(1,3),DCM(2,3));

attitude313 = attitude313';

end