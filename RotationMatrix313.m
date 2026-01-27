function [DCM , DCM_Matlab ] = RotationMatrix313(attitude313)
%Contributors: Tyler Emmons
% Course number: ASEN 3801
% File name: RotationMatrix313
% Created: 1/27/26

%Inputs a 3x1 vector with Euler angles in degrees
%Outputs one DCM calculated with a 313 rotation sequence

a = attitude313(1);
b = attitude313(2);
y = attitude313(3);

R3_a = [ cosd(a)  sind(a) 0;
        -sind(a)  cosd(a) 0;
         0        0       1];

R1_b = [1 0 0;
        0 cosd(b) sind(b);
        0 -sind(b) cosd(b)];

R3_y = [ cosd(y)  sind(y) 0;
        -sind(y)  cosd(y) 0;
         0        0       1];


DCM = R3_a * R1_b * R3_y;

DCM_Matlab = angle2dcm(a, b, y, "ZXZ");

end

