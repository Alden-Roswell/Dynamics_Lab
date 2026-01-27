function attitude313 = EulerAngles313(DCM)
%input is the DCM created in the main file, initialize the values inside
%that we need from the matrix

r13 = DCM(1,3); %row 1 column 3
r23 = DCM(2,3); %row 2 column 3
r31 = DCM(3,1); %row 3 column 1
r32 = DCM(3,2); %row 3 column 2
r33 = DCM(3,3); %row 3 column 3

%attitude313: 3 x 1 vector with the 3-1-3 Euler angles in the form attitude313 = [roll, pitch, yaw]T
yaw = atan((r31)/(-r32)); %calculate yaw angle of rotation given DCM
pitch = acos(r33); %calculate pitch angle of rotation given DCM
roll = atan(r13/r23); %calculate roll angle of rotation given DCM
attitude313 = [roll; pitch; yaw];

end
