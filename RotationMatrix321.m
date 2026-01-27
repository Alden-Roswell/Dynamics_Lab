function DCM = RotationMatrix321(attitude321)
g = attitude321(3); %psi
b = attitude321(2); %theta
a = attitude321(1); %phi

R_x = @(theta_x) [1, 0, 0;
    0, cos(theta_x),sin(theta_x);
    0,-sin(theta_x),cos(theta_x)];
R_y = @(theta_y) [cos(theta_y),0,-sin(theta_y);
    0,1,0;
    sin(theta_y),0,cos(theta_y)];
R_z = @(theta_z) [cos(theta_z), sin(theta_z), 0;
    -sin(theta_z), cos(theta_z), 0;
    0, 0, 1];

DCM = R_x(a)*R_y(b)*R_z(g);
end