
%A = 1;
%[t,y] = ode45(@(t,y) odefcn(t,y,A), tspan, y0)

[rho] = stdatmo(1655); %% altitude of boulder
% Define parameters for the object
Cd = 0.6; % Drag coefficient
D = 0.02; % Diameter (m)
A = pi*(D/2)^2;  % Cross-sectional area (m^2)
m = 0.05;  % Mass (kg)
g = 9.81; % Acceleration due to gravity (m/s^2)
wind_vel = [0,0]; % Wind velocity (m/s)

tspan = [0,5];
x_0 = [0,0,0,0,20,20];
tol = 1e-8;
opts = odeset(RelTol=tol,AbsTol=tol);
[t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,m,g,wind_vel), tspan, x_0,opts);
figure();
axis equal;
plot(x(:,2), x(:,3))

function xdot = objectEOM(t,x,rho,Cd,A,m,g,wind_vel)
v_x = x(4);
v_y = x(5);
v_z = x(6);
v_fluid = [v_x - wind_vel(1), v_y - wind_vel(2),v_z];
speed = norm(v_fluid);
FDrag = - 0.5 * rho * Cd * A * speed^2 * (v_fluid/speed);
a_x = FDrag(1)/m; % Acceleration in x direction
a_y = FDrag(2)/m; % Acceleration in x direction
a_z = FDrag(3)/m - g; % Acceleration in y direction


xdot = [v_x; v_y; v_z; a_x; a_y; a_z];
end