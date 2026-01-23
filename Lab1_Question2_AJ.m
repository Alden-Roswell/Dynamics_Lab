
% Contributors:  AJ, Harvey
% Course number: ASEN 3801
% File name: Lab1_Question2_AJ
% Created: 1/13/26

clear;
close all;
clc;

%A = 1;
%[t,y] = ode45(@(t,y) odefcn(t,y,A), tspan, y0)


% Define parameters for the object
[rho] = stdatmo(1655); %% altitude of boulder
Cd = 0.6; % Drag coefficient
D = 0.02; % Diameter (m)
A = pi*(D/2)^2;  % Cross-sectional area (m^2)
mass = 0.05;  % Mass (kg)
g = 9.81; % Acceleration due to gravity (m/s^2)
n = 6;

%% Varying sidewind
%% vary wind velocity from 0-50 to observe its effects
wind_vel = [linspace(0,50,n);linspace(0,0,n)]; % Wind velocity (m/s) 
figure();
hold on;
xend = zeros([1,n]);
normend = zeros([1,n]);
for i = 1:n
    %iterate through and compare each wind case
    tspan = [0,5];
    x_0 = [0,0,0,0,20,-20];
    tol = 1e-8;
    opts = odeset(RelTol=tol,AbsTol=tol);
    [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,mass,g,wind_vel(:,i)), tspan, x_0,opts);
    plot3(x(:,1), x(:,2), x(:,3),'DisplayName', wind_vel(1,i) + "m/s")
    xend(i) = x(end,1);
    normend(i) = norm(x(end,1:3));
    hold on;
end

%Plot 3d graph of ball flight path

set(gca,'Zdir','reverse','Ydir','reverse')
axis equal;
legend()
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
zlim([-20,0])
xlim([0,50])
ylim([0,75])
title('3D Trajectory of the Object');
grid on;
axis equal;
view(3);
hold off;

% Plot effects of sidewind on horizontal displacement
figure()
plot(wind_vel(1,:),xend,'LineWidth', 2)
xlim([0,50])
ylim([0,35])
xlabel("North Wind Velocity (m/s)")
ylabel("Horizontal Displacement (m)")
title("Horizontal Displacement vs Wind Velocity")
grid on;

% Plot effects of sidewind on total displacement
figure()
plot(wind_vel(1,:),normend,'LineWidth', 2)
xlim([0,50])
ylim([68,74])
xlabel("North Wind Velocity (m/s)")
ylabel("Total Displacement (m)")
title("Total Displacement vs Wind Velocity")
grid on;

%% Varying altitudes and headiwnds

n = 101;
altitudes = [0,500,1000,1500,2000];
[rho] = stdatmo(altitudes);
m = length(rho);
wind_vel = [linspace(0,50,n);linspace(0,0,n)]; % Wind velocity (m/s)
normend = zeros([m,n]);
%iterate in across different altitudes and wind speeds to see how the two
%variables effect the total displacement of the ball
for j = 1:m
for i = 1:n
    j;i;
    tspan = [0,5];
    x_0 = [0,0,0,0,20,-20];
    tol = 1e-8;
    opts = odeset(RelTol=tol,AbsTol=tol);
    [t,x] = ode45(@(t,x) objectEOM(t,x, rho(j) ,Cd,A,mass,g,wind_vel(:,i)), tspan, x_0,opts);

    normend(j,i) = norm(x(end,1:3));
   
end
end
%Plot of total displacement based on altitude and wind velocity
figure()
hold on;
for i = 1:m
plot(wind_vel(1,:),normend(i,:),'DisplayName', altitudes(i) + "m",'LineWidth', 2)
end
legend('Location','southeast')
grid on;
xlabel("Wind Velocity (m/s)")
ylabel("Total Displacement (m)")
title("Distance vs Wind Velocity at Different Altitudes")

%Plot of total displacement vs altitude
figure()
plot(altitudes,normend(:,1),'LineWidth', 2)
grid on;
xlabel("Geopotential Altitude (m)")
ylabel("Total Distance (m)")
title("Distance vs Altitude With 20 m/s Headwind")


%% Varying Mass 
n = 101;
[rho] = stdatmo(1655); %% altitude of boulder
Cd = 0.6; % Drag coefficient
D = 0.02; % Diameter (m)
A = pi*(D/2)^2;  % Cross-sectional area (m^2)
g = 9.81; % Acceleration due to gravity (m/s^2)
m = 6;
%create vector mass values that increase logarithmically
logmass = linspace(-3,2,m);
mass = 0.05 * 2.^logmass;
%define baseline energy value
Energy = 0.5 * 0.05 * 2 * 20^2;

wind_vel = [linspace(0,50,n);linspace(0,0,n)]; % Wind velocity (m/s)
normend = zeros([m,n]);
for j = 1:m
for i = 1:n
    %declare velocity based on baseline energy value
    v(j) = sqrt(2 * Energy / mass(j));
    
    tspan = [0,5];
    x_0 = [0,0,0,0, v(j)*sqrt(2)/2, -v(j)*sqrt(2)/2];
    tol = 1e-8;
    opts = odeset(RelTol=tol,AbsTol=tol);
    [t,x] = ode45(@(t,x) objectEOM(t,x, rho ,Cd,A,mass(j),g, wind_vel(:,i)), tspan, x_0, opts);
    normend(j,i) = norm(x(end,1:3));
   
end
end
%% Plot effects of different mass and wind velocity  on total displacement 
figure()
hold on;
for i = 1:m
plot(wind_vel(1,:),normend(i,:),'DisplayName', mass(i) + "kg",'LineWidth', 2)
end
legend('Location','northwest')
grid on;
xlabel("Wind Velocity (m/s)")
ylabel("Total Displacement (m)")
title("Preformance of Different Mass Projectiles")


function xdot = objectEOM(t,x,rho,Cd,A,m,g,wind_vel)

%%INPUTS: t = time value
%         x = state vector containing position and velocity for x y and z
% 
% OUTPUTS: dydt = rate of change of state vector, velocity and accleration
%in x y and z
%                 with dydt(1) = dwdt
%                 dydt(2) = dxdt
%                 dydt(3) = dydt
%                 dydt(4) = dzdt
% METHODOLOGY: define the projectile motion of a ball under aerodynamics
% forces in 3 dimensions
%                
% set velocity in state vector to rate of change of position.
v_x = x(4); 
v_y = x(5);
v_z = x(6);
%define relative fluid velocity for drag calculation
v_fluid = [v_x - wind_vel(1), v_y - wind_vel(2),v_z];
speed = norm(v_fluid);
%multiply drag force by unit vector of relative wind.
FDrag = - 0.5 * rho * Cd * A * speed^2 * (v_fluid/speed);
a_x = FDrag(1)/m; % Acceleration in x direction
a_y = FDrag(2)/m; % Acceleration in x direction
a_z = FDrag(3)/m + g; % Acceleration in y direction
if x(3) > 0;
    xdot = [0;0;0;0;0;0];
else% Set z position to zero if below ground level

xdot = [v_x; v_y; v_z; a_x; a_y; a_z];
end
end