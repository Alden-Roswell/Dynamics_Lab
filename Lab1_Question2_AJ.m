
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
wind_vel = [linspace(0,20,n);linspace(0,0,n)]; % Wind velocity (m/s) 
figure();
hold on;
xend = zeros([1,n]);
normend = zeros([1,n]);
for i = 1:n
    tspan = [0,5];
    x_0 = [0,0,0,0,20,-20];
    tol = 1e-8;
    opts = odeset(RelTol=tol,AbsTol=tol);
    [t,x] = ode45(@(t,x) objectEOM(t,x,rho,Cd,A,mass,g,wind_vel(:,i)), tspan, x_0,opts);
    plot3(x(:,1), x(:,2), x(:,3))
    xend(i) = x(end,1);
    normend(i) = norm(x(end,1:3));
    hold on;
end



set(gca,'Zdir','reverse','Ydir','reverse')
axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
zlim([-20,0])
xlim([0,20])
ylim([0,75])
title('3D Trajectory of the Object');
grid on;
axis equal;
view(3);
hold off;

figure()
plot(wind_vel(1,:),xend,'LineWidth', 2)
xlim([0,20])
ylim([0,10])
xlabel("North Wind Velocity (m/s)")
ylabel("Horizontal Displacement (m)")
title("Horizontal Displacement vs Wind Velocity")

figure()
plot(wind_vel(1,:),normend,'LineWidth', 2)
xlim([0,20])
ylim([70,74])
xlabel("North Wind Velocity (m/s)")
ylabel("Horizontal Displacement (m)")
title("Total Displacement vs Wind Velocity")

%% Varying altitudes and headiwnds

n = 101;
altitudes = [0,500,1000,1500,2000];
[rho] = stdatmo(altitudes);
m = length(rho);
wind_vel = [linspace(0,0,n);linspace(-20,20,n)]; % Wind velocity (m/s)
normend = zeros([m,n]);
for j = 1:m
for i = 1:n
    j,i
    tspan = [0,5];
    x_0 = [0,0,0,0,20,-20];
    tol = 1e-8;
    opts = odeset(RelTol=tol,AbsTol=tol);
    [t,x] = ode45(@(t,x) objectEOM(t,x, rho(j) ,Cd,A,mass,g,wind_vel(:,i)), tspan, x_0,opts);

    normend(j,i) = norm(x(end,1:3));
   
end
end
figure()
hold on;
for i = 1:m
plot(wind_vel(2,:),normend(i,:),'DisplayName', altitudes(i) + "m",'LineWidth', 2)
end
legend('Location','southeast')
grid on;
xlabel("Wind Velocity (m/s)")
ylabel("Total Displacement (m)")

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
logmass = linspace(-3,2,m);
mass = 0.05 * 2.^logmass;
Energy = 0.5 * 0.05 * 2 * 20^2;
wind_vel = [linspace(0,0,n);linspace(-20,20,n)]; % Wind velocity (m/s)
normend = zeros([m,n]);
for j = 1:m
for i = 1:n
    v(j) = sqrt(2 * Energy / mass(j));
    
    tspan = [0,5];
    x_0 = [0,0,0,0, v(j)*sqrt(2)/2, -v(j)*sqrt(2)/2];
    tol = 1e-8;
    opts = odeset(RelTol=tol,AbsTol=tol);
    [t,x] = ode45(@(t,x) objectEOM(t,x, rho ,Cd,A,mass(j),g, wind_vel(:,i)), tspan, x_0, opts);
    normend(j,i) = norm(x(end,1:3));
   
end
end
v
mass
normend;
figure()
hold on;
for i = 1:m
plot(wind_vel(2,:),normend(i,:),'DisplayName', mass(i) + "kg",'LineWidth', 2)
end
legend('Location','northwest')
grid on;
xlabel("Wind Velocity (m/s)")
ylabel("Total Displacement (m)")
title("Preformance of Different Mass Projectiles")


function xdot = objectEOM(t,x,rho,Cd,A,m,g,wind_vel)
v_x = x(4);
v_y = x(5);
v_z = x(6);
v_fluid = [v_x - wind_vel(1), v_y - wind_vel(2),v_z];
speed = norm(v_fluid);
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