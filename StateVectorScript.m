% Contributors: Harvey, AJ
% Course number: ASEN 3801
% File name: StateVectorScript
% Created: 1/13/26

clc;clear;
% Set initial condition vector and time interval
% Order for vector is [w,x,y,z] which are synonymous with 1,2,3,4
initial_condition = [8,2,2,6];
tspan = [0, 20];

function dydt = ODEFUN(t,y)
% 
% INPUTS: t = time value
%         y = state vector containing [w,x,y,z]'
% 
% OUTPUTS: dydt = represents the system of ODEs
%                 with dydt(1) = dwdt
%                 dydt(2) = dxdt
%                 dydt(3) = dydt
%                 dydt(4) = dzdt
% METHODOLOGY: Define the system of ODEs to pass into ode45
%                 using the [w,x,y,z] -> [1,2,3,4] definition
%
dydt = zeros(4,1);
dydt(1) = -9*y(1) + y(3);
dydt(2) = 4*y(1)*y(2)*y(3) - (y(2))^2;
dydt(3) = 2*y(1) - y(2) - 2*y(4);
dydt(4) = y(2)*y(3) - (y(3))^2 - 3*(y(4))^3;
end

% Set relative and absolute tolerance
tol = 1e-8;
opts = odeset(RelTol=tol,AbsTol=tol);
% Get the general solution to the dynamic problem
[t,y] = ode45(@ODEFUN,tspan,initial_condition,opts);

% Subplot of 4 rows 1 column displaying evolution of each component
figure();
subplot(4,1,1);
plot(t,y(:,1));
title('W Component');
ylabel('W (n.d.)');
xlabel('Time (n.d.)');

subplot(4,1,2);
plot(t,y(:,2));
title('X Component');
ylabel('X (n.d.)');
xlabel('Time (n.d.)');

subplot(4,1,3);
plot(t,y(:,3));
title('Y Component');
ylabel('Y (n.d.)');
xlabel('Time (n.d.)');

subplot(4,1,4);
plot(t,y(:,4));
title('Z Component');
ylabel('Z (n.d.)');
xlabel('Time (n.d.)');

% Setup the error matrix for the table to show different
% tolerance errors
ToleranceDifferenceMatrix = zeros(4,5);
% Reference tolerance
RefTol = 1e-12;
% Vector of test tolerances
tols = [1e-2,1e-4,1e-6,1e-8,1e-10];
% Set reference general solution
ReferenceOpts = odeset(RelTol=RefTol,AbsTol=RefTol);
[ReferenceT,ReferenceY] = ode45(@ODEFUN,tspan,initial_condition,ReferenceOpts);

% Iterate through the 5 test tolerances and set the corresponding
% matrix index to the absolute value of the error per test tolerance
for i = 1:5
    LocalOps = odeset(RelTol=tols(i),AbsTol=tols(i));
    [LocalT,LocalY] = ode45(@ODEFUN,tspan,initial_condition,LocalOps);
    ToleranceDifferenceMatrix(1,i) = abs(LocalY(end,1) - ReferenceY(end,1));
    ToleranceDifferenceMatrix(2,i) = abs(LocalY(end,2) - ReferenceY(end,2));
    ToleranceDifferenceMatrix(3,i) = abs(LocalY(end,3) - ReferenceY(end,3));
    ToleranceDifferenceMatrix(4,i) = abs(LocalY(end,4) - ReferenceY(end,4));
end