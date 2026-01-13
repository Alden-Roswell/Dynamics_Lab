%ODE 45 Example Call
%[TOUT,YOUT] = ODE45(ODEFUN,TSPAN,Y0,OPTIONS,ADDVAR)

StateInitial = [1,1,1,1];
TSpan = [0,20];

[TOut,X] = ode45(@ODEFUN, TSpan, StateInitial);
figure()
xlabel('Time');
ylabel('State Variable w');
title('ODE45 Solution for w over Time');
subplot(TOut,X(:,1))

function state_dot = ODEFUN(t,X)
w = X(1);
x = X(2);
y = X(3);
z = X(4);
w_dot = -9*w + y;
x_dot = 4*w*x*y - x^2;
y_dot = 2*w -x - 2*z;
z_dot = x*y - y^2 - 3*z^3;
state_dot = transpose([w_dot, x_dot, y_dot, z_dot]);

end