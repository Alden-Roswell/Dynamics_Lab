function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

%%Inertial Position
figure(fig(1));
hold on;
subplot(3,1,1); plot(time, aircraft_state_array(1,:),col(1),DisplayName=col(2))
title('Inertial Position: x (North)')
xlabel('Time (s)')
ylabel('x (m)')
legend()
grid on;
hold on;

subplot(3,1,2); plot(time, aircraft_state_array(2,:),col(1),DisplayName=col(2))
title('Inertial Position: y (East)')
xlabel('Time (s)')
ylabel('y (m)')
legend()
grid on;
hold on;

subplot(3,1,3); plot(time, aircraft_state_array(3,:),col(1),DisplayName=col(2))
title('Inertial Position: z (Down)')
xlabel('Time (s)')
ylabel('z (m)')
legend()
grid on;
hold on;



%%Euler Angles
figure(fig(2));
hold on;
subplot(3,1,1); plot(time, aircraft_state_array(4,:),col(1),DisplayName=col(2))
title('Euler Angles: \phi (Roll)')
xlabel('Time (s)')
ylabel('\phi (rad)')
legend()
grid on;
hold on;

subplot(3,1,2); plot(time, aircraft_state_array(5,:),col(1),DisplayName=col(2))
title('Euler Angles: \theta (Pitch)')
xlabel('Time (s)')
ylabel('\theta (rad)')
legend()
grid on;
hold on;

subplot(3,1,3); plot(time, aircraft_state_array(6,:),col(1),DisplayName=col(2))
title('Euler Angles: \psi (Yaw)')
xlabel('Time (s)')
ylabel('\psi (rad)')
legend()
grid on;
hold on;

%%Inertial Body Frame Velocity
figure(fig(3));
hold on;
subplot(3,1,1); plot(time, aircraft_state_array(7,:),col(1),DisplayName=col(2))
title('Body Frame Inertial Velocity: u (Forward)')
xlabel('Time (s)')
ylabel('u (m/s)')
legend()
grid on;
hold on;

subplot(3,1,2); plot(time, aircraft_state_array(8,:),col(1),DisplayName=col(2))
title('Body Frame Inertial Velocity: v (Right)')
xlabel('Time (s)')
ylabel('v (m/s)')
legend()
grid on;
hold on;

subplot(3,1,3); plot(time, aircraft_state_array(9,:),col(1),DisplayName=col(2))
title('Body Frame Inertial Velocity: w (Down)')
xlabel('Time (s)')
ylabel('w (m/s)')
legend()
grid on;
hold on;

figure(fig(4));
hold on;
%%Angular Velocity
subplot(3,1,1); plot(time, aircraft_state_array(10,:),col(1),DisplayName=col(2))
title('Angular Velocity: p (Roll Rate)')
xlabel('Time (s)')
ylabel('p (rad/s)')
legend()
grid on;
hold on;

subplot(3,1,2); plot(time, aircraft_state_array(11,:),col(1),DisplayName=col(2))
title('Angular Velocity: q (Pitch Rate)')
xlabel('Time (s)')
ylabel('q (rad/s)')
legend()
grid on;
hold on;

subplot(3,1,3); plot(time, aircraft_state_array(12,:),col(1),DisplayName=col(2))
title('Angular Velocity: r (Yaw Rate)')
xlabel('Time (s)')
ylabel('r (rad/s)')
legend()
grid on;
hold on;


figure(fig(5))
hold on;
grid on;
subplot(4,1,1); plot(time, control_input_array(1,:),col(1),DisplayName=col(2))
title('Control Input: Z_c')
xlabel('Time (s)')
ylabel('Z_c')
legend()
grid on;
hold on;

subplot(4,1,2); plot(time, control_input_array(2,:),col(1),DisplayName=col(2))
title('Control Input: L_c')
xlabel('Time (s)')
ylabel('L_c')
legend()
grid on;
hold on;

subplot(4,1,3); plot(time, control_input_array(3,:),col(1),DisplayName=col(2))
title('Control Input: M_c')
xlabel('Time (s)')
ylabel('M_c')
legend()
grid on;
hold on;

subplot(4,1,4); plot(time, control_input_array(4,:),col(1),DisplayName=col(2))
title('Control Input: N_c')
xlabel('Time (s)')
ylabel('N_c')
legend()
grid on;
hold on;

ax = figure(fig(6));
hold on;
grid on;
plot3(aircraft_state_array(1,:),aircraft_state_array(2,:),aircraft_state_array(3,:),col(1),DisplayName=col(2))
scatter3(aircraft_state_array(1,1),aircraft_state_array(2,1),aircraft_state_array(3,1), 'g', 'filled',DisplayName="Start")
scatter3(aircraft_state_array(1,end),aircraft_state_array(2,end),aircraft_state_array(3,end), 'r', 'filled',DisplayName="End")
ax = gca;
set(ax, 'YDir', 'reverse')
set(ax, 'ZDir', 'reverse')
axis equal;
view(3)
legend()
end