clear
format compact

%Robot
gear = 9.6;
wrad = 0.0315;
B = 0.15;

%Model
R = 6.8;
L = 0.01;
Kb = 0.009;
Kt = 0.009;
D = 3.2e-7;
J = 5.1e-7;
J_new = 5.4836e-6;

%Controller
Kp_P = 0.05; %0.1046;
tau_i = 0.1187;
Ts = 0.001;


out = sim('Motor_model_with_Pcontroller_full_robot');

f = figure('Color', 'w');
hold off
plot(out.robot.Time, out.robot.Data(:,1), 'LineWidth', 1.5)
hold on
plot(out.robot.Time, out.robot.Data(:,2), 'LineWidth', 1.5)
plot(out.robot.Time, out.robot.Data(:,3), 'LineWidth', 1.5)
plot(out.robot.Time, out.robot.Data(:,4), 'LineWidth', 1.5)
plot(out.robot.Time, out.robot.Data(:,5), 'LineWidth', 1.5)
plot(out.robot.Time, out.robot.Data(:,6), 'LineWidth', 1.5)
grid on
xlabel('Time (s)'); 
set(gca, 'Color','w', 'XColor','k', 'YColor','k')
legend('Velocity Step (m/s)', 'Turnrate Step (rad/s)', 'Robot Distance (m)','Robot Velocity (m/s)','Robot heading (rad)', 'turnrate(rad/s)')
exportgraphics(f, 'BADmatlab_plot_turn.png', 'Resolution', 300, 'BackgroundColor', 'white');

f = figure('Color', 'w');
hold off
plot(out.pose.Time, out.pose.Data(:,1), 'LineWidth', 1.5)
hold on
plot(out.pose.Time, out.pose.Data(:,2), 'LineWidth', 1.5)
plot(out.pose.Time, out.pose.Data(:,3), 'LineWidth', 1.5)
plot(out.pose.Time, out.pose.Data(:,4), 'LineWidth', 1.5)
plot(out.pose.Time, out.pose.Data(:,5), 'LineWidth', 1.5)
plot(out.pose.Time, out.pose.Data(:,6), 'LineWidth', 1.5)
grid on
xlabel('Time (s)'); 
set(gca, 'Color','w', 'XColor','k', 'YColor','k')
legend('vel (m/s)', 'turnrate (rad/s)', 'distance (m)', 'heading (rad)',  'position x', 'position y')
exportgraphics(f, 'matlab_plot_straight.png', 'Resolution', 300, 'BackgroundColor', 'white');

