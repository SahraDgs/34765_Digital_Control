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

%Controller turnrate
Kpt = 5;
taui_t = 0.2;


out = sim('full_robot_driving_straight_turnrate_control');

f = figure('Color', 'w');
hold off
plot(out.turn.Time, out.turn.Data(:,1), 'LineWidth', 1.5)   %turnrate ref
hold on
plot(out.robot.Time, out.robot.Data(:,5), 'LineWidth', 1.5)
plot(out.turn.Time, out.turn.Data(:,2), 'LineWidth', 1.5)   %turnrate (rad/s)
plot(out.turn.Time, out.turn.Data(:,4)/10, 'LineWidth', 1.5)   %tr_i
plot(out.velP_left.Time, out.velP_left.Data(:,2)/10, 'LineWidth', 1.5)  %mot
plot(out.velP_right.Time, out.velP_right.Data(:,2)/10, 'LineWidth', 1.5)
grid on
xlabel('Time (s)'); 
set(gca, 'Color','w', 'XColor','k', 'YColor','k')
legend('Turnrate ref (rad/s)', 'Heading (rad)', 'turnrate (rad/s)','turnrate i/10','Motor voltage left/10 (V)', 'Motor voltage right/10 (V)')
exportgraphics(f, 'matlab_plot_staright_tr_control.png', 'Resolution', 300, 'BackgroundColor', 'white');
