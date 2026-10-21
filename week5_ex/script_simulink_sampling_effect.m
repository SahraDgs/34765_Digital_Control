clear
format compact

%Robot
gear = 9.6;
wrad = 0.0315;

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


out = sim('Motor_model_with_controller_sampling');

f = figure('Color', 'w');
hold off
plot(out.velP.Time, out.velP.Data(:,1), 'LineWidth', 1.5)
hold on
plot(out.velP.Time, out.velP.Data(:,2), 'LineWidth', 1.5)
plot(out.velP.Time, out.velP.Data(:,4), 'LineWidth', 1.5)
grid on
xlabel('Time (s)');
set(gca, 'Color','w', 'XColor','k', 'YColor','k')
legend('ref (m/s)', 'motor voltage /10 (V)', 'velocity (m/s)')
%exportgraphics(f, 'step_response.png', 'Resolution', 300, 'BackgroundColor', 'white');
