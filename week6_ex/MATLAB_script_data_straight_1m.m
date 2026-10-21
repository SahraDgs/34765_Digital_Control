clear
format compact
gear = 9.68;
wrad = 0.03;
% Charge data from the log that correspond to an straighteasing sequence of the
% voltage
dd_straight = load('data_straight_1m_lec6.log');

N_straight = 3242;          % samples per run
run_straight = 1;
d_straight = dd_straight((run_straight-1)*N_straight + (1:N_straight), :);

t_straight = d_straight(:,1);
driv_vel = d_straight(:,16);
turn_rate = d_straight(:,17);

distance = d_straight(:, 18);
heading = d_straight(:, 19);
x = d_straight(:, 20);
y = d_straight(:, 21);

% PLot

figure('Position', [100 100 1200 450], 'Theme', 'light');
plot(t_straight, driv_vel)   
hold on
plot(t_straight, turn_rate)

grid on
legend('Driving Velocity', 'turn rate (rad/s)')
xlabel('Time (sec)')
exportgraphics(gcf, 'plot_pos_vel.png', 'Resolution', 300);



figure('Position', [100 100 1200 450], 'Theme', 'light');
plot(t_straight, distance, 'LineWidth', 1.5)   
hold on
plot(t_straight, heading, 'LineWidth', 1.5)
plot(t_straight, x, 'LineWidth', 1.5)
plot(t_straight, y, 'LineWidth', 1.5)

grid on
legend('distance (m)', 'heading (rad)', 'x (m)', 'y (m)')
xlabel('Time (sec)')
exportgraphics(gcf, 'plot_pos.png', 'Resolution', 300);


