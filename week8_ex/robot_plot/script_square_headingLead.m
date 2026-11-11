clear
format compact
gear = 9.6;
wrad = 0.0315;
% Charge data from the log that correspond to an straighteasing sequence of the
% voltage
dd_straight = load('data_square_accLimit_final_version.log');

N_straight = 1826 ;          % samples per run
run_straight = 1;
d_straight = dd_straight((run_straight-1)*N_straight + (1:N_straight), :);

t_straight = d_straight(:,1);
driv_vel = d_straight(:,16);
turn_rate = d_straight(:,17);

heading = d_straight(:, 19);
x = d_straight(:, 20);
y = d_straight(:, 21);  

%to check the slip
w_enc = d_straight(:, 24);   %turn rate from the encoder (what we want)
gyro_z = d_straight(:, 13);  %actual turn rate (what we actually get)



vel_right = d_straight(:, 9) * wrad/gear;

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

plot(t_straight, heading, 'LineWidth', 1.5)
hold on
plot(t_straight, x, 'LineWidth', 1.5)
plot(t_straight, y, 'LineWidth', 1.5)
plot(t_straight, vel_right, 'LineWidth', 1.5)

grid on
legend('heading (rad)', 'x (m)', 'y (m)', 'velocity right (m/s)')
xlabel('Time (sec)')
exportgraphics(gcf, 'plot_pos.png', 'Resolution', 300);


%pot to check the slip
figure('Position', [100 100 1200 450], 'Theme', 'light');   

plot(t_straight, gyro_z* pi/180, 'LineWidth', 1.5)
hold on
plot(t_straight, w_enc, 'LineWidth', 1.5)

grid on
legend('actual turnrate (rad/s)', 'encoder turnrate (rad/s)')
xlabel('Time (sec)')
exportgraphics(gcf, 'plot_slip.png', 'Resolution', 300);

%pot to check the slip
figure('Position', [100 100 1200 450], 'Theme', 'light');   

plot(t_straight, (gyro_z* pi/180) - w_enc, 'LineWidth', 1.5)

grid on
legend('difference turnrate (rad/s)')
xlabel('Time (sec)')
exportgraphics(gcf, 'plot_slip_diff.png', 'Resolution', 300);


