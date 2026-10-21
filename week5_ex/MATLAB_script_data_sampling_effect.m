clear
format compact
gear = 9.68;
wrad = 0.03;
% Charge data from the log that correspond to an increasing sequence of the
% voltage
dd_incr = load('data_incr_seq_sampling_effect_80000.log');

N_incr = 1802;          % samples per run
run_incr = 1;
d_incr = dd_incr((run_incr-1)*N_incr + (1:N_incr), :);

t_incr = d_incr(:,1);
omega_incr = d_incr(:,8);

i_incr_left = d_incr(:,14);
i_incr_right = d_incr(:,15);



%Plot the velocity, the motor voltage and the current for the increasing
%sequence
h = figure(1000);
hold off
ax1 = subplot(2,1,1);
plot(t_incr, omega_incr/1000)   %motor velocity
hold on
plot(t_incr, i_incr_left)  % current left
plot(t_incr, i_incr_right)  % current right
grid on
legend('Velocity left (rad/s)', 'current left (A)', 'current right (A)')
xlabel('Time (sec)')


dd_incr = load('data_incr_seq_sampling_effect_1990.log');

N_incr = 1802;          % samples per run
run_incr = 1;
d_incr = dd_incr((run_incr-1)*N_incr + (1:N_incr), :);

t_incr = d_incr(:,1);
omega_incr = d_incr(:,8);

i_incr_left = d_incr(:,14);
i_incr_right = d_incr(:,15);

h = figure(1000);
hold off
ax1 = subplot(2,1,1);
plot(t_incr, omega_incr/1000)   %motor velocity
hold on
plot(t_incr, i_incr_left)  % current left
plot(t_incr, i_incr_right)  % current right
grid on
legend('Velocity left (rad/s)', 'current left (A)', 'current right (A)')
xlabel('Time (sec)')






dd_incr = load('data_incr_seq_P_control_20000.log');

N_incr = 94;          % samples per run
run_incr = 1;
d_incr = dd_incr((run_incr-1)*N_incr + (1:N_incr), :);

t_incr = d_incr(:,1);
omega_incr = d_incr(:,8)* wrad / gear;

voltage_motor_left = d_incr(:,4);
voltage_motor_right = d_incr(:,5);
ref = d_incr(:, 3);

h = figure(1000);
hold off
ax1 = subplot(2,1,1);
plot(t_incr, omega_incr)   %motor velocity
hold on
plot(t_incr, voltage_motor_left)  % current left
plot(t_incr, ref);
%plot(t_incr, voltage_motor_right)  % current right

grid on
legend('Velocity left (rad/s)', 'motor voltage left', 'ref')
xlabel('Time (sec)')

dd_incr = load('data_incr_seq_P_control.log');

N_incr = 1802;          % samples per run
run_incr = 1;
d_incr = dd_incr((run_incr-1)*N_incr + (1:N_incr), :);

t_incr = d_incr(:,1);
omega_incr = d_incr(:,8)* wrad / gear;

voltage_motor_left = d_incr(:,4);
voltage_motor_right = d_incr(:,5);
ref = d_incr(:, 3);

h = figure(1000);
hold off
ax1 = subplot(1,1,1);
plot(t_incr, omega_incr)   %motor velocity
hold on
plot(t_incr, voltage_motor_left)  % current left
plot(t_incr, ref);
%plot(t_incr, voltage_motor_right)  % current right

grid on
legend('Velocity left (rad/s)', 'motor voltage left', 'ref')
xlabel('Time (sec)')
xlim([0 1.6])



