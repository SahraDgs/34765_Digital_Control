%% Load measured step sequence

dd_step = load('device-monitor-260915-135104.log');

t_step     = dd_step(:,1);   % time [s]
V_step     = dd_step(:,4);   % left motor voltage [V]
encoder_step = dd_step(:,6); % left encoder count
omega_step = dd_step(:,8);   % left motor velocity [rad/s]
i_step     = dd_step(:,14);  % motor current [A]


%% Plot measured step sequence

h = figure(1);

plot(t_step, V_step, 'LineWidth', 1.5)
hold on
plot(t_step, i_step, 'LineWidth', 1.5)
plot(t_step, omega_step/100, 'LineWidth', 1.5)

grid on
xlabel('Time (sec)')
ylabel('Value')

legend('Motor voltage (V)', ...
       'Motor current (A)', ...
       'Velocity (rad/s / 100)', ...
       'Location','northwest')

title('Measured step sequence')

xlim([0 1.6])

%% Load measured spike sequence

dd_spike = load('data_spike_sequence_1905samples.log');

t_spike       = dd_spike(:,1);
V_spike       = dd_spike(:,4);
encoder_spike = dd_spike(:,6);
omega_spike   = dd_spike(:,8);
i_spike       = dd_spike(:,14);

% Change in encoder value
dEncoder = [0; diff(encoder_spike)];

%% Plot measured spike sequence

figure(2)

yyaxis left
plot(t_spike, V_spike, 'LineWidth', 1.5)
hold on
plot(t_spike, i_spike, 'LineWidth', 1.5)
plot(t_spike, dEncoder, 'LineWidth', 1.5)
xlim([0.015 0.040])


ylabel('Voltage (V) / Current (A) / Encoder change')

yyaxis right
plot(t_spike, omega_spike, 'LineWidth', 1.5)

ylabel('Motor velocity (rad/s)')

grid on
xlabel('Time (sec)')

legend('Motor voltage (V)', ...
       'Motor current (A)', ...
       'Change in encoder', ...
       'Motor velocity (rad/s)', ...
       'Location','best')

title('Measured spike sequence')

%% Find R from spike experiment

% Region where current is approximately steady
% and motor has not started moving much yet
idx_R = t_spike >= 0.023 & t_spike <= 0.025;

Vss = mean(V_spike(idx_R));
iss = mean(i_spike(idx_R));

R = Vss / iss;

%fprintf('Vss = %.3f V\n', Vss);
%fprintf('iss = %.3f A\n', iss);
fprintf('R = %.3f Ohm\n', R);

%% Find L

% Current at 63.2% of steady-state value
i_tau = 0.632 * iss;

% Start of first 6 V pulse
idx_start = find(V_spike > 5, 1, 'first');
t_start = t_spike(idx_start);

% Find first time current reaches 63.2%
idx_tau = find((t_spike >= t_start) & ...
               (i_spike >= i_tau), 1, 'first');

t_tau = t_spike(idx_tau);

% Electrical time constant
tau = t_tau - t_start;

% Inductance
L = tau * R;

%fprintf('63.2%% current = %.3f A\n', i_tau);
%fprintf('t_start = %.6f s\n', t_start);
%fprintf('t_tau = %.6f s\n', t_tau);
%fprintf('tau = %.6f s\n', tau);
fprintf('L = %.6f H\n', L);

%% Find Kb from 3V -> 6V step

% Steady-state regions
idx_3V = t_step >= 0.85 & t_step <= 0.98;
idx_6V = t_step >= 1.35 & t_step <= 1.48;

% Mean values
V_3 = mean(V_step(idx_3V));
V_6 = mean(V_step(idx_6V));

i_3 = mean(i_step(idx_3V));
i_6 = mean(i_step(idx_6V));

omega_3 = mean(omega_step(idx_3V));
omega_6 = mean(omega_step(idx_6V));

% Differences
delta_V = V_6 - V_3;
delta_i = i_6 - i_3;
delta_omega = omega_6 - omega_3;

%fprintf('delta_V = %.3f V\n', delta_V);
%fprintf('delta_i = %.4f A\n', delta_i);
%fprintf('delta_omega = %.2f rad/s\n', delta_omega);


%% Calculate Kb

delta_V_R = R * delta_i;

delta_V_bemf = delta_V - delta_V_R;

Kb = delta_V_bemf / delta_omega;

%fprintf('delta_V_R = %.4f V\n', delta_V_R);
%fprintf('delta_V_bemf = %.4f V\n', delta_V_bemf);
fprintf('Kb = %.6f V/(rad/s)\n', Kb);

%% Find Kt

Kt = Kb;

fprintf('Kt = %.6f Nm/A\n', Kt);

%% Find D

delta_T = Kt * delta_i;

D = delta_T / delta_omega;

%fprintf('delta_T = %.8f Nm\n', delta_T);
fprintf('D = %.10f Nm/(rad/s)\n', D);


%% Find mechanical time constant for J

% 63.2% point between 3 V and 6 V steady-state velocities
omega_tau = omega_3 + 0.632 * (omega_6 - omega_3);

%fprintf('omega_3 = %.2f rad/s\n', omega_3);
%fprintf('omega_6 = %.2f rad/s\n', omega_6);
%fprintf('63.2%% velocity = %.2f rad/s\n', omega_tau);


%% Find J

% 63.2% of velocity change from 3 V to 6 V
omega_tau = omega_3 + 0.632*(omega_6 - omega_3);

% Find start of 6 V step
idx_start_J = find(t_step >= 1.0 & V_step >= 5.5, 1, 'first');
t_start_J = t_step(idx_start_J);

% Find when velocity reaches 63.2%
idx_tau_J = find((t_step >= t_start_J) & ...
                 (omega_step >= omega_tau), 1, 'first');

t_tau_J = t_step(idx_tau_J);

% Mechanical time constant
tau_m = t_tau_J - t_start_J;

% Moment of inertia
J = tau_m * (D + (Kt*Kb)/R);

%fprintf('omega_tau = %.2f rad/s\n', omega_tau);
%fprintf('t_start = %.4f s\n', t_start_J);
%fprintf('t_tau = %.4f s\n', t_tau_J);
%fprintf('tau_m = %.4f s\n', tau_m);
fprintf('J = %.10f kg*m^2\n', J);

%% Exercise 2

%% Effect of robot mass

robotMass  = 0.931;     % Total robot mass [kg]
gear       = 9.68;      % Gear ratio
wheelRadius = 0.03;     % Wheel radius [m]


r2m = wheelRadius/gear;

% J_mass = (m/2)*(r/g)^2
%
% where:
%   m = robot mass
%   r = wheel radius
%   g = gear ratio

J_mass = (robotMass/2) * (wheelRadius/gear)^2;

% Add the equivalent inertia from the robot mass to the inertia
% identified in Exercise 1.

J_total = J + J_mass;


%% Transfer function
Gs = tf([Kt],[J_total*L (D*L + J_total*R) (D*R + Kt * Kb)]);
[num,den] = pade(0.01, 2);
Gd = tf(num, den);
h = figure(15);
hold off
bode(Gs)
hold on
bode(Gs*Gd)
grid on
legend('no delay', '10ms delay')

%% Quenstion 1: PI-frequency

% Find gain margin, phase margin and crossover frequencies
[Gm, Pm, Wcg, Wcp] = margin(Gs*Gd);

% Wcg is the frequency where the phase is -180 degrees, aka the PI-frequency
w_PI = Wcg;
w_PI

%% Quenstion 2: Stability with Kp = 1

Kp = 1;

% Open-loop transfer function
G_open = Kp * Gs * Gd;

% Find gain and phase margins
[Gm, Pm, Wcg, Wcp] = margin(G_open);

fprintf('Gain margin = %.4f\n', Gm);
fprintf('Phase margin = %.2f degrees\n', Pm);

% Closed-loop transfer function with negative feedback
G_closed = feedback(G_open, 1);

% Find closed-loop poles
p = pole(G_closed);

disp('Closed-loop poles:')
disp(p)

% A continuous-time system is stable if all poles
% have negative real parts
if all(real(p) < 0)
    disp('The system is stable')
else
    disp('The system is unstable')
end


%% Check what causes instability

Kp = 1;

% 1. Motor only - no delay
G_closed_no_delay = feedback(Kp*Gs, 1);

disp('--- NO DELAY ---')
pole(G_closed_no_delay)

if isstable(G_closed_no_delay)
    disp('Stable without delay')
else
    disp('Unstable without delay')
end


% 2. Motor + 10 ms delay
G_closed_delay = feedback(Kp*Gs*Gd, 1);

disp('--- WITH 10 ms DELAY ---')
pole(G_closed_delay)

if isstable(G_closed_delay)
    disp('Stable with delay')
else
    disp('Unstable with delay')
end


%% Quenstion 3

%% Design considerations:

%Will there be a steady-state error for a step input?
%Yes. With a P-controller, there will be a steady-state 
% error for a step input because the system has no integral 
% action. A PI-controller introduces integral action and can 
% eliminate the steady-state error.

%Could a D or Lead term improve response?
%Yes. A D or Lead term could improve the transient response 
% by increasing the phase margin. However, it also increases 
% the high-frequency gain and may amplify noise in the velocity 
% measurement. Therefore, a PI-controller is preferred.


%A D (and Lead) term includes an element of differentiation, 
% which increases high-frequency gain.
%Could that be an issue, considering the noise on the velocity 
% estimate?

%% PI controller design
%% PI controller design
%% PI controller design

% Desired phase margin
PM_desired = 60;      % degrees

% PI zero is placed a factor N below crossover frequency
N = 3;

% For N = 3, the PI controller adds approximately -18 deg.
% Therefore, the plant phase at crossover should be -102 deg.
phase_target = -102;

% Frequency range
w = logspace(-1, 4, 10000);

% Get frequency response
[mag, phase] = bode(Gs*Gd, w);

mag   = squeeze(mag);
phase = squeeze(phase);

% Wrap phase to the range [-360, 0] degrees
phase(phase > 0) = phase(phase > 0) - 360;

% Find frequency where phase is closest to -102 degrees
[~, idx] = min(abs(phase - phase_target));

wc = w(idx);

% PI zero frequency
w_PI = wc / N;

% Integrator time constant
tau_i = N / wc;

fprintf('Crossover frequency wc = %.3f rad/s\n', wc);
fprintf('PI zero frequency = %.3f rad/s\n', w_PI);
fprintf('tau_i = %.5f s\n', tau_i);
figure

bode(Gs*Gd)
grid on
xline(wc, '--', 'wc')

title('Plant with 10 ms delay')

%% Find Kp

% Find the magnitude of the plant at wc
[mag_wc, ~] = bode(Gs*Gd, wc);

mag_wc = squeeze(mag_wc);

% Convert magnitude to dB
mag_wc_dB = 20*log10(mag_wc);

% Choose Kp so the magnitude at wc becomes 0 dB
Kp = 10^(-mag_wc_dB/20);

fprintf('Magnitude at wc = %.2f dB\n', mag_wc_dB);
fprintf('Kp = %.5f\n', Kp);

%% Find Kp for appropriate phase margin

% PI controller using the calculated values
s = tf('s');

C_PI = Kp * (1 + 1/(tau_i*s));

% Open-loop system with PI controller and delay
G_open_PI = C_PI * Gs * Gd;

% Find gain margin and phase margin
[Gm_PI, Pm_PI, Wcg_PI, Wcp_PI] = margin(G_open_PI);

fprintf('Kp = %.5f\n', Kp);
fprintf('Phase margin = %.2f degrees\n', Pm_PI);
fprintf('Crossover frequency = %.2f rad/s\n', Wcp_PI);

% Plot margins
figure
margin(G_open_PI)
grid on
title('PI Controller - Stability Margins')


%% Plot step response

t = out.so.Time;
vel = out.so.Data(:,5);

figure(9)
plot(t, vel, 'LineWidth', 1.5)
grid on

xlabel('Time (s)')
ylabel('Motor velocity (rad/s)')
title('PI Controller Step Response')

xlim([0 0.5])

%% Plot step response

t = out.so.Time;
omega = out.so.Data(:,5);   % rad/s signal

figure
plot(t, omega, 'LineWidth', 1.5)
hold on

yline(100, '--', 'Reference = 100 rad/s')

grid on
xlabel('Time (s)')
ylabel('Motor velocity (rad/s)')
title('PI Controller Step Response')

xlim([0 0.5])
ylim([0 120])

fprintf('Kp                  = %.5f\n', Kp);
fprintf('tau_i               = %.5f s\n', tau_i);

%% Step response performance

t = out.so.Time;
omega = out.so.Data(:,5);

idx = t >= 0.1;

t_response = t(idx) - 0.1;
omega_response = omega(idx);

info = stepinfo(omega_response, t_response, 100);

%fprintf('Rise time     = %.4f s\n', info.RiseTime);
%fprintf('Settling time = %.4f s\n', info.SettlingTime);
%fprintf('Overshoot     = %.2f %%\n', info.Overshoot);
%fprintf('Peak velocity = %.2f rad/s\n', info.Peak);

%% The final PI controller parameters are:
% Kp = 0.09606
% tau_i = 0.11269 s
%
% The controller gives a phase margin of 59.25 degrees.
% In simulation, the step response has a rise time of 0.0512 s,
% a settling time of 0.0844 s, and an overshoot of only 0.59%.
% Therefore, these controller parameters provide a fast response
% with minimal overshoot and are selected as the final values.
