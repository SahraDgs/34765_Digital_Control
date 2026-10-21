clear
format compact
% Charge data from the log that correspond to an increasing sequence of the
% voltage
dd_incr = load('logs/data_increasing_sequence_1905.log');

N_incr = 3571;          % samples per run
run_incr = 1;
d_incr = dd_incr((run_incr-1)*N_incr + (1:N_incr), :);

t_incr = d_incr(:,1);
omega_incr = d_incr(:,8);
V_incr = d_incr(:,4);
i_incr = d_incr(:,14);

% Charge data from the log that correspond to a three spikes sequence for
% the voltage
dd_spike = load('logs/data_spike_sequence_1905samples.log');

N_spike = 1905;          % samples per run
run_spike = 1;
d_spike = dd_spike((run_spike-1)*N_spike + (1:N_spike), :);

t_spike = d_spike(:,1);
omega_spike = d_spike(:,8);
V_spike = d_spike(:,4);
i_spike = d_spike(:,14);

%Plot the velocity, the motor voltage and the current for the increasing
%sequence
h = figure(1000);
hold off
ax1 = subplot(2,1,1);
plot(t_incr, omega_incr/100)   %motor velocity
hold on
plot(t_incr, V_incr)  % motor voltage
plot(t_incr, i_incr)  % current
grid on
legend('Velocity left (rad/s)', 'motor voltage (V)', 'current (A)')
xlabel('Time (sec)')

%Same plot but for the three spikes sequence
hold off
ax2 = subplot(2,1,2);
plot(t_spike, omega_spike/100)   %motor velocity
hold on
plot(t_spike, V_spike)  % motor voltage
plot(t_spike, i_spike)  % current
grid on
legend('Velocity left (rad/s)', 'motor voltage (V)', 'current (A)')
xlabel('Time (sec)')

%xlim([0.015 0.035])
%xlim([0.325 0.35])
%xlim([0.63 0.66])


% Use the spike sequence to calculate R and L
vss_spike = V_spike(55);
iss_spike = (mean(i_spike(t_spike > 0.335 & t_spike < 0.34)));
t_ss_spike = t_spike(55);

R = vss_spike/iss_spike;

omega_thr = 0.632 * iss_spike;
for j = 2 : length(i_spike)
    if(i_spike(j) > omega_thr)
        time_cross_th = t_spike(j);
        break
    end
end

tau = time_cross_th - 0.02;
L = tau * R;


% Use the increasing sequence to calculate Kb (and Kt), D and J
delta_V = 3;   %6-3V

delta_i =  mean(i_incr(t_incr > 1.3 & t_incr < 1.5)) - mean(i_incr(t_incr > 0.8 & t_incr < 1.));

%GIVE almost zero: redo the experiment with reduced sample t

delta_omega = mean(omega_incr(t_incr > 1.3 & t_incr < 1.5)) - mean(omega_incr(t_incr > 0.8 & t_incr < 1.));

Kb = (delta_V - R*delta_i)/delta_omega;
Kt = Kb;

D = (Kt * delta_i)/delta_omega;

% Use the increasing sequence to calculate J (and tau1)

omega_ss_incr =  mean(omega_incr(t_incr > 1.3 & t_incr < 1.5));
omega_pr_step = mean(omega_incr(t_incr > 0.7 & t_incr < 1.0));

omega1_thr = omega_pr_step + 0.632 * (omega_ss_incr - omega_pr_step);
j0 = find(t_incr >= 1, 1);
for j = j0 : length(omega_incr)
    if(omega_incr(j) > omega1_thr)
        time_cross_th1 = t_incr(j);
        break
    end
end

tau1 = time_cross_th1 - 1;
omega1 = -1/tau1;
f = @(J) (-(D*L+J*R) + sqrt(D^2*L^2-2*D*J*L*R +J^2*R^2-4*J*Kb*Kt*L))./(2*J*L) - omega1;

J0 = 1e-5; %value from the slides

[J, fval, exitflag] = fsolve(f, J0);

fprintf('v spike %.2g\n', vss_spike)
fprintf('R = %.2g\n', R)
fprintf('L = %.2g\n', L)
fprintf('tau = %.2g\n', tau)
fprintf('Kb = %.2g\n', Kb)
fprintf('Kt = %.2g\n', Kt)
fprintf('D = %.2g\n', D)
fprintf('J = %.2g\n', J)


robot_weight = 0.931; %kg

J_new= 0.5 * 0.025 * 0.009^2 + 0.5 * robot_weight *(0.03/9.68)^2;

%% Transfer function
Gs = tf([Kt],[J_new*L (D*L + J_new*R) (D*R + Kt * Kb)]);
[num,den] = pade(0.01, 2);
Gd = tf(num, den);
h = figure(15);
hold off
bode(Gs);
hold on
bode(Gs*Gd);
grid on
legend('no delay', '10ms delay')

[Gm, Pm, Wg, Wc] = margin(Gs*Gd);


%% Stability analysis
G_loop = feedback(1*Gs*Gd, 1);
poles = pole(G_loop);
poles_no_delay = pole(feedback(1*Gs, 1));

%% Controller Design

phase_margin = 60;
phase_c_P = -180+60;

w_axe = logspace(-1, 4, 50000);       % 0.1 to 1e4 rad/s, 50000 points
[mag, phase, w] = bode(Gs*Gd, w_axe); 
mag = squeeze(mag);
phase = squeeze(phase);

if(length(mag)~=length(phase))
    fprintf('Attention Not the same length');
end

% P controller

wc_P = 0;

for k = 1:length(mag)
    %fprintf('diff phase %.2g\n', phase(k) - phase_c_P)
    if((phase(k) - phase_c_P) < 0)
        wc_P = w(k);
        G_P_contr = mag(k);
        break
    end
end

if wc_P == 0
    error('No crossing found for the P controler');
end

Kp_P = 1/G_P_contr;

%% PI controller
N = 3;
phase_c_PI = phase_c_P - atand(-1/N);

wc_PI = 0;
for k = 1:length(mag)
    %fprintf('diff phase %.2g\n', phase(k) - phase_c_PI)
    if((phase(k) - phase_c_PI) < 0)
        wc_PI = w(k);
        G_PI_contr = mag(k);
        break
    end
end

if wc_PI == 0
    error('No crossing found for the PI controller');
end

Kp_PI = 1/G_PI_contr;
tau_i = N/wc_PI;

% For simulink model
r_wheel = 0.03;
n       = 9.68;
r2m  = r_wheel / n;

poles
Wg
Kp_P
Kp_PI
tau_i
J_new
wc_PI

Kp_PI, tau_i, J_new, R, L, Kt, Kb, D
w_m = (D*R + Kt*Kb)/(J_new*R)
tau_m = 1/w_m


out = sim('Motor_model_with_controller');

f = figure('Color', 'w');
plot(out.so.Time, out.so.Data(:,4), 'LineWidth', 1.5)
grid on
xlabel('Time (s)'); ylabel('Velocity (rad/s)')
set(gca, 'Color','w', 'XColor','k', 'YColor','k')
exportgraphics(f, 'step_response.png', 'Resolution', 300, 'BackgroundColor', 'white');

out = sim('Motor_model_with_controller');
ref = 100;
t = out.so.Time;
y = out.so.Data(:, 4);

k = t >= 0.1;
S = stepinfo(y(k), t(k) - 0.1, ref);
e_ss = ref - y(end);

fprintf('Rise time = %.1f ms\n', S.RiseTime*1000)
fprintf('Overshoot = %.2f %%\n', S.Overshoot)
fprintf('Settling time = %.1f ms\n', S.SettlingTime*1000)
fprintf('Steady-state error = %.2f rad/s\n', e_ss)

poles_no_delay