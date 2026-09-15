%dd = load('logs/data_spike_sequence_1905samples.log'); % use the filename and directory of your logfile.

dd = load('logs/data_increasing_sequence_400.log');

N   = 185;          % samples per run
run = 1;            % 1..6
d   = dd((run-1)*N + (1:N), :);


t = d(:,1);
omega = d(:,8);
V = d(:,4);
i = d(:,14);

h = figure(1000);
hold off
plot(t, omega/100)   %motor velocity
hold on
plot(t, V)  % motor voltage
plot(t, i)  % current
grid on
legend('Velocity left (rad/s)', 'motor voltage (V)', 'current (A)')
xlabel('Time (sec)')

%xlim([0.015 0.035])

%t_start_moving = diff(d(:,14));

%print(t_start_moving)

vss_spike = d(55,5);  %to be sure (should be = 6)
iss_spike = d(55,14);
t_ss_spike = d(55,1);

R = vss_spike/iss_spike;

tau_current_threshold = 0.632 * iss_spike;

for j = 2 : length(d(:,14))
    if(d(j,14) > tau_current_threshold)
        time_cross_th = d(j,1);
        break
    end
end


tau = time_cross_th - 0.02;
L = tau * R;

delta_V = 3;   %6-3V

delta_i =  mean(i(t > 1.30 & t < 1.53)) - mean(i(t > 0.80 & t < 1.02));
%GIVE almost zero: redo the experiment with reduced sample t

delta_omega = mean(omega(t > 1.30 & t < 1.53)) - mean(omega(t > 0.80 & t < 1.02));

Kb = (delta_V - R*delta_i)/delta_omega;
Kt = Kb;

D = (Kt * delta_i)/delta_omega;


L
tau
Kb
Kt
D


weight = 930; %g