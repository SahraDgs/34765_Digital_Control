clear;

%%Robot
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

%% temp

kp_t = 5;
taui_t = 0.2;

%temporary parameters
Kph = 5;
alpha = 1;    % lead = (s+1)/(s+1) = 1 while linearizing
taud = 1;

%simulink switch
useZOH = 0;

%for teh analysis
t_step = 0.5;
ref_heading = pi/2;

%% estimate turn model transfer function
%
model = 'week8_heading_control';
load_system(model);
open_system(model);
% define points in model
ios(1) = linio(strcat(model,'/Uh'),1,'openinput');
ios(2) = linio(strcat(model, '/Robot model'),3,'openoutput');
% attach to model
setlinio(model,ios);
% Use the snapshot time(s) 0 seconds
op = [1.0];
% Linearize the model
sys = linearize(model,ios,op,linearizeOptions('SampleTime',0)); % before: sys = linearize(model,ios,op);
%% get transfer function
Gheading = minreal(tf(sys));
%Before
%[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
%Gheading = minreal(tf(num, den));

%Gheading
%w = logspace(0,2,1000);
%h = figure(220);
%hold off
%bode(Gheading, w)
%grid on


%% Frequency response of the plant
w_axe = logspace(-1, 3, 50000);    % 0.1 to 1000 rad/s
[mag, phase, w] = bode(Gheading, w_axe);
mag = squeeze(mag);
phase = squeeze(phase);



%% P (Kph) + lead controller
phase_margin = 60;
phi_lead = 40;                                    % chosen phase lead (deg), keep below ~55
alpha = (1 - sind(phi_lead))/(1 + sind(phi_lead));
phase_c_lead = -180 + phase_margin - phi_lead;    % plant phase at the new crossover
k_wc = find(phase < phase_c_lead, 1);

wc_lead = w(k_wc);
taud = 1/(wc_lead*sqrt(alpha));                % max phase lead exactly at wc
Kph = sqrt(alpha)/mag(k_wc);                     % because |lead(j*wc)| = 1/sqrt(alpha)

Glead = tf([taud 1], [alpha*taud 1]);

fprintf('P+lead: wc = %.2f rad/s, Kph = %.3f, alpha = %.3f, taud = %.4f s\n', wc_lead, Kph, alpha, taud)

%% Lead coefficients for the firmware (Tustin, T = Ts)
% x = Kph*e;  u = b0*x + b1*x1 - a1*u1;  x1 = x;  u1 = u;
[bz, az] = tfdata(c2d(Glead, Ts, 'tustin'), 'v');
fprintf('Firmware lead: b0 = %.6f, b1 = %.6f, a1 = %.6f\n', bz(1), bz(2), az(2))

%% Simulink check (with ZOH)
useZOH = 1;
out = sim(model);
t = out.robot.Time;
y = out.robot.Data(:,5);    % heading
r = out.robot.Data(:,2);    % reference

k = t >= t_step;
S = stepinfo(y(k), t(k) - t_step, ref_heading);
fprintf('Rise time     = %.1f ms\n', S.RiseTime*1000)
fprintf('Overshoot     = %.2f %%\n', S.Overshoot)
fprintf('Settling time = %.1f ms\n', S.SettlingTime*1000)
fprintf('SS error      = %.4f rad\n', ref_heading - y(end))

figure(220); clf
plot(t, y, t, r, '--'); grid on
xlabel('Time (s)'); ylabel('Heading (rad)')
legend('heading', 'reference')