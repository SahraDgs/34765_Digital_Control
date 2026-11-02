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

Kph = 5;

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
sys = linearize(model,ios,op);
%% get transfer function
[num,den] = ss2tf(sys.A, sys.B, sys.C, sys.D);
Gheading = minreal(tf(num, den));
w = logspace(0,2,1000);
h = figure(220);
hold off
bode(Gheading, w)
grid on

