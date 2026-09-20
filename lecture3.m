
s = tf('s');

% Define the transfer function G(s)
G = 1/(0.2*s + 1);

%Continius controller
C=(500*s+50)/(100*s^2+s);

CZa=c2d (C, 0.1);
CZb=c2d (C,1);

CZa
CZb
 

%% Exercise E1

%% E1a
% Given difference equation:
%
% u(k) = 0.5*u(k-1) - 0.3*u(k-2)
%
% Has the characteristic equation:
%
% z^2 - 0.5*z + 0.3 = 0

% Define the coefficients of the characteristic polynomial
p_a = [1 -0.5 0.3];

% Find the roots of the characteristic equation
z_a = roots(p_a);

% Calculate the magnitude of each root
mag_a = abs(z_a);
%mag_a

% Check if all roots are inside the unit circle
if all(mag_a < 1)
    disp('E1a is stable')
else
    disp('E1a is unstable')
end


%% E1b
% Given difference equation:
%
% u(k) = 1.6*u(k-1) - 1*u(k-2)
%
% Has the characteristic equation:
%
% z^2 - 1.6*z + 1 = 0

% Define the coefficients of the characteristic polynomial
p_b = [1 -1.6 1];

% Find the roots of the characteristic equation
z_b = roots(p_b);

% Calculate the magnitude of each root
mag_b = abs(z_b);
mag_b

% Check if all roots are inside the unit circle
if all(mag_b < 1)
    disp('E1b is stable')
else
    disp('E1b is unstable')
end



%% E1c
% Given difference equation:
%
% u(k) = 0.8*u(k-1) + 0.4*u(k-2)
%
% Has the characteristic equation:
%
% z^2 - 0.8*z - 0.4 = 0

% Define the coefficients of the characteristic polynomial
p_c = [1 -0.8 -0.4];

% Find the roots of the characteristic equation
z_c = roots(p_c);

% Calculate the magnitude of each root
mag_c = abs(z_c);
mag_c

% Check if all roots are inside the unit circle
if all(mag_c < 1)
    disp('E1c is stable')
else
    disp('E1c is unstable')
end


%% Exercise E2

%% E2a

% Difference equation:
% u_(k+2) = 0.25*u_k
%
% Assume a solution of the form:
% u_k = A*z^k
%
% Substitute into the difference equation:
% A*z^(k+2) = 0.25*A*z^k
%
%
% Divide by A*z^k:
% z^2 = 0.25
%
% Characteristic equation:
% z^2 - 0.25 = 0

% Find the roots of the characteristic equation
z_2a = roots([1 0 -0.25]);

% Find the magnitude of the roots
mag_2a = abs(z_2a);

% Check if all roots are inside the unit circle
if all(mag_2a < 1)
    disp('E2a is stable')
else
    disp('E2a is unstable')
end

% Both roots have magnitude smaller than 1,
% therefore the system is stable.

%% E2b

% Difference equation:
% u_(k+2) = -0.25*u_k
%
% Assume a solution of the form:
% u_k = A*z^k
%
% Substitute into the difference equation:
% A*z^(k+2) = -0.25*A*z^k
%
%
% Divide by A*z^k:
% z^2 = -0.25
%
% Characteristic equation:
% z^2 + 0.25 = 0

% Find the roots of the characteristic equation
z_2b = roots([1 0 0.25]);

% Find the magnitude of the roots
mag_2b = abs(z_2b);

% Check if all roots are inside the unit circle
if all(mag_2b < 1)
    disp('E2b is stable')
else
    disp('E2b is unstable')
end

% Both roots have magnitude smaller than 1,
% therefore the system is stable.


%% E2c

% Difference equation:
% u_(k+2) = u_(k+1) - 0.5*u_k
%
% Assume a solution of the form:
% u_k = A*z^k
%
% Substitute into the difference equation:
% A*z^(k+2) = A*z^(k+1) - 0.5*A*z^k
%
% Divide by A*z^k:
% z^2 = z - 0.5
%
% Characteristic equation:
% z^2 - z + 0.5 = 0

% Find the roots of the characteristic equation
z_2c = roots([1 -1 0.5]);

% Find the magnitude of the roots
mag_2c = abs(z_2c);

% Check if all roots are inside the unit circle
if all(mag_2c < 1)
    disp('E2c is stable')
else
    disp('E2c is unstable')
end

% Both roots have magnitude smaller than 1,
% therefore the system is stable.


%% E2 find A1 and A2

%% E2 find A1 and A2

%% E2a

% General solution:
% u_k = A1*z1^k + A2*z2^k
%
% Initial conditions:
% u_0 = 0
% u_1 = 1

z1 = z_2a(1);
z2 = z_2a(2);

% From the initial conditions:
% A1 + A2 = 0
% A1*z1 + A2*z2 = 1

A = [1 1;
     z1 z2];

b = [0;
     1];

A_2a = A\b;

A1_2a = A_2a(1)
A2_2a = A_2a(2)


%% E2b

z1 = z_2b(1);
z2 = z_2b(2);

A = [1 1;
     z1 z2];

b = [0;
     1];

A_2b = A\b;

A1_2b = A_2b(1)
A2_2b = A_2b(2)


%% E2c

z1 = z_2c(1);
z2 = z_2c(2);

A = [1 1;
     z1 z2];

b = [0;
     1];

A_2c = A\b;

A1_2c = A_2c(1)
A2_2c = A_2c(2)

%% Exercise E3

%% E3a

% Characteristic equation:
% z^3 - 1.1*z^2 + 0.01*z + 0.405 = 0

% Find the roots
z_3a = roots([1 -1.1 0.01 0.405]);

% Find the magnitude of the roots
mag_3a = abs(z_3a);

% Check how many roots are outside the unit circle
outside_3a = sum(mag_3a > 1);

z_3a
mag_3a

if outside_3a == 0
    disp('E3a has no roots outside the unit circle')
elseif outside_3a == 1
    disp('E3a has 1 root outside the unit circle')
else
    fprintf('E3a has %d roots outside the unit circle\n', outside_3a)
end


%% E3b

% Characteristic equation:
% z^3 - 3.6*z^2 + 4*z - 1.6 = 0

% Find the roots
z_3b = roots([1 -3.6 4 -1.6]);

% Find the magnitude of the roots
mag_3b = abs(z_3b);

% Check how many roots are outside the unit circle
outside_3b = sum(mag_3b > 1);

z_3b
mag_3b

if outside_3b == 0
    disp('E3b has no roots outside the unit circle')
elseif outside_3b == 1
    disp('E3b has 1 root outside the unit circle')
else
    fprintf('E3b has %d roots outside the unit circle\n', outside_3b)
end
