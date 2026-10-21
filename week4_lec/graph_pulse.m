clear; close all; clc;


Ts = 0.5;    % sample time
t_end  = 20;

H = @(r, theta) tf([1 -r*cosd(theta)], [1 -2*r*cosd(theta) r^2], Ts);

% Sect 1a: vary r, theta fixed
theta = 30;
r = [0.5 0.7 0.9];
sys = cell(size(r));
for i = 1:numel(r)
    sys{i} = H(r(i), theta);
    sys{i} = sys{i} / dcgain(sys{i});
end

figure('Position', [100 100 1200 450], 'Theme', 'light');
subplot(1,2,1);
step(sys{:}, t_end); grid on;
legend("r = " + r);
%title('Part 1a: varying r (\theta = 30^\circ) (normalized)');

subplot(1,2,2);
pzmap(sys{:});
axis equal;
axis([-1.5 1.5 -1.5 1.5]);


exportgraphics(gcf, 'part1a.png', 'Resolution', 300);

theta = 30;
r = [0.5 1 1.02 1.04];
sys = cell(size(r));
for i = 1:numel(r)
    sys{i} = H(r(i), theta);
    sys{i} = sys{i} / dcgain(sys{i});
end

figure('Position', [100 100 1200 450], 'Theme', 'light');
subplot(1,2,1);
step(sys{:}, t_end); grid on;
legend("r = " + r);
%title('Part 1a_unstable: varying r (\theta = 30^\circ) (normalized)');

subplot(1,2,2);
pzmap(sys{:});
axis equal;
axis([-1.5 1.5 -1.5 1.5]);


exportgraphics(gcf, 'part1a_unstable.png', 'Resolution', 300);

% Sect 1b: vary theta, r fixed
r = 0.8;
theta = [15 30 60];
sys = cell(size(theta));
for i = 1:numel(theta)
    sys{i} = H(r, theta(i));
    sys{i} = sys{i} / dcgain(sys{i});
end

figure('Position', [100 100 1200 450], 'Theme', 'light');
subplot(1,2,1);
step(sys{:}, t_end); grid on;
legend("\theta = " + theta + "^\circ");
%title('Part 1b: varying \theta (r = 0.8) (normalized)');

subplot(1,2,2);
pzmap(sys{:});
axis equal;
axis([-1.5 1.5 -1.5 1.5]);


exportgraphics(gcf, 'part1b.png', 'Resolution', 300);

r=0.8;
theta = [30 150];
sys = cell(size(theta));
for i = 1:numel(theta)
    sys{i} = H(r, theta(i));
    sys{i} = sys{i} / dcgain(sys{i});
end

figure('Position', [100 100 1200 450], 'Theme', 'light');
subplot(1,2,1);
step(sys{:}, t_end); grid on;
legend("\theta = " + theta + "^\circ");
%title('Part 1b: varying \theta (r = 0.8) (normalized)');

subplot(1,2,2);
pzmap(sys{:});
axis equal;
axis([-1.5 1.5 -1.5 1.5]);


exportgraphics(gcf, 'part1b_left_plane.png', 'Resolution', 300);


% Sect 2: zero fixed at c, only the imaginary part of the poles changes
c = 0.5;
theta = [15 30 45];   % keep theta < acosd(c) so that r < 1 (stable)
sys = cell(size(theta));
for i = 1:numel(theta)
    r = c / cosd(theta(i));
    sys{i} = H(r, theta(i));   % still eq. 4.104, since r*cosd(theta) = c
    sys{i} = sys{i} / dcgain(sys{i});
end


figure('Position', [100 100 1200 450], 'Theme', 'light');
subplot(1,2,1);
step(sys{:}, t_end); grid on;
legend("\theta = " + theta + "^\circ");
%title('Part 2: zero fixed at 0.5');

subplot(1,2,2);
pzmap(sys{:});
axis equal;
axis([-1.5 1.5 -1.5 1.5]);

exportgraphics(gcf, 'part2.png', 'Resolution', 300);

H3 = @(r, theta, z0) tf([1 -z0], [1 -2*r*cosd(theta) r^2], Ts);

b  = 0.5;             % fixed imaginary part of the poles
a  = [0.1 0.5];    % real parts to compare
z0 = 0.5;             % fixed zero
sys = cell(size(a));
for i = 1:numel(a)
    r     = sqrt(a(i)^2 + b^2);
    theta = atan2d(b, a(i));
    sys{i} = H3(r, theta, z0);
    sys{i} = sys{i} / dcgain(sys{i});
end


figure('Position', [100 100 1200 450], 'Theme', 'light');
subplot(1,2,1);
step(sys{:}, t_end); grid on;
legend("\theta = " + theta + "^\circ");
%title('Part 2: zero fixed at 0.5');

subplot(1,2,2);
pzmap(sys{:});
axis equal;
axis([-1.5 1.5 -1.5 1.5]);

exportgraphics(gcf, 'part2_real.png', 'Resolution', 300);


% Sect 3: poles of eq. 4.104 fixed, zero z2 moved freely
r = 0.8;
theta = 30;
z2 = [-0.5 0 r*cosd(theta) 0.9 1.5];
sys = cell(size(z2));
for i = 1:numel(z2)
    sys{i} = H3(r, theta, z2(i));
    sys{i} = sys{i} / dcgain(sys{i});
end


figure('Position', [100 100 1200 450], 'Theme', 'light');
subplot(1,2,1);
step(sys{:}, t_end); grid on;
legend("z_2 = " + round(z2, 2));
%title('Part 3: poles fixed, zero moved (normalized)');

subplot(1,2,2);
pzmap(sys{:});
axis equal;
axis([-1.5 1.5 -1.5 1.5]);

exportgraphics(gcf, 'part3.png', 'Resolution', 300);


