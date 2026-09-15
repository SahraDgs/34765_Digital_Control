
dd = load('C:\Users\sahra\Documents\master_year2\First_semester\34765_Digital_Control\week1_exercices\logs\device-monitor-260902-110803.log'); % use the filename and directory of your logfile.

% added by me
b = find(diff(dd(:,1)) < 0);

s = b(end-1) + 1;                   % start of run two before the last
e = b(end);                       % end of that run
r = s:e;

h = figure(1000);
hold off
plot(dd(r,1), dd(r,8))
hold on
plot(dd(r,1), dd(r,9))
plot(dd(r,1), dd(r,13))  % column with (most) vertical gyro data
grid on
legend('Velocity left (rad/s)', 'Velocity right (rad/s)', 'Turnrate (??)')
xlabel('Time (sec)')