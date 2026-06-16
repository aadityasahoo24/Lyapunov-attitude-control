t = (0:steps-1)*dt;

figure;

% MRPs
subplot(3,1,1)
plot(t, attitude(:,1), 'LineWidth', 1.2); hold on
plot(t, attitude(:,2), 'LineWidth', 1.2);
plot(t, attitude(:,3), 'LineWidth', 1.2);
grid on
xlabel('Time [s]')
ylabel('\sigma')
title('MRP Attitude')
legend('\sigma_1','\sigma_2','\sigma_3')

% Angular rates
subplot(3,1,2)
plot(t, rad2deg(rates(:,1)), 'LineWidth', 1.2); hold on
plot(t, rad2deg(rates(:,2)), 'LineWidth', 1.2);
plot(t, rad2deg(rates(:,3)), 'LineWidth', 1.2);
grid on
xlabel('Time [s]')
ylabel('\omega [deg/s]')
title('Body Angular Rates')
legend('\omega_1','\omega_2','\omega_3')

% Lyapunov function
subplot(3,1,3)
plot(t, V, 'k', 'LineWidth', 1.5)
grid on
xlabel('Time [s]')
ylabel('V')
title('Lyapunov Function')