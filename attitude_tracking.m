clear;

i1 = 100;
i2 = 75;
i3 = 80;

I = [i1 0 0; 0 i2 0; 0 0 i3];

sig0 = [0.1 0.2 -0.1]';
omega0 = deg2rad([30 10 -20])';

K = 5;
P = 10*eye(3);

T = 120;
dt = 0.001;

steps = T/dt;

sig_BN = sig0;
omega_BN = omega0;

skew = @(v) [0, -v(3), v(2);
             v(3), 0, -v(1);
             -v(2), v(1), 0];

V = zeros(steps, 1);
rates = zeros(steps, 3);
attitude = zeros(steps, 3);

B_mat = @(sig) (1-sig'*sig)*eye(3) + 2*skew(sig) + 2 * (sig * sig');

f = 0.05;
sig_RN0 = [0.2*sin(0);0.3*cos(0);-0.3*sin(0)];
sigdot_RN0 = [0.2*f*cos(0);-0.3*f*sin(0);-0.3*f*cos(0)];

omega_RN = 4*(B_mat(sig_RN0)\sigdot_RN0);
omega_RN_p = omega_RN;

for k = 1:steps
    t = k*dt;
    sig_RN = [0.2*sin(f*t); 0.3*cos(f*t); -0.3*sin(f*t)];
    sigdot_RN = [0.2*f*cos(f*t); -0.3*f*sin(f*t); -0.3*f*cos(f*t)];

    omega_RN_p = omega_RN;
    omega_RN = 4*(B_mat(sig_RN)\sigdot_RN);

    C_BN = MRP2DCM(sig_BN);
    C_RN = MRP2DCM(sig_RN);
    C_BR = C_BN*C_RN';

    sig_BR = DCM2MRP(C_BR);

    B_omega_RN = C_BR*omega_RN;
    omega_BR = omega_BN - B_omega_RN;
    
    omegadot_RN = (omega_RN - omega_RN_p)/dt;
    B_omegadot_RN = C_BR*omegadot_RN;
    u = -K*sig_BR - P*omega_BR + I*(B_omegadot_RN - skew(omega_BN)*B_omega_RN) + skew(omega_BN)*I*omega_BN;

    %euler dynamics
    omegadot_BN = I\(-skew(omega_BN)*I*omega_BN + u);
    %mrp changes
    sigdot_BN = 0.25 * B_mat(sig_BN) * omega_BN;

    omega_BN = omega_BN + omegadot_BN * dt;
    sig_BN = sig_BN + sigdot_BN * dt;

    if norm(sig_BN)>1
        sig_BN = -sig_BN/norm(sig_BN)^2;
    end

    lyap = @(omega, sigma) 0.5 * omega'*I*omega + 2*K*log(1 + sigma' * sigma);
    rates(k, :) = omega_BR';
    attitude(k, :) = sig_BR';
    V(k) = lyap(omega_BR, sig_BR);

    if t == 40
        norm(sig_BR)
    end

end