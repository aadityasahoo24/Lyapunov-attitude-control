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

sig = sig0;
omega = omega0;

skew = @(v) [0, -v(3), v(2);
             v(3), 0, -v(1);
             -v(2), v(1), 0];

V = zeros(steps, 1);
rates = zeros(steps, 3);
attitude = zeros(steps, 3);

for k = 1:steps
     if norm(sig)^2 > 1
         sig = - sig / (norm(sig)^2);
     end

    lyap = @(omega, sigma) 0.5 * omega'*I*omega + 2*K*log(1 + sigma' * sigma);
    rates(k, :) = omega';
    attitude(k, :) = sig';
    V(k) = lyap(omega, sig);
        
    B_sig = ((1 - norm(sig)^2)*eye(3) + 2*skew(sig) + 2*(sig*sig'));

    u = -K*sig - P* omega + skew(omega)*I*omega;

    omegadot = (eye(3)/I)*(-skew(omega)*I*omega + u);

    sigmadot = 0.25 * B_sig * omega;

    omega = omega + dt*omegadot;
    sig = sig + dt*sigmadot;

    if k == 30/dt
        norm(sig)
    end

end