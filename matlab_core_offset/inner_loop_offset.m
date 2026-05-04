function [uplant, inner] = inner_loop_offset(R, Omega, cmd, params)
%INNER_LOOP_OFFSET
% Geometric inner-loop controller about the suspension point.

mQ = params.mQ;
g  = params.g;
JS = params.JS;
r  = params.r(:);

kR     = params.kR;
kOmega = params.kOmega;

e3 = [0;0;1];

R     = reshape(R,3,3);
Omega = Omega(:);

Rd        = cmd.Rd;
Omega_d   = cmd.Omega_d(:);
dOmega_d  = cmd.dOmega_d(:);
pddot_d   = cmd.pddot_d(:);
u_t_d     = cmd.u_t_d;

% Attitude error
eR = 0.5 * vee_map(Rd.'*R - R.'*Rd);

% Angular-velocity error
eOmega = Omega - R.'*Rd*Omega_d;

% Suspension-point torque command
tau_b = ...
    -kR*eR ...
    -kOmega*eOmega ...
    + cross(Omega, JS*Omega) ...
    - JS*(hat_map(Omega)*R.'*Rd*Omega_d - R.'*Rd*dOmega_d) ...
    - cross(r, R.'*(mQ*pddot_d)) ...
    + cross(r, R.'*(mQ*g*e3));

% Torque conversion to center of mass
tau_cm = tau_b + cross(r, [0;0;u_t_d]);

inner.eR = eR;
inner.eOmega = eOmega;
inner.tau_b = tau_b;
inner.tau_cm = tau_cm;
inner.u_t_d = u_t_d;

% Plant input
uplant = [u_t_d; tau_cm];

end

function S = hat_map(v)

v = v(:);

S = [  0    -v(3)   v(2);
      v(3)   0     -v(1);
     -v(2)  v(1)    0   ];

end

function v = vee_map(S)

v = [S(3,2);
     S(1,3);
     S(2,1)];

end