function [cmd, aux] = mapper_QSFA_U_to_inner(t, xred, Ud, traj, params)
%MAPPER_QSFA_U_TO_INNER
% Integrated mapper from outer-loop U_d and nominal flatness trajectory
% to inner-loop desired commands.
%
% Inputs:
%   t      : time
%   xred   : [xL; q; vL; omega]
%   Ud     : outer-loop suspension-point virtual acceleration input
%   traj   : nominal load trajectory derivatives
%            traj.xL0   = x_L,r
%            traj.xL1   = dx_L,r
%            traj.xL2   = d2x_L,r
%            traj.xL3   = d3x_L,r
%            traj.xL4   = d4x_L,r
%            traj.xL5   = d5x_L,r
%            traj.xL6   = d6x_L,r
%            traj.psi   = yaw command
%
%   params : physical parameters
%            params.mQ
%            params.mL
%            params.L
%            params.g
%            params.r
%
% Outputs:
%   cmd.u_t_d
%   cmd.Rd
%   cmd.Omega_d
%   cmd.dOmega_d
%   cmd.pddot_d
%   cmd.F_d
%   cmd.eta_d

mQ = params.mQ;
mL = params.mL;
L  = params.L;
g  = params.g;
r  = params.r(:);

e3 = [0;0;1];

xred = xred(:);
Ud   = Ud(:);

xL    = xred(1:3);
q     = xred(4:6);
vL    = xred(7:9);
omega = xred(10:12);

q = q / norm(q);
omega = omega - q*dot(q, omega);

psi = traj.psi;

% ==========================================================
% 1) U_d-based suspension-point acceleration
% ==========================================================

pddot_U_d = g*e3 + Ud;

% ==========================================================
% 2) Reduced-model diagnostic cable force C_U,d
% ==========================================================

omega_sq = dot(omega, omega);

xLddot_U_d = ...
    g*e3 ...
    - L*omega_sq*q ...
    + dot(Ud, q)*q;

C_U_d = mL*(xLddot_U_d - g*e3);

% ==========================================================
% 3) Nominal flatness cable force and its derivatives
% ==========================================================

xL0 = traj.xL0(:);
xL1 = traj.xL1(:);
xL2 = traj.xL2(:);
xL3 = traj.xL3(:);
xL4 = traj.xL4(:);
xL5 = traj.xL5(:);
xL6 = traj.xL6(:);

T0 = mL*(xL2 - g*e3);
T1 = mL*xL3;
T2 = mL*xL4;
T3 = mL*xL5;
T4 = mL*xL6;

Td_scalar = -norm(T0);

% q_r,d = -T_d / ||T_d||
Q = unit_vector_derivatives({-T0, -T1, -T2, -T3, -T4}, 4);

q_r_d   = Q{1};
dq_r_d  = Q{2};
d2q_r_d = Q{3};
d3q_r_d = Q{4};
d4q_r_d = Q{5};

% ==========================================================
% 4) Nominal suspension-point derivatives
% ==========================================================

pddot_r_d = xL2 - L*d2q_r_d;
p3_r_d    = xL3 - L*d3q_r_d;
p4_r_d    = xL4 - L*d4q_r_d;

% ==========================================================
% 5) Nominal reference force for attitude-rate construction
% ==========================================================

F_r_d     = T0 + mQ*pddot_r_d - mQ*g*e3;
Fdot_r_d  = T1 + mQ*p3_r_d;
Fddot_r_d = T2 + mQ*p4_r_d;

% b3_r,d = -F_r,d / ||F_r,d||
B3 = unit_vector_derivatives({-F_r_d, -Fdot_r_d, -Fddot_r_d}, 2);

b3r_d   = B3{1};
db3r_d  = B3{2};
ddb3r_d = B3{3};

% ==========================================================
% 6) Nominal reference axes and their derivatives
% ==========================================================

[Rr_d, Rrdot_d, Rrddot_d, axes_r] = construct_axis_derivatives( ...
    F_r_d, b3r_d, db3r_d, ddb3r_d, psi);

eta_d = Rrddot_d * r;

Omega_hat_d = Rr_d.' * Rrdot_d;
Omega_hat_d = 0.5*(Omega_hat_d - Omega_hat_d.');

Omega_d = vee_map(Omega_hat_d);

dOmega_hat_d = Rrdot_d.'*Rrdot_d + Rr_d.'*Rrddot_d;
dOmega_hat_d = 0.5*(dOmega_hat_d - dOmega_hat_d.');

dOmega_d = vee_map(dOmega_hat_d);

% ==========================================================
% 7) U_d-based final force command
% ==========================================================

F_U_d = mQ*Ud + Td_scalar*q;

rho_U = norm(F_U_d);

if rho_U <= 1e-9
    error('mapper_QSFA_U_to_inner:singularForce', ...
          'F_U_d norm is too small to construct R_U_d.');
end

u_t_d = -rho_U;

b3U_d = F_U_d/u_t_d;
b3U_d = b3U_d / norm(b3U_d);

% ==========================================================
% 8) U_d-based commanded attitude
% ==========================================================

[RU_d, axes_U] = construct_attitude_from_force_axis(F_U_d, b3U_d, psi);

% ==========================================================
% 9) Pack inner-loop command
% ==========================================================

cmd.t = t;

cmd.u_t_d = u_t_d;
cmd.Rd = RU_d;
cmd.Omega_d = Omega_d;
cmd.dOmega_d = dOmega_d;

cmd.pddot_d = pddot_U_d;
cmd.F_d = F_U_d;
cmd.eta_d = eta_d;

cmd.Ud = Ud;
cmd.Td_scalar = Td_scalar;
cmd.C_U_d = C_U_d;

cmd.xL = xL;
cmd.vL = vL;
cmd.q = q;
cmd.omega = omega;

cmd.b1d = axes_U.b1;
cmd.b2d = axes_U.b2;
cmd.b3d = axes_U.b3;

% ==========================================================
% 10) Auxiliary outputs for debugging
% ==========================================================

aux.xL0 = xL0;
aux.xL1 = xL1;
aux.xL2 = xL2;
aux.xL3 = xL3;
aux.xL4 = xL4;
aux.xL5 = xL5;
aux.xL6 = xL6;

aux.T_d = T0;
aux.Td_scalar = Td_scalar;

aux.q_r_d = q_r_d;
aux.dq_r_d = dq_r_d;
aux.d2q_r_d = d2q_r_d;
aux.d3q_r_d = d3q_r_d;
aux.d4q_r_d = d4q_r_d;

aux.pddot_r_d = pddot_r_d;
aux.p3_r_d = p3_r_d;
aux.p4_r_d = p4_r_d;

aux.F_r_d = F_r_d;
aux.Fdot_r_d = Fdot_r_d;
aux.Fddot_r_d = Fddot_r_d;

aux.Rr_d = Rr_d;
aux.Rrdot_d = Rrdot_d;
aux.Rrddot_d = Rrddot_d;
aux.axes_r = axes_r;

aux.RU_d = RU_d;
aux.axes_U = axes_U;

aux.Omega_hat_d = Omega_hat_d;
aux.dOmega_hat_d = dOmega_hat_d;

aux.F_U_d = F_U_d;
aux.rho_U = rho_U;

aux.RU_orthogonality_error = norm(RU_d.'*RU_d - eye(3), 'fro');
aux.RU_det = det(RU_d);

aux.Rr_orthogonality_error = norm(Rr_d.'*Rr_d - eye(3), 'fro');
aux.Rr_det = det(Rr_d);

end

function [R, axes] = construct_attitude_from_force_axis(F, b3, psi)

Fx = F(1);
Fy = F(2);
Fz = F(3);

num = Fx*cos(psi) + Fy*sin(psi);

if abs(Fz) <= 1e-9
    error('mapper_QSFA_U_to_inner:singularTheta', ...
          'Fz is too small for the heading-angle construction.');
end

theta = atan(num/Fz);

h = [cos(theta); 0; -sin(theta)];

c = cross(b3, h);

if norm(c) <= 1e-9
    error('construct_attitude_from_force_axis:singularHeading', ...
          'Heading construction is singular.');
end

s = norm(c);

b1 = -cross(b3, c)/s;
b1 = b1 / norm(b1);

b2 = cross(b3, b1);
b2 = b2 / norm(b2);

R = [b1, b2, b3];

if norm(R.'*R - eye(3), 'fro') > 1e-6 || det(R) < 0.999
    error('construct_attitude_from_force_axis:badRotation', ...
          'Constructed attitude is not a valid rotation matrix.');
end

axes.b1 = b1;
axes.b2 = b2;
axes.b3 = b3;
axes.h = h;
axes.c = c;
axes.s = s;
axes.theta = theta;

end

function [R, Rdot, Rddot, axes] = construct_axis_derivatives(F, b3, db3, ddb3, psi)

Fx = F(1);
Fy = F(2);
Fz = F(3);

num = Fx*cos(psi) + Fy*sin(psi);

if abs(Fz) <= 1e-9
    error('mapper_QSFA_U_to_inner:singularTheta', ...
          'Fz is too small for the heading-angle construction.');
end

theta = atan(num/Fz);

h = [cos(theta); 0; -sin(theta)];

% Active reconstruction assumption:
% derivatives of h are neglected.
dh = zeros(3,1);
ddh = zeros(3,1);

c = cross(b3, h);

if norm(c) <= 1e-9
    error('construct_axis_derivatives:singularHeading', ...
          'Heading construction is singular.');
end

s = norm(c);

w = -cross(b3, c);

b1 = w/s;

dc = cross(db3, h) + cross(b3, dh);

ddc = cross(ddb3, h) ...
    + 2*cross(db3, dh) ...
    + cross(b3, ddh);

ds = dot(c, dc)/s;

dds = (dot(dc, dc) + dot(c, ddc) - ds^2)/s;

dw = -cross(db3, c) - cross(b3, dc);

ddw = -cross(ddb3, c) ...
      -2*cross(db3, dc) ...
      -cross(b3, ddc);

db1 = dw/s - (ds/s)*b1;

ddb1 = ddw/s ...
       -2*(ds/s)*db1 ...
       -(dds/s)*b1;

b2 = cross(b3, b1);

db2 = cross(db3, b1) + cross(b3, db1);

ddb2 = cross(ddb3, b1) ...
       + cross(b3, ddb1) ...
       + 2*cross(db3, db1);

R = [b1, b2, b3];

Rdot = [db1, db2, db3];

Rddot = [ddb1, ddb2, ddb3];

axes.b1 = b1;
axes.b2 = b2;
axes.b3 = b3;

axes.db1 = db1;
axes.db2 = db2;
axes.db3 = db3;

axes.ddb1 = ddb1;
axes.ddb2 = ddb2;
axes.ddb3 = ddb3;

axes.h = h;
axes.dh = dh;
axes.ddh = ddh;

axes.c = c;
axes.dc = dc;
axes.ddc = ddc;

axes.s = s;
axes.ds = ds;
axes.dds = dds;

axes.w = w;
axes.dw = dw;
axes.ddw = ddw;

axes.theta = theta;

end

function B = unit_vector_derivatives(A, N)
%UNIT_VECTOR_DERIVATIVES
% Computes derivatives of b = a/||a|| up to order N.
%
% A is a cell array:
%   A{1} = a
%   A{2} = adot
%   A{3} = addot
%   ...
%
% B returns:
%   B{1} = b
%   B{2} = bdot
%   B{3} = bddot
%   ...

if numel(A) < N+1
    error('unit_vector_derivatives:insufficientInput', ...
          'A must contain derivatives up to order N.');
end

for k = 1:N+1
    A{k} = A{k}(:);
end

s = zeros(N+1,1);

for n = 0:N
    val = 0;
    for k = 0:n
        val = val + nchoosek(n,k)*dot(A{k+1}, A{n-k+1});
    end
    s(n+1) = val;
end

rho = zeros(N+1,1);
rho(1) = sqrt(s(1));

if rho(1) <= 1e-12
    error('unit_vector_derivatives:zeroNorm', ...
          'Cannot normalize a near-zero vector.');
end

for n = 1:N
    val = 0;
    for k = 1:n-1
        val = val + nchoosek(n,k)*rho(k+1)*rho(n-k+1);
    end
    rho(n+1) = (s(n+1) - val)/(2*rho(1));
end

B = cell(N+1,1);
B{1} = A{1}/rho(1);

for n = 1:N
    val = A{n+1};
    for k = 1:n
        val = val - nchoosek(n,k)*rho(k+1)*B{n-k+1};
    end
    B{n+1} = val/rho(1);
end

end

function v = vee_map(S)

v = [S(3,2);
     S(1,3);
     S(2,1)];

end
