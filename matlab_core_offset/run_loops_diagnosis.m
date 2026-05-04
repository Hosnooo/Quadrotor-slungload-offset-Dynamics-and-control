clear; clc; close all;

addpath(pwd);
addpath(fullfile(pwd,'simulationtools'));

% ==========================================================
% PARAMETERS
% ==========================================================

p = struct();

% Maple/generated-code names
p.m__Q = 1.0;
p.m__L = 0.2;
p.L    = 0.5;
p.g    = 9.81;

p.r = [0.25; 0; 0];
p.J = diag([0.02, 0.02, 0.04]);

p.r__1 = p.r(1);
p.r__2 = p.r(2);
p.r__3 = p.r(3);

p.J__1 = p.J(1,1);
p.J__2 = p.J(2,2);
p.J__3 = p.J(3,3);

p = add_tracking_gains_from_poles(p, {
    [-1.5 -1.5 -1.5 -1.5], ...
    [-1.5 -1.5 -1.5 -1.5], ...
    [-1.2 -1.2]
});

% New mapper / inner-loop names
p.mQ = p.m__Q;
p.mL = p.m__L;
p.JS = p.J;

p.kR     = diag([4.0, 4.0, 4.0]);
p.kOmega = diag([1.2, 1.2, 1.2]);

% ==========================================================
% HOVER / REGULATION STATE
% ==========================================================

t0 = 0;

xred = [
    0; 0; -0.5;     % xL
    0; 0; 1;        % q
    0; 0; 0;        % vL
    0; 0; 0         % omega
];

R0 = eye(3);
Omega0 = zeros(3,1);

x_full = [
    xred(1:3);
    xred(4:6);

    R0(1,1); R0(1,2); R0(1,3);
    R0(2,1); R0(2,2); R0(2,3);
    R0(3,1); R0(3,2); R0(3,3);

    xred(7:9);
    xred(10:12);
    Omega0
];

% ==========================================================
% REFERENCE
% ==========================================================

ref_fun = @ref_regulation_steps;

Rt = ref_fun(t0);

traj = make_traj_struct(Rt);
traj.psi = 0;

% ==========================================================
% OUTER LOOP
% ==========================================================

Ud = controller_QSFA_U(t0, xred, p, ref_fun);

% ==========================================================
% MAPPER
% ==========================================================

[cmd, aux_map] = mapper_QSFA_U_to_inner(t0, xred, Ud, traj, p);

% ==========================================================
% INNER LOOP
% ==========================================================

R_actual = R_from_full_state(x_full);
Omega_actual = x_full(22:24);

[uplant, aux_inner] = inner_loop_offset(R_actual, Omega_actual, cmd, p);

% ==========================================================
% FULL PLANT RESIDUAL
% ==========================================================

[fvec_full, G_full] = model_offset(x_full, p);

u_guess = uplant(:);
u_trim_lsq = -G_full\fvec_full;

res_guess = fvec_full + G_full*u_guess;
res_trim  = fvec_full + G_full*u_trim_lsq;

% ==========================================================
% SIGN CONVENTION CHECK
% ==========================================================

u_test_1 = [ cmd.u_t_d;  aux_inner.tau_b];
u_test_2 = [-cmd.u_t_d;  aux_inner.tau_b];
u_test_3 = [ cmd.u_t_d; -aux_inner.tau_b];
u_test_4 = [-cmd.u_t_d; -aux_inner.tau_b];

res_1 = fvec_full + G_full*u_test_1;
res_2 = fvec_full + G_full*u_test_2;
res_3 = fvec_full + G_full*u_test_3;
res_4 = fvec_full + G_full*u_test_4;

% ==========================================================
% REPORT
% ==========================================================

fprintf('\n==================================================\n');
fprintf('Integrated mapper + inner-loop diagnostic\n');
fprintf('==================================================\n');

fprintf('\nFiles:\n');
fprintf('controller: '); which controller_QSFA_U
fprintf('mapper:     '); which mapper_QSFA_U_to_inner
fprintf('inner loop: '); which inner_loop_offset
fprintf('model:      '); which model_offset

fprintf('\nOuter-loop command:\n');
disp('Ud ='); disp(Ud);

fprintf('\nMapper outputs:\n');
disp('cmd.F_d ='); disp(cmd.F_d);
fprintf('cmd.u_t_d = %.12f\n', cmd.u_t_d);
disp('cmd.Rd ='); disp(cmd.Rd);
disp('cmd.Omega_d ='); disp(cmd.Omega_d);
disp('cmd.dOmega_d ='); disp(cmd.dOmega_d);
disp('cmd.pddot_d ='); disp(cmd.pddot_d);
disp('cmd.eta_d ='); disp(cmd.eta_d);

fprintf('\nMapper consistency:\n');
fprintf('||q|| - 1                 = %.12e\n', norm(cmd.q)-1);
fprintf('omega^T q                 = %.12e\n', dot(cmd.omega,cmd.q));
fprintf('det(R_U_d)                = %.12e\n', aux_map.RU_det);
fprintf('||R_U_d^T R_U_d - I||     = %.12e\n', aux_map.RU_orthogonality_error);
fprintf('det(R_r_d)                = %.12e\n', aux_map.Rr_det);
fprintf('||R_r_d^T R_r_d - I||     = %.12e\n', aux_map.Rr_orthogonality_error);
fprintf('||skew(Rr^T Rrdot)|| check = %.12e\n', ...
    norm(aux_map.Omega_hat_d + aux_map.Omega_hat_d.', 'fro'));

fprintf('\nInner-loop outputs:\n');
disp('eR ='); disp(aux_inner.eR);
disp('eOmega ='); disp(aux_inner.eOmega);
disp('tau_b ='); disp(aux_inner.tau_b);
disp('tau_cm ='); disp(aux_inner.tau_cm);
disp('uplant ='); disp(uplant);

fprintf('\nFull plant trim residual:\n');
disp('u_trim_lsq = -G\\f ='); disp(u_trim_lsq);
fprintf('norm(f + G*u_guess) = %.12e\n', norm(res_guess));
fprintf('norm(f + G*u_trim)  = %.12e\n', norm(res_trim));

fprintf('\nSign convention residuals:\n');
fprintf('norm for [+u_t; +tau_b] = %.12e\n', norm(res_1));
fprintf('norm for [-u_t; +tau_b] = %.12e\n', norm(res_2));
fprintf('norm for [+u_t; -tau_b] = %.12e\n', norm(res_3));
fprintf('norm for [-u_t; -tau_b] = %.12e\n', norm(res_4));

fprintf('\nRelevant hover G rows:\n');
disp('G_full(18,:)  vL3dot row =');    disp(G_full(18,:));
disp('G_full(22,:)  Omega1dot row ='); disp(G_full(22,:));
disp('G_full(23,:)  Omega2dot row ='); disp(G_full(23,:));
disp('G_full(24,:)  Omega3dot row ='); disp(G_full(24,:));

% ==========================================================
% EXTRA ZERO-ERROR INNER-LOOP TEST
% ==========================================================

R_match = cmd.Rd;
Omega_match = R_match.'*cmd.Rd*cmd.Omega_d;

[uplant_match, inner_match] = inner_loop_offset(R_match, Omega_match, cmd, p);

fprintf('\nInner-loop zero-error synthetic test:\n');
fprintf('||eR||       = %.12e\n', norm(inner_match.eR));
fprintf('||eOmega||   = %.12e\n', norm(inner_match.eOmega));
disp('uplant_match ='); disp(uplant_match);

% ==========================================================
% LOCAL HELPERS
% ==========================================================

function traj = make_traj_struct(Rt)

traj.xL0 = Rt(:,1);
traj.xL1 = Rt(:,2);
traj.xL2 = Rt(:,3);
traj.xL3 = Rt(:,4);
traj.xL4 = Rt(:,5);
traj.xL5 = Rt(:,6);
traj.xL6 = Rt(:,7);

end

function R = R_from_full_state(x)

R = [
    x(7),  x(8),  x(9);
    x(10), x(11), x(12);
    x(13), x(14), x(15)
];

end