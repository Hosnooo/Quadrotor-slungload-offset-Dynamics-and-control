function R = ref_regulation_steps(t)
% Piecewise-constant regulation mission.
% Returns 3x7:
% [r, r_dot, r_ddot, r_3dot, r_4dot, r_5dot, r_6dot]
%
% No smoothing. Between switching times the target is constant,
% so all derivatives are zero.

t_sw = [5, 20, 35, 50];

P = [
     0.00,  0.00, -0.5;
     0.00,  0.25, -0.5;
     0.25,  0.25, -1.0;
     0.25,  0.00, -1.0;
     0.00,  0.00, -0.5
];

idx = 1 + sum(t >= t_sw);
r = P(idx,:).';

R = [
    r(1), 0, 0, 0, 0, 0, 0;
    r(2), 0, 0, 0, 0, 0, 0;
    r(3), 0, 0, 0, 0, 0, 0
];
end