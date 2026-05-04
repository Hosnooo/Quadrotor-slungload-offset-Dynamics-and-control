function R = ref_fig8(t)
% Smooth figure-8 reference.
% Returns 3x7:
% [r, r_dot, r_ddot, r_3dot, r_4dot, r_5dot, r_6dot]

A = 0.75;
w = 0.35;

r1   = A*sin(w*t);
r1d  = A*w*cos(w*t);
r1dd = -A*w^2*sin(w*t);
r1d3 = -A*w^3*cos(w*t);
r1d4 = A*w^4*sin(w*t);
r1d5 = A*w^5*cos(w*t);
r1d6 = -A*w^6*sin(w*t);

r2   = 0.5*A*sin(2*w*t);
r2d  = A*w*cos(2*w*t);
r2dd = -2*A*w^2*sin(2*w*t);
r2d3 = -4*A*w^3*cos(2*w*t);
r2d4 = 8*A*w^4*sin(2*w*t);
r2d5 = 16*A*w^5*cos(2*w*t);
r2d6 = -32*A*w^6*sin(2*w*t);

r3   = -0.5;
r3d  = 0;
r3dd = 0;
r3d3 = 0;
r3d4 = 0;
r3d5 = 0;
r3d6 = 0;

R = [
    r1, r1d, r1dd, r1d3, r1d4, r1d5, r1d6;
    r2, r2d, r2dd, r2d3, r2d4, r2d5, r2d6;
    r3, r3d, r3dd, r3d3, r3d4, r3d5, r3d6
];
end