function R = ref_spiral(t)
% Smooth spiral reference.
% Returns 3x7:
% [r, r_dot, r_ddot, r_3dot, r_4dot, r_5dot, r_6dot]

r0 = 0.05;
a = 0.015;
w = 0.35;
zref = -0.5;

rho = r0 + a*t;

c1 = cos(w*t);
s1 = sin(w*t);

r1   = rho*c1;
r1d  = a*c1 - rho*w*s1;
r1dd = -2*a*w*s1 - rho*w^2*c1;
r1d3 = -3*a*w^2*c1 + rho*w^3*s1;
r1d4 = 4*a*w^3*s1 + rho*w^4*c1;
r1d5 = 5*a*w^4*c1 - rho*w^5*s1;
r1d6 = -6*a*w^5*s1 - rho*w^6*c1;

r2   = rho*s1;
r2d  = a*s1 + rho*w*c1;
r2dd = 2*a*w*c1 - rho*w^2*s1;
r2d3 = -3*a*w^2*s1 - rho*w^3*c1;
r2d4 = -4*a*w^3*c1 + rho*w^4*s1;
r2d5 = 5*a*w^4*s1 + rho*w^5*c1;
r2d6 = 6*a*w^5*c1 - rho*w^6*s1;

r3   = zref;
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