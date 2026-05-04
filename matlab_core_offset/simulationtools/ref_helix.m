function R = ref_helix(t)
% Smooth helical reference.
% Returns 3x7:
% [r, r_dot, r_ddot, r_3dot, r_4dot, r_5dot, r_6dot]

Rxy = 0.35;
w = 0.35;
vz = 0.02;
z0 = -0.5;

r1   = Rxy*cos(w*t);
r1d  = -Rxy*w*sin(w*t);
r1dd = -Rxy*w^2*cos(w*t);
r1d3 = Rxy*w^3*sin(w*t);
r1d4 = Rxy*w^4*cos(w*t);
r1d5 = -Rxy*w^5*sin(w*t);
r1d6 = -Rxy*w^6*cos(w*t);

r2   = Rxy*sin(w*t);
r2d  = Rxy*w*cos(w*t);
r2dd = -Rxy*w^2*sin(w*t);
r2d3 = -Rxy*w^3*cos(w*t);
r2d4 = Rxy*w^4*sin(w*t);
r2d5 = Rxy*w^5*cos(w*t);
r2d6 = -Rxy*w^6*sin(w*t);

r3   = z0 + vz*t;
r3d  = vz;
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