function R = ref_fig8_zsin(t)
% Smooth figure-8 in x-y with sinusoidal motion in z.
% Returns 3x7:
% [r, r_dot, r_ddot, r_3dot, r_4dot, r_5dot, r_6dot]

Axy = 0.75;
wxy = 0.35;

Az = 0.08;
wz = 2.0;
z0 = -0.5;

r1   = Axy*sin(wxy*t);
r1d  = Axy*wxy*cos(wxy*t);
r1dd = -Axy*wxy^2*sin(wxy*t);
r1d3 = -Axy*wxy^3*cos(wxy*t);
r1d4 = Axy*wxy^4*sin(wxy*t);
r1d5 = Axy*wxy^5*cos(wxy*t);
r1d6 = -Axy*wxy^6*sin(wxy*t);

r2   = 0.5*Axy*sin(2*wxy*t);
r2d  = Axy*wxy*cos(2*wxy*t);
r2dd = -2*Axy*wxy^2*sin(2*wxy*t);
r2d3 = -4*Axy*wxy^3*cos(2*wxy*t);
r2d4 = 8*Axy*wxy^4*sin(2*wxy*t);
r2d5 = 16*Axy*wxy^5*cos(2*wxy*t);
r2d6 = -32*Axy*wxy^6*sin(2*wxy*t);

r3   = z0 + Az*sin(wz*t);
r3d  = Az*wz*cos(wz*t);
r3dd = -Az*wz^2*sin(wz*t);
r3d3 = -Az*wz^3*cos(wz*t);
r3d4 = Az*wz^4*sin(wz*t);
r3d5 = Az*wz^5*cos(wz*t);
r3d6 = -Az*wz^6*sin(wz*t);

R = [
    r1, r1d, r1dd, r1d3, r1d4, r1d5, r1d6;
    r2, r2d, r2dd, r2d3, r2d4, r2d5, r2d6;
    r3, r3d, r3dd, r3d3, r3d4, r3d5, r3d6
];
end