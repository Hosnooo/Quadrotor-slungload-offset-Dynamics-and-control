% simulationtools/ref_fig8_rough.m

function R = ref_fig8_rough(t)
% Regular figure-8 reference with stronger horizontal acceleration.
% No z excitation.
%
% This keeps the trajectory visually clean, but excites drone roll/pitch
% by using a faster planar figure-8.

% Constant altitude
z0 = -0.55;

% Regular figure-8 parameters
A = 1.00;   % x amplitude [m]
B = 0.50;   % y amplitude [m]
w = 1.15;   % frequency [rad/s]
          % increase to 1.25 if you want more rotation
          % decrease to 0.95 if it is too aggressive

% Clean figure-8:
% x = A sin(wt)
% y = B sin(2wt)
x = zeros(1,7);
y = zeros(1,7);
z = zeros(1,7);

for k = 0:6
    x(k+1) = A*w^k*sin(w*t + k*pi/2);

    % y has frequency 2w
    y(k+1) = B*(2*w)^k*sin(2*w*t + k*pi/2);
end

% Constant z, all z derivatives are zero
z(1) = z0;

R = [
    x;
    y;
    z
];

end