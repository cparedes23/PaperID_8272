% INVERSEKINEMATICSCARA
% Q = INVERSEKINEMATICSCARA(P) returns joint values ​​Q(q1,q2,q3)
% having the values ​​of the position P(X,Y,Z).
%

function q = inversekinematicscara(T, codo)
P = T(1:3,4);
x = P(1); y = P(2); z = P(3);

% ------------------------------------------------------------
% Denavit-Hartenberg parameters of the robot
% ------------------------------------------------------------
a     = [0.325   0.275   0   ];
alfa  = [0       0       0   ];

% ------------------------------------------------------
% Calculation of joint q1
% ------------------------------------------------------
E = 2*x*a(1);
F = 2*y*a(1);
G = (a(2))^2-(a(1))^2-x^2-y^2;

t11 = (-F+sqrt(E^2+F^2-G^2))/(G-E);
t12 = (-F-sqrt(E^2+F^2-G^2))/(G-E);

if codo == 1 % elbow down
    q1 = 2*atan(t11);
else         % elbow up
    q1 = 2*atan(t12);
end

% ------------------------------------------------------
% Calculation of joint q2
% ------------------------------------------------------
cosq2 = (x^2+y^2-(a(1))^2-(a(2))^2)/(2*a(1)*a(2));

if codo == 1 % elbow down
    q2 = atan(sqrt(1-cosq2^2)/cosq2);
else         % elbow up
    q2 = atan(-sqrt(1-cosq2^2)/cosq2);
end

% ------------------------------------------------------
% Calculation of joint q3
% ------------------------------------------------------
q3 = -z-0.083;

% Vector of joint coordinates
q = [q1   q2   q3]';

end