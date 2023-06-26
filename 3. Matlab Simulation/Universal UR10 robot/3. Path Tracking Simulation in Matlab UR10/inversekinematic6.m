% INVERSEKINEMATIC6
% Q = INVERSEKINEMATIC6(T, codo) returns joint values Q(q1,q2,q3,q4,q5,q6)
% having the values of the position P(X,Y,Z).
%

function q = inversekinematic6(T, codo)

P = T(1:3,4);
x = P(1);     y = P(2);    z = P(3);
nx = T(1,1); ny = T(2,1); nz = T(3,1);
ox = T(1,2); oy = T(2,2); oz = T(3,2);
ax = T(1,3); ay = T(2,3); az = T(3,3);

% Measure of the links
L1 = 0.09000;
L2 = 0.61290;
L3 = 0.57160;
L4 = 0.16389;
L5 = 0.11570;
L6 = 0.17570;

% ------------------------------------------------------------
% Denavit-Hartenberg parameters of the robot
% ------------------------------------------------------------
d     = [L1      0       0       L4      L5     L6  ];
a     = [0       L2      L3      0       0      0   ];
alfa  = [pi/2    0       0       pi/2   -pi/2   0   ];

% ------------------------------------------------------
% Calculation of joint q1
% ------------------------------------------------------
a = x-L6*ax;
b = L6*ay-y;
c = L4;

if codo == 1 % elbow down
    q1 = atan2(c,sqrt(a^2+b^2-c^2))-atan2(b,a);
else         % elbow up
    q1 = atan2(c,-sqrt(a^2+b^2-c^2))-atan2(b,a);
end

% ------------------------------------------------------
% Calculation of joint q5
% ------------------------------------------------------
s5 = sqrt((nx*sin(q1)-ny*cos(q1))^2+(ox*sin(q1)-oy*cos(q1))^2);
c5 = ax*sin(q1)-ay*cos(q1);

if codo == 1 % elbow down
    q5 = atan2(s5, c5);
else         % elbow up
    q5 = atan2(-s5, c5);
end

% ------------------------------------------------------
% Calculation of joint q6
% ------------------------------------------------------
s6 = (-ox*sin(q1)+oy*cos(q1))/sin(q5);
c6 = (nx*sin(q1)-ny*cos(q1))/sin(q5);

if codo == 1 % elbow down
    q6 = atan2(s6, c6);
else         % elbow up
    q6 = atan2(-s6, c6);
end

% ------------------------------------------------------
% Parameters calculated from A04(3:1,4)
% ------------------------------------------------------
px4 = x-L6*ax+L5*ox*cos(q6)+L5*nx*sin(q6);
py4 = y-L6*ay+L5*oy*cos(q6)+L5*ny*sin(q6);
pz4 = z-L6*az+L5*oz*cos(q6)+L5*nz*sin(q6);

l = sqrt(px4^2+py4^2);
r1 = sqrt(l^2-L4^2);
r2 = pz4-L1;

% ------------------------------------------------------
% Calculation of joint q3
% ------------------------------------------------------
c3 = (r1^2+r2^2-L2^2-L3^2)/(2*L2*L3);
s3 = sqrt(1-c3^2);

if codo == 1 % elbow down
    q3 = atan2(s3, c3);
else         % elbow up
    q3 = atan2(-s3, c3);
end

% ------------------------------------------------------
% Calculation of joint q2
% ------------------------------------------------------
alpha = atan2(r2,r1);
betha = atan2(L3*sin(q3),L2+L3*cos(q3));

if codo == 1 % elbow down
    q2 = alpha-betha;
else         % elbow up
    q2 = alpha+betha;
end

% ------------------------------------------------------
% Parameters calculated from A04(3,1) and A04(3,3)
% ------------------------------------------------------
s234 = nz*cos(q5)*cos(q6)-az*sin(q5)-oz*cos(q5)*sin(q6);
c234 = oz*cos(q6)+nz*sin(q6);

if codo == 1 % elbow down
    q234 = atan2(s234, c234);
else         % elbow up
    q234 = atan2(-s234, c234);
end

% ------------------------------------------------------
% Calculation of joint q4
% ------------------------------------------------------
q4 = q234-q3-q2;

% Vector of joint coordinates
q = [q1   q2   q3   q4   q5   q6]';

end