% DRAWROBOT3Dscara 3D representation of a robot.
% DRAWROBOT3D4(Q) Performs a 3D representation of a robot as a function 
% of the vector of joint variables Q.
%

function drawrobot3dscara(q)

% Denavit-Hartenberg parameters of the robot
teta  = [q(1)    q(2)    0         ];
d     = [0       0      -q(3)-0.083];
a     = [0.325   0.275   0         ];
alfa  = [0       0       0         ];

% Matrices of homogeneous transformation between consecutive coordinate 
% systems
A01 = denavit(teta(1), d(1), a(1), alfa(1));
A12 = denavit(teta(2), d(2), a(2), alfa(2));
A23 = denavit(teta(3), d(3), a(3), alfa(3));

% Transformation matrix from the first to the last system to the 
% corresponding system
A02 = A01*A12;
A03 = A02*A23;

% Position vector (x, y, z) of each coordinate system
x0 = 0;             y0 = 0;              z0 = 0;
x1 = A01(1,4);      y1 = A01(2,4);       z1 = A01(3,4);
x2 = A02(1,4);      y2 = A02(2,4);       z2 = A02(3,4);
x3 = A03(1,4);      y3 = A03(2,4);       z3 = A03(3,4);

% The robot is drawn
x = [x0  x1   x2  x3];
y = [y0  y1   y2  y3];
z = [z0  z1   z2  z3];
plot3(x,y,z);

% The axes are named and a grid is added
xlabel('x axis')
ylabel('y axis')
zlabel('z axis')
grid;

% Axis limits are established
axis([0  1  -1  1  -1  1]);

end