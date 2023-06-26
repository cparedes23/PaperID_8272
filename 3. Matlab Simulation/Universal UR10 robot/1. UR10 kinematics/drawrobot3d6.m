% DRAWROBOT3D6 3D representation of a robot.
% DRAWROBOT3D6(Q) Performs a 3D representation of a robot as a function 
% of the vector of joint variables Q.
%

function drawrobot3d6(q)

% Size of the links
L1 = 0.09000;
L2 = 0.61290;
L3 = 0.57160;
L4 = 0.16389;
L5 = 0.11570;
L6 = 0.17570;

% Denavit-Hartenberg parameters of the robot
teta = q;
d     = [L1      0       0       L4      L5     L6  ];
a     = [0       L2      L3      0       0      0   ];
alfa  = [pi/2    0       0       pi/2   -pi/2   0   ];

% Matrices of homogeneous transformation between consecutive coordinate 
% systems
A01 = denavit(teta(1), d(1), a(1), alfa(1));
A12 = denavit(teta(2), d(2), a(2), alfa(2));
A23 = denavit(teta(3), d(3), a(3), alfa(3));
A34 = denavit(teta(4), d(4), a(4), alfa(4));
A45 = denavit(teta(5), d(5), a(5), alfa(5));
A56 = denavit(teta(6), d(6), a(6), alfa(6));

% Transformation matrix from the first to the last system to the 
% corresponding system
A02 = A01 * A12;
A03 = A02 * A23;
A04 = A03 * A34;
A05 = A04 * A45;
A06 = A05 * A56;

% Position vector (x, y, z) of each coordinate system
x0 = 0; y0 = 0; z0 = 0;
x1 = A01(1,4); y1 = A01(2,4); z1 = A01(3,4);
x2 = A02(1,4); y2 = A02(2,4); z2 = A02(3,4);
x3 = A03(1,4); y3 = A03(2,4); z3 = A03(3,4);
x4 = A04(1,4); y4 = A04(2,4); z4 = A04(3,4);
x5 = A05(1,4); y5 = A05(2,4); z5 = A05(3,4);
x6 = A06(1,4); y6 = A06(2,4); z6 = A06(3,4);

L = 0.16;
xi = L*sin(teta(1)); 
yi = -L*cos(teta(1)); 
zi = d(1);
xj = a(2)*cos(teta(1))*cos(teta(2))+L*sin(teta(1)); 
yj = a(2)*sin(teta(1))*cos(teta(2))-L*cos(teta(1));
zj = a(2)*sin(teta(2))+d(1);

% The robot is drawn
x = [x0 x1 xi xj x2 x3 x4 x5 x6];
y = [y0 y1 yi yj y2 y3 y4 y5 y6];
z = [z0 z1 zi zj z2 z3 z4 z5 z6];
plot3(x,y,z,'k');

% The axes are named and a grid is added
grid on;
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');

% Denavit Hartenberg coordinate system
if teta == [0 0 0 0 0 0]
    hold on;
    drawsistema3dh(x0,y0,z0,0);
    drawsistema3dh(x1,y1,z1,1);
    drawsistema3dh(x2,y2,z2,2);
    drawsistema3dh(x3,y3,z3,3);
    drawsistema3dh(x4,y4,z4,4);
    drawsistema3dh(x5,y5,z5,5);
    drawsistema3dh(x6,y6,z6,6);
end 

% Axis limits are established
axis([0 1.5 -1 1 -0.5 1]);

end