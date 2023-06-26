% DIRECTKINEMATIC6
% A06 = DIRECTKINEMATIC6(Q) return the transformation matrix from the 
% first coordinate system to the last one based on the vector Q.
%

function A06 = directkinematic6(q)

% Size of the links
L1 = 0.09000;
L2 = 0.61290;
L3 = 0.57160;
L4 = 0.16389;
L5 = 0.11570;
L6 = 0.17570;

% ------------------------------------------------------------
% Denavit-Hartenberg parameters of the robot
% ------------------------------------------------------------
teta  = [q(1)   q(2)     q(3)    q(4)    q(5)   q(6)];
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

% Transformation matrix from first to last coordinate system
A06 = A01*A12*A23*A34*A45*A56;

end