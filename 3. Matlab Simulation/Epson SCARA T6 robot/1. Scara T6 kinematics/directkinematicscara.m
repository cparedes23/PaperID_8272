% DIRECTKINEMATICSCARA
% A03 = DIRECTKINEMATICSCARA(Q) return the transformation matrix from the 
% first coordinate system to the last one based on the vector Q.
%

function A03 = directkinematicscara(q)

% ------------------------------------------------------------
% Denavit-Hartenberg parameters of the robot
% ------------------------------------------------------------
teta  = [q(1)    q(2)    0         ];
d     = [0       0      -q(3)-0.083];
a     = [0.325   0.275   0         ];
alfa  = [0       0       0         ];

% Matrices of homogeneous transformation between consecutive coordinate 
% systems
A01 = denavit(teta(1), d(1), a(1), alfa(1));
A12 = denavit(teta(2), d(2), a(2), alfa(2));
A23 = denavit(teta(3), d(3), a(3), alfa(3));

% Transformation matrix from first to last coordinate system
A03 = A01*A12*A23;

end