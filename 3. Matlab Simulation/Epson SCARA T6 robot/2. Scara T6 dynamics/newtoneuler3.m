%NEWTONEULER3 Inverse dynamics using the Newton-Euler method.
% TAU = NEWTONEULER4(Q, QP, QPP, G, M5, IEXTER) calculates the vector
%3x1 vector of input torques/forces to the joints. Q is the vector
%3x1 vector of joint coordinates. QP is the 3x1 vector representing
% the velocity of each joint. QPP is the 3x1 vector representing
% the acceleration of each joint. G is the value of gravity(m/s^2).
%M4 is the mass of the external load (kg) carried by the robot arm.
%IEXTER is the 3x3 matrix of inertia of the external load (Kg-m^2).
%

function tau = newtoneuler3(q,qp,qpp,g,m4,Iexter)

% ------------------------------------------------------------
% Denavit-Hartenberg robot parameters
% ------------------------------------------------------------
teta  = [q(1)    q(2)    0         ];
d     = [0       0      -q(3)-0.083];
a     = [0.325   0.275   0         ];
alfa  = [0       0       0         ];

% ------------------------------------------------------------
% Center of gravity positioning factors
% ------------------------------------------------------------
factor1 = -0.5; factor2 = -0.5; factor3 = -0.5; 

% ------------------------------------------------------------
% Mass of each element (Kg)
% ------------------------------------------------------------
m1 = 1.6797; m2 = 7.5355; m3 = 0.08317;

% ------------------------------------------------------------
% Viscous friction coefficient of each joint
% -----------------------------------------------
b1 = 0.05; b2 = 0.05; b3 = 0.05;

% ------------------------------------------------------------
% Inertia Matrices (Kg-m^2)
% ------------------------------------------------------------
r10I_r01 = [0.0020 0 -0.0017;0 0.0712 0;-0.0017 0 0.0725];
r20I_r02 = [0.1351 0 -0.1429;0 0.3854 0;-0.1429 0 0.2658];
r30I_r03 = [0.0051 0  0     ;0 0.0051 0; 0      0 0     ];

% ------------------------------------------------------------
% Vectors ri0pi, ri0si.
% ------------------------------------------------------------
r10p1 = [0.325; 0; -0.0029];
r20p2 = [0.275; 0; 0.0025];
r30p3 = [ 0   ; 0; -0.41 ];
r40p4 = zeros(3,1);
r10s1 = [-0.17756; 0; 0.00405];
r20s2 = [-0.15534; 0; 0.11710];
r30s3 = [ 0      ; 0; 0.207  ];
r40s4 = zeros(3,1);

% ------------------------------------------------------------
% Transformation matrices
% ------------------------------------------------------------
r01 = dh(teta(1), alfa(1)); r10 = r01';
r12 = dh(teta(2), alfa(2)); r21 = r12';
r23 = dh(teta(3), alfa(3)); r32 = r23';
r34 = eye(3);               r43 = r34';

% ------------------------------------------------------------
% Angular velocity of the joints
% ------------------------------------------------------------
r00w0 = zeros(3,1);
r10w1 = ri0wi(r10, r00w0, qp(1));
r20w2 = ri0wi(r21, r10w1, qp(2));
r30w3 = r32*r20w2;
r40w4 = ri0wi(r43, r30w3, 0);

% ------------------------------------------------------------
% Angular acceleration of joints
% ------------------------------------------------------------
r00wp0 = zeros(3,1);
r10wp1 = ri0wpi(r10, r00wp0, r00w0, qp(1), qpp(1));
r20wp2 = ri0wpi(r21, r10wp1, r10w1, qp(2), qpp(2));
r30wp3 = r32*r20wp2;
r40wp4 = ri0wpi(r43, r30wp3, r30w3, 0, 0);

% ------------------------------------------------------------
% Linear joint acceleration
% ------------------------------------------------------------
r00vp0 = [0; 0; g];
r10vp1 = ri0vpi_r(r10, r00vp0, r10wp1, r10w1, r10p1);
r20vp2 = ri0vpi_r(r21, r10vp1, r20wp2, r20w2, r20p2);
r30vp3 = ri0vpi_p(r32, r20vp2, r30wp3, r30w3, r30p3, qp(3), qpp(3));
r40vp4 = ri0vpi_r(r43, r30vp3, r40wp4, r40w4, r40p4);

% ------------------------------------------------------------
% Acceleration of the center of mass of each element
% ------------------------------------------------------------
r10a1 = ri0ai(r10vp1, r10wp1, r10w1, r10s1);
r20a2 = ri0ai(r20vp2, r20wp2, r20w2, r20s2);
r30a3 = ri0ai(r30vp3, r30wp3, r30w3, r30s3);
r40a4 = ri0ai(r40vp4, r40wp4, r40w4, r40s4);

% ------------------------------------------------------------
% Force at the center of mass of each element
% ------------------------------------------------------------
r40f4 = ri0fi(r40a4, m4);
r30f3 = ri0fi(r30a3, m3);
r20f2 = ri0fi(r20a2, m2);
r10f1 = ri0fi(r10a1, m1);

% ------------------------------------------------------------
% Torque at the center of mass of each eleme
% ------------------------------------------------------------
r40n4 = ri0ni(r40wp4, r40w4, Iexter);
r30n3 = ri0ni(r30wp3, r30w3, r30I_r03);
r20n2 = ri0ni(r20wp2, r20w2, r20I_r02);
r10n1 = ri0ni(r10wp1, r10w1, r10I_r01);

% ------------------------------------------------------------
%  Joint forces
% ------------------------------------------------------------
r40f4a = r40f4;
r30f3a = ri0fia(r34, r40f4a, r30f3);
r20f2a = ri0fia(r23, r30f3a, r20f2);
r10f1a = ri0fia(r12, r20f2a, r10f1);

% ------------------------------------------------------------
%  Joint forces
% ------------------------------------------------------------
r20p1 = r21*(r10p1); 
r30p2 = r32*(r20p2);
r40p3 = r43*(r30p3); 

r40n4a = r40n4;
r30n3a = ri0nia(r34, r40n4a, r40f4a, r30n3, r30f3, r40p3, r30p3, r30s3);
r20n2a = ri0nia(r23, r30n3a, r30f3a, r20n2, r20f2, r30p2, r20p2, r20s2);
r10n1a = ri0nia(r12, r20n2a, r20f2a, r10n1, r10f1, r20p1, r10p1, r10s1);

% ------------------------------------------------------------
% Actuator forces and torques
% ------------------------------------------------------------
t_1 = t_r(r10, r10n1a, qp(1), b1);
t_2 = t_r(r21, r20n2a, qp(2), b2);
t_3 = f_p(r32, r30f3a, qp(3), b3);

tau = [t_1; t_2; t_3];

end