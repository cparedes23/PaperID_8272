% NEWTONEULER6 Inverse dynamics using the Newton-Euler method.
% TAU = NEWTONEULER4(Q, QP, QPP, G, M5, IEXTER) calculates the vector
% 6x1 vector of input torques/forces to the joints.
%.............................................................................
clc,clear all, close all
syms q1 q2 q3 q4 q5 q6 qp1 qp2 qp3 qp4 qp5 qp6 qpp1 qpp2 qpp3 qpp4 qpp5 qpp6 
syms mexter Iexter 
assume(q1,'real')
assume(q2,'real')
assume(q3,'real')
assume(q4,'real')
assume(q5,'real')
assume(q6,'real')

assume(qp1,'real')
assume(qp2,'real')
assume(qp3,'real')
assume(qp4,'real')
assume(qp5,'real')
assume(qp6,'real')

assume(qpp1,'real')
assume(qpp2,'real')
assume(qpp3,'real')
assume(qpp4,'real')
assume(qpp5,'real')
assume(qpp6,'real')

assume(mexter,'real')
assume(Iexter,'real')

g=9.81;

% ------------------------------------------------------------
% Denavit-Hartenberg robot parameters
% ------------------------------------------------------------
teta  = [q1       q2       q3       q4       q5       q6     ];
d     = [0.09000  0        0        0.16389  0.11570  0.17570];
a     = [0        0.61290  0.57160  0        0        0      ];
alfa  = [pi/2     0        0        pi/2    -pi/2     0      ];

% ------------------------------------------------------------
% Mass of each element (Kg)
% ------------------------------------------------------------
m1 = 3.1350; m2 = 8.8872; m3 = 4.7466;
m4 = 0.7431; m5 = 0.7431; m6 = 0.4078;

% ------------------------------------------------------------
% Viscous friction coefficient of each joint
% ------------------------------------------------------------
b1 = 0; b2 = 0; b3 = 0; 
b4 = 0; b5 = 0; b6 = 0;

% ------------------------------------------------------------
% Inertia Matrices (Kg-m^2)
% ------------------------------------------------------------
r10I_r01 = [0.0125 0  0     ; 0 0.0107  0.0005;  0       0.0005 0.0012];
r20I_r02 = [0.2785 0 -0.5435; 0 1.8853  0     ; -0.5435  0      1.6255];
r30I_r03 = [0.0187 0 -0.0767; 0 0.6860  0     ; -0.0767  0      0.6731];
r40I_r04 = [0.0013 0  0     ; 0 0.0011  0     ;  0       0      0.0009];
r50I_r05 = [0.0013 0  0     ; 0 0.0011  0     ;  0       0      0.0009];
r60I_r06 = [0.0032 0  0     ; 0 0.0034 -0.0002;  0      -0.0002 0.0012];

% ------------------------------------------------------------
% Vectors ri0pi, ri0si.
% ------------------------------------------------------------
r10p1 = [ 0      ;  0.09   ;  0      ];
r20p2 = [ 0.61290;  0      ;  0      ];
r30p3 = [ 0.57160;  0      ;  0      ];
r40p4 = [ 0      ;  0.16389;  0      ];
r50p5 = [ 0      ; -0.11570;  0      ];
r60p6 = [ 0      ;  0      ;  0.17570];
r70p7 = zeros(3,1);
r10s1 = [ 0      ; -0.01152;  0.01162];
r20s2 = [-0.36185;  0      ;  0.17034];
r30s3 = [-0.31467;  0      ;  0.05142];
r40s4 = [ 0      ; -0.00760;  0.00974];
r50s5 = [ 0      ;  0.00760;  0.00974];
r60s6 = [-0.00001;  0.00477; -0.07921];
r70s7 = zeros(3,1);

% ------------------------------------------------------------
% Transformation matrices
% ------------------------------------------------------------
r01 = dh(teta(1), alfa(1)); r10 = r01';
r12 = dh(teta(2), alfa(2)); r21 = r12';
r23 = dh(teta(3), alfa(3)); r32 = r23';
r34 = dh(teta(4), alfa(4)); r43 = r34';
r45 = dh(teta(5), alfa(5)); r54 = r45';
r56 = dh(teta(6), alfa(6)); r65 = r56';
r67 = eye(3); r76 = r67';

% ------------------------------------------------------------
% Angular velocity of the joints
% ------------------------------------------------------------
r00w0 = zeros(3,1);
r10w1 = ri0wi(r10, r00w0, qp1);
r20w2 = ri0wi(r21, r10w1, qp2);
r30w3 = ri0wi(r32, r20w2, qp3);
r40w4 = ri0wi(r43, r30w3, qp4);
r50w5 = ri0wi(r54, r40w4, qp5);
r60w6 = ri0wi(r65, r50w5, qp6);
r70w7 = ri0wi(r76, r60w6, 0);

% ------------------------------------------------------------
% Angular acceleration of joints
% ------------------------------------------------------------
r00wp0 = zeros(3,1);
r10wp1 = ri0wpi(r10, r00wp0, r00w0, qp1, qpp1);
r20wp2 = ri0wpi(r21, r10wp1, r10w1, qp2, qpp2);
r30wp3 = ri0wpi(r32, r20wp2, r20w2, qp3, qpp3);
r40wp4 = ri0wpi(r43, r30wp3, r30w3, qp4, qpp4);
r50wp5 = ri0wpi(r54, r40wp4, r40w4, qp5, qpp5);
r60wp6 = ri0wpi(r65, r50wp5, r50w5, qp6, qpp6);
r70wp7 = ri0wpi(r76, r60wp6, r60w6, 0, 0);

% ------------------------------------------------------------
% Linear joint acceleration
% ------------------------------------------------------------
r00vp0 = [0; 0; g];
r10vp1 = ri0vpi_r(r10, r00vp0, r10wp1, r10w1, r10p1);
r20vp2 = ri0vpi_r(r21, r10vp1, r20wp2, r20w2, r20p2);
r30vp3 = ri0vpi_r(r32, r20vp2, r30wp3, r30w3, r30p3);
r40vp4 = ri0vpi_r(r43, r30vp3, r40wp4, r40w4, r40p4);
r50vp5 = ri0vpi_r(r54, r40vp4, r50wp5, r50w5, r50p5);
r60vp6 = ri0vpi_r(r65, r50vp5, r60wp6, r60w6, r60p6);
r70vp7 = ri0vpi_r(r76, r60vp6, r70wp7, r70w7, r70p7);

% ------------------------------------------------------------
% Acceleration of the center of mass of each element
% ------------------------------------------------------------
r10a1 = ri0ai(r10vp1, r10wp1, r10w1, r10s1);
r20a2 = ri0ai(r20vp2, r20wp2, r20w2, r20s2);
r30a3 = ri0ai(r30vp3, r30wp3, r30w3, r30s3);
r40a4 = ri0ai(r40vp4, r40wp4, r40w4, r40s4);
r50a5 = ri0ai(r50vp5, r50wp5, r50w5, r50s5);
r60a6 = ri0ai(r60vp6, r60wp6, r60w6, r60s6);
r70a7 = ri0ai(r70vp7, r70wp7, r70w7, r70s7);

% ------------------------------------------------------------
% Force at the center of mass of each element
% ------------------------------------------------------------
r70f7 = ri0fi(r70a7, mexter);
r60f6 = ri0fi(r60a6, m6);
r50f5 = ri0fi(r50a5, m5);
r40f4 = ri0fi(r40a4, m4);
r30f3 = ri0fi(r30a3, m3);
r20f2 = ri0fi(r20a2, m2);
r10f1 = ri0fi(r10a1, m1);

% ------------------------------------------------------------
% Torque at the center of mass of each element
% ------------------------------------------------------------
r70n7 = ri0ni(r70wp7, r70w7, Iexter);
r60n6 = ri0ni(r60wp6, r60w6, r60I_r06);
r50n5 = ri0ni(r50wp5, r50w5, r50I_r05);
r40n4 = ri0ni(r40wp4, r40w4, r40I_r04);
r30n3 = ri0ni(r30wp3, r30w3, r30I_r03);
r20n2 = ri0ni(r20wp2, r20w2, r20I_r02);
r10n1 = ri0ni(r10wp1, r10w1, r10I_r01);

% ------------------------------------------------------------
% Joint forces
% ------------------------------------------------------------
r70f7a = r70f7;
r60f6a = ri0fia(r67, r70f7a, r60f6);
r50f5a = ri0fia(r56, r60f6a, r50f5);
r40f4a = ri0fia(r45, r50f5a, r40f4);
r30f3a = ri0fia(r34, r40f4a, r30f3);
r20f2a = ri0fia(r23, r30f3a, r20f2);
r10f1a = ri0fia(r12, r20f2a, r10f1);

% ------------------------------------------------------------
% Joint pairs
% ------------------------------------------------------------
r20p1 = r21*(r10p1); r30p2 = r32*(r20p2);
r40p3 = r43*(r30p3); r50p4 = r54*(r40p4);
r60p5 = r65*(r50p5); r70p6 = r76*(r60p6);
r70n7a = r70n7;
r60n6a = ri0nia(r67, r70n7a, r70f7a, r60n6, r60f6, r70p6, r60p6, r60s6);
r50n5a = ri0nia(r56, r60n6a, r60f6a, r50n5, r50f5, r60p5, r50p5, r50s5);
r40n4a = ri0nia(r45, r50n5a, r50f5a, r40n4, r40f4, r50p4, r40p4, r40s4);
r30n3a = ri0nia(r34, r40n4a, r40f4a, r30n3, r30f3, r40p3, r30p3, r30s3);
r20n2a = ri0nia(r23, r30n3a, r30f3a, r20n2, r20f2, r30p2, r20p2, r20s2);
r10n1a = ri0nia(r12, r20n2a, r20f2a, r10n1, r10f1, r20p1, r10p1, r10s1);

% ------------------------------------------------------------
% Forces and drive torques
% ------------------------------------------------------------
t_1 = simplify(t_r(r10, r10n1a, qp1, b1))
t_2 = simplify(t_r(r21, r20n2a, qp2, b2));
t_3 = simplify(t_r(r32, r30n3a, qp3, b3));
t_4 = simplify(t_r(r43, r40n4a, qp4, b4));
t_5 = simplify(t_r(r54, r50n5a, qp5, b5));
t_6 = simplify(t_r(r65, r60n6a, qp6, b6));

tau = [t_1; t_2; t_3; t_4; t_5; t_6];

vpa(tau,3)      %Redondea la parte numerica de la ecuacion a 3 decimales