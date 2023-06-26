%----------------------------------------------------------
%---- Calculation of inverse kinematics by general method -----
%---- Matrix multiplication -----
%----------------------------------------------------------
clc, clear all, close all
%-----------------------------------------------------------------
syms L1 L2 L3 L4 L5 L6 q1 q2 q3 q4 q5 q6 pi
syms nx ox ax X
syms ny oy ay Y
syms nz oz az Z
%L1=0.18; L2=0.40; L3=0.18; L4=0.18; L5=0.18; L6=0.18;
%q1=0; q2=0; q3=0; q4=pi/2; q5=pi/2;q6=0;

% ------------------------------------------------------------
% Denavit-Hartenberg parameters of the robot
% ------------------------------------------------------------
teta  = [q1      q2      q3      q4     q5      q6];
d     = [L1      0       0       L4      L5     L6];
a     = [0       L2      L3      0       0      0 ];
alfa  = [pi/2    0       0       pi/2   -pi/2   0 ];

%-----------------------------------------------------------------
%T06=T
%
T06=[nx ox ax X;
   ny oy ay Y;
   nz oz az Z;
   0   0  0 1];
 %}  
%-----------------------------------------------------------------
%%Homogeneous Transformation
A01 = denavit(teta(1), d(1), a(1), alfa(1));
A12 = denavit(teta(2), d(2), a(2), alfa(2));
A23 = denavit(teta(3), d(3), a(3), alfa(3));
A34 = denavit(teta(4), d(4), a(4), alfa(4));
A45 = denavit(teta(5), d(5), a(5), alfa(5));
A56 = denavit(teta(6), d(6), a(6), alfa(6)); 

A06 = simplify(A01*A12*A23*A34*A45*A56)
%A06 = A01*A12*A23*A34*A45*A56

%%Inverse Homogeneous Transformation
A10=inv(A01);
A21=inv(A12);
A32=inv(A23);
A43=inv(A34);
A54=inv(A45);
A65=inv(A56);

%%Calculation of joints
%Calculation q1 -> [3,3] y [3,4]
%Calculation q5 -> [3,1] y [3,2]
%M1:
%A16=simplify(A10*A06)
%T16=simplify(A10*T06)


%A04=simplify(A06*A65*A54)
%T04=simplify(T06*A65*A54)


%drawrobot3d6
%A02=simplify(A06*A65*A54*A43*A32)
%T02=simplify(T06*A65*A54*A43*A32)


