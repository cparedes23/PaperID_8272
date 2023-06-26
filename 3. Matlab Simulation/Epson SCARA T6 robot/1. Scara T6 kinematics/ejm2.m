%----------------------------------------------------------
%---- animation software test program -----
%---- animacion3 -----
%----------------------------------------------------------

clear all; close all; clc

%---------------------------------------------------------------------
% q1 and q2 are the initial and final articular coordinates
%---------------------------------------------------------------------
disp([' ']);
disp([' Vectors q1 and q2 of the joint coordinates ']);
disp([' initial and final. ']);

T1 = [1   0   0   0.2815;
      0   1   0   0.4375;
      0   0   1  -0.2   ;  
      0   0   0   1     ]

T2 = [1   0   0   0.500;
      0   1   0   0.200;
      0   0   1  -0.100;  
      0   0   0   1    ]
   
p1 = T1(1:3,4)
p2 = T2(1:3,4)
n  = [1 0 0]'
s  = [0 1 0]'
a  = [0 0 1]'

mat_q = planifica3(p1,p2,n,s,a,1,100);
animacion3(mat_q)
hold on
plot3([p1(1)  p2(1)], [p1(2)  p2(2)], [p1(3)  p2(3)] )