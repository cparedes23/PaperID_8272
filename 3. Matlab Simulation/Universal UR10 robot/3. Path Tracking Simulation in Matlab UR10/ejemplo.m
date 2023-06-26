%----------------------------------------------------------
%----planning software test program -----
%---- interpolation 4-3-4 -----
%----------------------------------------------------------

clear all; close all; clc

%---------------------------------------------------------------------
% q1 and q2 are the initial and final joint coordinates
%---------------------------------------------------------------------
disp([' ']);
disp([' Vectors q1 and q2 of the joint coordinates ']);
disp([' initial and final. ']);
  
q1 = rand(6,1)
q2 = rand(6,1)

%---------------------------------------------------------------------
% Call to the PLANIFICADOR function
%---------------------------------------------------------------------
[t,pos, vel, ace] = planificador(q1,q2);

%---------------------------------------------------------------------
% graphs of results
%---------------------------------------------------------------------
figure

subplot(1,3,1)
plot(t,pos)
grid
title('Position profile given by the planner')
xlabel('Time (seg)'), ylabel('Position (rad)')

subplot(1,3,2)
plot(t,vel)
grid
title('Velocity profile given by the planner')
xlabel('Time (seg)'), ylabel('Velocity (rad/seg)')

subplot(1,3,3)
plot(t,ace)
grid
title('Acceleration profile given by the planner')
xlabel('Time (seg)'), ylabel('Aceleration (rad/seg2)')
