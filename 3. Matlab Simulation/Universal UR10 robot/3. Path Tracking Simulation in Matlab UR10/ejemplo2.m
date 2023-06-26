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

T1 = [ 0  -0.9336    -0.3584    1.1707;
      -1   0          0         0.1639;
       0   0.3584    -0.9336   -0.1617;  
       0   0          0         1     ] 

T2 = [ 0  -0.9945    -0.1045    1.1960;
      -1   0          0        -0.1639;
       0   0.1045    -0.9945    0.1499;  
       0   0          0         1     ] 
   
q1 = inversekinematic6(T1,1)
q2 = inversekinematic6(T2,1)

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

%---------------------------------------------------------------------
% graphs of results
%---------------------------------------------------------------------
figure
plot3([T1(1,4)  T2(1,4)], [T1(2,4)  T2(2,4)], [T1(3,4)  T2(3,4)] )
hold on
animacion6(pos')   

%---------------------------------------------------------------------
% SAVE DATA:
%---------------------------------------------------------------------
% Joint Positions
xlswrite('angulo_2023.cvs', [pos],'Hoja1');
n = length(pos);
for j=1:n
    q1_final(j,:)=[j pos(j,1)*180/pi];
    q2_final(j,:)=[j pos(j,2)*180/pi];
    q3_final(j,:)=[j pos(j,3)*180/pi];
    q4_final(j,:)=[j pos(j,4)*180/pi];
    q5_final(j,:)=[j pos(j,5)*180/pi];
    q6_final(j,:)=[j pos(j,6)*180/pi];
end
dlmwrite('pos1.txt',q1_final)
dlmwrite('pos2.txt',q2_final)
dlmwrite('pos3.txt',q3_final)
dlmwrite('pos4.txt',q4_final)
dlmwrite('pos5.txt',q5_final)
dlmwrite('pos6.txt',q6_final)
