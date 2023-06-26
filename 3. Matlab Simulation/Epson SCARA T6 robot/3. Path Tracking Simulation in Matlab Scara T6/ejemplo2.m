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

T1 = [1   0   0   0.2815;
      0   1   0   0.4375;
      0   0   1  -0.2   ;  
      0   0   0   1     ]

T2 = [1   0   0   0.500;
      0   1   0   0.200;
      0   0   1  -0.100;  
      0   0   0   1    ]
   
q1 = inversekinematicscara(T1,1)
q2 = inversekinematicscara(T2,1)

%---------------------------------------------------------------------
% Call to the PLANIFICADOR function
%---------------------------------------------------------------------
[t, pos, vel, ace] = planificador(q1,q2);

%---------------------------------------------------------------------
% graphs of results
%---------------------------------------------------------------------
figure

subplot(1,3,1)
plot(t,pos)
grid
title('position profile')
xlabel('Time (seg)'), ylabel('Position (rad)')

subplot(1,3,2)
plot(t,vel)
grid
title('Speed ​​Profile')
xlabel('Time (seg)'), ylabel('Speed (rad/seg)')

subplot(1,3,3)
plot(t,ace)
grid
title('Acceleration Profile')
xlabel('Time (seg)'), ylabel('Acceleration (rad/seg2)')

%---------------------------------------------------------------------
% graphs of results
%---------------------------------------------------------------------
figure
plot3([T1(1,4)  T2(1,4)], [T1(2,4)  T2(2,4)], [T1(3,4)  T2(3,4)] )
hold on
animacion3(pos')    

%---------------------------------------------------------------------
% SAVE DATA:
%---------------------------------------------------------------------
% Joint Positions
xlswrite('angulo_2023.cvs', [pos],'Hoja1');
n = length(pos);
for j=1:n
    q1_final(j,:)=[j pos(j,1)*180/pi];
    q2_final(j,:)=[j pos(j,2)*180/pi];
    q3_final(j,:)=[j pos(j,3)*1000];
end
dlmwrite('pos1.txt',q1_final)
dlmwrite('pos2.txt',q2_final)
dlmwrite('pos3.txt',q3_final)