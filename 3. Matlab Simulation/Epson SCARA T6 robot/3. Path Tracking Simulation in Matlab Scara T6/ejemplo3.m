%----------------------------------------------------------
%----planning software test program -----
%---- interpolation 4-3-4 -----
%----------------------------------------------------------

clear all; close all; clc

%---------------------------------------------------------------------
% Creation of a set of points
%---------------------------------------------------------------------
wayPoints = [0.600   0.450   0.050   0.150   0.500;    %[x]
             0       0.200   0.500   0.400   0.050;    %[y]
            -0.083  -0.083  -0.283  -0.083  -0.183]    %[z]

%---------------------------------------------------------------------
% Graph and Add Label to Point
%---------------------------------------------------------------------
figure
plot3(wayPoints(1,:),wayPoints(2,:),wayPoints(3,:),'.','MarkerSize', 10,'MarkerEdgeColor', 'red')   %plot points
hold on
plot3(wayPoints(1,:),wayPoints(2,:),wayPoints(3,:), 'LineWidth', 1, 'Color', 'blue')                %graph lines

% Name the points
text(wayPoints(1,1), wayPoints(2,1), wayPoints(3,1)+0.1, 'Initial P.', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,2), wayPoints(2,2), wayPoints(3,2)+0.1, 'P2', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,3), wayPoints(2,3), wayPoints(3,3)+0.1, 'P3', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,4), wayPoints(2,4), wayPoints(3,4)+0.1, 'P4', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,5), wayPoints(2,5), wayPoints(3,5)-0.1, 'Final P.', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');

%---------------------------------------------------------------------
% q1 and q2 are the initial and final joint coordinates
%---------------------------------------------------------------------
disp([' ']);
disp([' Vectors q1 and q2 of the joint coordinates ']);
disp([' initial and final. ']);

T1 = [1   0   0   wayPoints(1,1);
      0   1   0   wayPoints(2,1);
      0   0   1   wayPoints(3,1);  
      0   0   0   1             ]

T2 = [1   0   0   wayPoints(1,2);
      0   1   0   wayPoints(2,2);
      0   0   1   wayPoints(3,2);  
      0   0   0   1             ]

T3 = [1   0   0   wayPoints(1,3);
      0   1   0   wayPoints(2,3);
      0   0   1   wayPoints(3,3);  
      0   0   0   1             ]
  
T4 = [1   0   0   wayPoints(1,4);
      0   1   0   wayPoints(2,4);
      0   0   1   wayPoints(3,4);  
      0   0   0   1             ]

T5 = [1   0   0   wayPoints(1,5);
      0   1   0   wayPoints(2,5);
      0   0   1   wayPoints(3,5);  
      0   0   0   1             ]
   
q1 = inversekinematicscara(T1,1)
q2 = inversekinematicscara(T2,1)
q3 = inversekinematicscara(T3,1)
q4 = inversekinematicscara(T4,1)
q5 = inversekinematicscara(T5,1)

%---------------------------------------------------------------------
% Call to the PLANIFICADOR function
%---------------------------------------------------------------------
[t_12,pos_12, vel_12, ace_12] = planificador(q1,q2);
[t_23,pos_23, vel_23, ace_23] = planificador(q2,q3);
[t_34,pos_34, vel_34, ace_34] = planificador(q3,q4);
[t_45,pos_45, vel_45, ace_45] = planificador(q4,q5);

%---------------------------------------------------------------------
% We store all the times in the array "t"
%---------------------------------------------------------------------
n = length(t_12)+length(t_23)+length(t_34)+length(t_45);    %array size
t = zeros(1,n);                                             %matrix creation
for j=1:n
    t(1,j) = 0.01*j;
end

%---------------------------------------------------------------------
% We store all the positions in the array "pos"
%---------------------------------------------------------------------
n = length(pos_12)+length(pos_23)+length(pos_34)+length(pos_45);    %array size
pos = zeros(n,3);                                                   %matrix creation
for j=1:length(pos_12)
    pos(j,:) = pos_12(j,:);
end

for j=1:length(pos_23)
    pos(length(pos_12)+j,:) = pos_23(j,:);
end

for j=1:length(pos_34)
    pos(length(pos_12)+length(pos_23)+j,:) = pos_34(j,:);
end

for j=1:length(pos_45)
    pos(length(pos_12)+length(pos_23)+length(pos_34)+j,:) = pos_45(j,:);
end

%---------------------------------------------------------------------
% We store all the velocities in the "vel" matrix
%---------------------------------------------------------------------
n = length(vel_12)+length(vel_23)+length(vel_34)+length(vel_45);    %array size
vel = zeros(n,3);                                                   %matrix creation
for j=1:length(vel_12)
    vel(j,:) = vel_12(j,:);
end

for j=1:length(vel_23)
    vel(length(vel_12)+j,:) = vel_23(j,:);
end

for j=1:length(vel_34)
    vel(length(vel_12)+length(vel_23)+j,:) = vel_34(j,:);
end

for j=1:length(vel_45)
    vel(length(vel_12)+length(vel_23)+length(vel_34)+j,:) = vel_45(j,:);
end

%---------------------------------------------------------------------
% We store all the accelerations in the array "ace"
%---------------------------------------------------------------------
n = length(ace_12)+length(ace_23)+length(ace_34)+length(ace_45);    %array size
ace = zeros(n,3);                                                   %matrix creation
for j=1:length(ace_12)
    ace(j,:) = ace_12(j,:);
end

for j=1:length(ace_23)
    ace(length(ace_12)+j,:) = ace_23(j,:);
end

for j=1:length(ace_34)
    ace(length(ace_12)+length(ace_23)+j,:) = ace_34(j,:);
end

for j=1:length(ace_45)
    ace(length(ace_12)+length(ace_23)+length(ace_34)+j,:) = ace_45(j,:);
end

%---------------------------------------------------------------------
% Simulation of results
%---------------------------------------------------------------------
hold on
animacion3(pos') 
title('Trajectory of SCARA T6 robot')
grid on

%---------------------------------------------------------------------
% graphs of results
%---------------------------------------------------------------------
figure

subplot(1,3,1)
plot(t,pos)
grid
title('Position Profile')
xlabel('Time (seg)'), ylabel('Position (rad)')

subplot(1,3,2)
plot(t,vel)
grid
title('Speed Profile')
xlabel('Time (seg)'), ylabel('Speed (rad/seg)')

subplot(1,3,3)
plot(t,ace)
grid
title('Acceleration Profile')
xlabel('Time (seg)'), ylabel('Acceleration (rad/seg2)')

%---------------------------------------------------------------------
% SAVE DATA:
%---------------------------------------------------------------------
% Joint Positions
xlswrite('angulo_2023.cvs', [pos],'Hoja1');
for j=1:n
    q1_final(j,:)=[j pos(j,1)*180/pi];
    q2_final(j,:)=[j pos(j,2)*180/pi];
    q3_final(j,:)=[j pos(j,3)*1000];
end
dlmwrite('pos1.txt',q1_final)
dlmwrite('pos2.txt',q2_final)
dlmwrite('pos3.txt',q3_final)