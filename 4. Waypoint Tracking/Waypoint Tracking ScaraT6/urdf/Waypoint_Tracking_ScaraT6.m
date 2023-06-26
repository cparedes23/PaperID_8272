%----------------------------------------------------------
%---- Trajectory Simulation -----
%---- Waypoint Tracking Scara T6 -----
%----------------------------------------------------------

%% Waypoint tracking demostration using Robotics System Toolbox

% This demostration performs inverse kinematic of a robotic
% manipulator to follow a desired set of waypoints

clear all; close all; clc

%---------------------------------------------------------------------
% q1 and q2 are the initial and final articular coordinates
%---------------------------------------------------------------------
disp([' ']);
disp([' Vectors q1 and q2 of the joint coordinates ']);
disp([' initial and final. ']);


%% Load and display robot
robot = importrobot('Trayectory_ScaraT6.urdf')     % We import the robot
show(robot,'visuals','on','Frames','off')          % Visualizes the robot and hides the reference axes
%%
%---------------------------------------------------------------------
% Creation of a set of points
%---------------------------------------------------------------------
wayPoints = [0.600   0.450   0.050   0.150   0.500;    %[x]
             0       0.200   0.500   0.400   0.050;    %[y]
            -0.083  -0.083  -0.283  -0.083  -0.183]    %[z]

%---------------------------------------------------------------------
% Plot and Add label to point
%---------------------------------------------------------------------
hold on
plot3(wayPoints(1,:),wayPoints(2,:),wayPoints(3,:),'.','MarkerSize', 15,'MarkerEdgeColor', 'red')   % Plot points
hold on
plot3(wayPoints(1,:),wayPoints(2,:),wayPoints(3,:), 'LineWidth', 1, 'Color', 'blue')                % Plot points

% Naming the points
text(wayPoints(1,1)+0.15, wayPoints(2,1)-0.1, wayPoints(3,1)-0.02, 'Initial P.', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,2), wayPoints(2,2), wayPoints(3,2)+0.05, 'P2', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,3), wayPoints(2,3), wayPoints(3,3)+0.05, 'P3', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,4), wayPoints(2,4), wayPoints(3,4)+0.05, 'P4', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,5), wayPoints(2,5)+0.05, wayPoints(3,5)-0.05, 'Final P.', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');

%% Visualize robot configuraations
title('Sacara Robot - Path Tracking')
axis([-0.5 1 -0.5 1 -0.5 0.5]);