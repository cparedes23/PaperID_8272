%----------------------------------------------------------
%---- Trajectory Simulation -----
%---- Waypoint Tracking UR10 -----
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
robot = importrobot('Trayectory_UR10.urdf')     % We import the robot
show(robot,'visuals','on','Frames','off')       % Visualizes the robot and hides the reference axes

%%
%---------------------------------------------------------------------
% Creation of a set of points
%---------------------------------------------------------------------
%
wayPoints = [ 0.1757   1.150   0.900   0.026;  %[x]
             -0.1639  -0.164  -0.364  -0.870;  %[y]
              1.3902  -0.288   0.065  -0.447]  %[z]

%---------------------------------------------------------------------
% Plot and Add label to point
%---------------------------------------------------------------------
hold on
plot3(wayPoints(1,:),wayPoints(2,:),wayPoints(3,:),'.','MarkerSize', 15,'MarkerEdgeColor', 'red')   % Plot points

%---------------------------------------------------------------------
% Trayectory spline
%---------------------------------------------------------------------
trajectory = cscvn(wayPoints)   % Create a smooth curve from the Waypoints
fnplt(trajectory, 'b', 2);      % Plot trayectory spline

%---------------------------------------------------------------------
% Naming the points
%---------------------------------------------------------------------
text(wayPoints(1,1), wayPoints(2,1), wayPoints(3,1)+0.07, 'Initial P.', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,2), wayPoints(2,2), wayPoints(3,2)+0.07, 'P2', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,3), wayPoints(2,3), wayPoints(3,3)+0.07, 'P3', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');
text(wayPoints(1,4)-0.4, wayPoints(2,4)-0.15, wayPoints(3,4)-0.1, 'Final P.', 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'red');


%% Visualize robot configuraations
title('UR10 Robot - Path Tracking')
axis([-0.5 1.5 -1 0.5 -1 1.5]);
%}