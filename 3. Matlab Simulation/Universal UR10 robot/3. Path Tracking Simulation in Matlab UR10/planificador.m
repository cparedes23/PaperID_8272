% PLANNER Path planner with 4-3-4 interpolator.
% [T,POS_PLAN, VEL_PLAN, ACE_PLAN]=PLANNIFIADOR(Q1,Q2) calculates the
% position, velocity and acceleration matrices of node points that
% represent the trajectory planning between points Q1 and Q2
% complying with the restrictions of smooth trajectory and performance of
% actuators.
% Use the 4-3-4 polynomials in the three path segments.
%

function [t, pos_plan, vel_plan, ace_plan] = planificador(q1,q2)

%**********************drive parameters**************

%---------------------------------------------------------------------
% Specifications of the starting and braking times of each motor.
%---------------------------------------------------------------------
tmotor = 0.1*ones(6,2);

%---------------------------------------------------------------------
%Maximum speeds of each motor.
%---------------------------------------------------------------------
%velmax in catalog -> [120; 120; 180; 180; 180; 180]   °/s
velmax = [2.0944; 2.0944; 3.1416; 3.1416; 3.1416; 3.1416];  %[rad/s, rad/s, m/s]

%**********************coordinated planner***********************

%---------------------------------------------------------------------
% Initialization of the position - velocity - acceleration vectors.
%---------------------------------------------------------------------
q = zeros(6,1);
q0 = [q1 q q];
qf = [q2 q q];

%---------------------------------------------------------------------
% Synchronization of the motors so that they carry out a coordinated movement
%---------------------------------------------------------------------
[velo2,tmaximo]=sincronizador(q0,qf,velmax);

%---------------------------------------------------------------------
%Initialization of time scale and matrices.
%---------------------------------------------------------------------
t = 0:0.01:(tmaximo+0.15);

% +0.15 is added in order to increase the time interval and sample
% the entire braking range of the articulation, assuming the
% approximations made in the SYNCHRONIZER function.

ini=zeros(length(t),1);
pos_plan(:,1)=ini;
vel_plan(:,1)=ini;
ace_plan(:,1)=ini;

%---------------------------------------------------------------------
% Calculation of the coefficients of the polynomials and evaluation of the
% interpolation polynomials.
%---------------------------------------------------------------------

for i = 1:6
[caso,A,tt] = calculocoef(i,velo2,q0,qf,tmotor);
posi=evalpos(t,tt,caso,A);
pos_plan(:,i)=posi';
ve=evalvel(t,tt,caso,A);
vel_plan(:,i)=ve';
ace=evalacel(t,tt,caso,A);
ace_plan(:,i)=ace';
end

return

end
