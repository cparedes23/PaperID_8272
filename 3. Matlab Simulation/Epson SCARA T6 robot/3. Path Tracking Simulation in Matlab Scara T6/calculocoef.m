% CALCULOCEF Function for calculating the coefficients of the 4-3-4 polynomials.
% [CASE, A ,TT]=CALCULOCOEF(ELEM,VEL,Q0,QF,TMOTOR) calculates the coefficients
% of the 4-3-4 interpolated polynomials based on the specifications of
% actuators and start and end points.
% The 5x3 matrix A contains the coefficients of the interpolated polynomials.
%TT is the 3x1 vector of the acceleration time intervals, velocity
% maximum and deceleration of the motors.
% case=1: Each motor manages to reach its maximum speed.
% case=2: There is no time to reach the maximum speed of each motor.

function [caso,A,tt] = calculocoef(elem,vel,q0,qf,tmotor)
%---------------------------------------------------------------------
%Motor response times (acceleration and braking)
%---------------------------------------------------------------------
ti = tmotor(elem,1);
tf = tmotor(elem,2);

%---------------------------------------------------------------------
% Calculation of the coefficients.
%---------------------------------------------------------------------
if vel(elem) ~= 0
    %-Determination of the case
    desp = (qf(elem,1) - q0(elem,1));
    ttot = abs(desp/vel(elem));
    if ttot > (ti + tf)
        caso = 1;
    else
        caso = 2;
    end
    
%****************************** CASE 1********************************
    if caso == 1
        
    %time vector: Start - Max speed - Deceleration
    tt = [ ti ttot-(ti+tf) tf ];
    
    %Determination of polynomial coefficients
    A = zeros(3,5);
    
    % Coefficients of the position polynomial
    A(1,1) = q0(elem,1);
    A(1,2) = q0(elem,2)*tt(1);
    A(1,3) = q0(elem,3)*tt(1)^2/2;
    A(1,4) = tt(1)*vel(elem) - A(1,2) - 4*A(1,3)/3;
    A(1,5) = -tt(1)*vel(elem)/2 + A(1,2)/2 + A(1,3)/2;
    
    % acceleration polynomial coefficients
    A(3,1) = qf(elem,1);
    A(3,2) = qf(elem,2)*tt(3);
    A(3,3) = qf(elem,3)*tt(3)^2/2;
    A(3,4) = tt(3)*vel(elem) - A(3,2) + 4*A(3,3)/3;
    A(3,5) = (tt(3)*vel(elem) - A(3,2) + A(3,3))/2;
    
    %Space traveled in the previous intervals
    x1 = A(1,2) + A(1,3) + A(1,4) + A(1,5);
    x3 = A(3,2) - A(3,3) + A(3,4) - A(3,5);
    x2 = qf(elem,1) - q0(elem,1) - ( x1 + x3);
    
    % Real time at maximum speed.
    tt(2) = x2/vel(elem);
    
    %Coefficients of the velocity polynomial.
    A(2,1) = A(1,1) + A(1,2) + A(1,3) + A(1,4) + A(1,5);
    A(2,2) = vel(elem)*tt(2);
    
%******************************** CASE 2******************************
    elseif caso == 2
    t = (ti + tf)/2;
    tt = [ t t ];
    A = zeros(2,5);

    % engine start.
    A(1,1) = q0(elem,1);
    A(1,2) = q0(elem,2)*tt(1);
    A(1,3) = q0(elem,3)*tt(1)^2/2;

    % motor deceleration.
    A(2,1) = qf(elem,1);
    A(2,2) = qf(elem,2)*tt(2);
    A(2,3) = qf(elem,3)*tt(2)^2/2;

    % The polynomials for this case are of fifth order.
    B = [    1            1             1            -1;
          6/tt(1)^2    12/tt(1)^2       0             0;
          3/tt(1)       4/tt(1)     -3/tt(2)       4/tt(2);
             0            0        -6/tt(2)^2   12/tt(2)^2 ];
  
    b = [ -A(1,1) + A(1,2) + A(1,3) + A(2,1) - A(2,2) + A(2,3);
                         -2*A(1,3)/tt(1)^2;
          -(A(1,2) + 2*A(1,3))/tt(1) + (A(2,2) - 2*A(2,3))/tt(2);
                         -2*A(2,3)/tt(2)^2 ];
                
    x = inv(B)*b;

    % Coefficients 4 and 5 of each segment.
    A(1,4) = x(1);
    A(1,5) = x(2);
    A(2,4) = x(3);
    A(2,5) = x(4);
    end

% For when the motor does not move.
else
caso = 2;
tt = [0 0 0];
A = zeros(2,5);
end

return

end