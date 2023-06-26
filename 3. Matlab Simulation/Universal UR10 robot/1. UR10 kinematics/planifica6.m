% PLANIFICA6 Path planning
% MAT_Q = PLANIFICA6(P1, P2, N, S, A, CODO, NPUNTOS) Perform a 
% straight-line path planning from Cartesian coordinate P1 to P2, such 
% that the end effector of the manipulator has the orientation expressed 
% by [N S A]. When CODO = 1, it indicates an upward elbow configuration of 
% the robot, meaning that joint 3 is positioned above joint 2. On the other
% hand, when CODO = -1, it indicates a downward elbow configuration, 
% meaning that joint 2 is positioned above joint 3. NPPOINTS indicates the 
% number of points into which the trajectory is divided.
% MAT_Q returns the joint coordinates, stored in columns, corresponding to 
% each of the Cartesian points into which the trajectory is divided.
% MAT_Q is a matrix of NPPOINTS + 2 columns and 3 rows.
%

function mat_q = planifica6(p1, p2, n, s, a, codo, npuntos)

% Unit vector calculation
u = p2-p1;
mu = sqrt(u(1)^2+u(2)^2+u(3)^2);
u = (1/mu)*u;

% Calculation of the distance between points
d = mu/(npuntos+1);

for i=0:(npuntos+1)
    % Calculation of the current Cartesian position of the manipulator hand
    p = p1+(i*d)*u;
    T = [n s a p];
    
    % Calculation of joint coordinates
    q = inversekinematic6(T,codo);
    mat_q(:,i+1) = q;
end

end