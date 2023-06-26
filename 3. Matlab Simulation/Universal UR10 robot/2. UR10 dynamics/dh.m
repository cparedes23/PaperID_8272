% DH Rotation matrix.
% R = DH(TETA, ALPHA) returns the 3x3 rotation matrix using 
% the Denavit-Hartenberg parameters TETA and ALPHA.
%

function R = dh(teta,alfa)

R = [cos(teta)  -cos(alfa)*sin(teta)   sin(alfa)*sin(teta)
     sin(teta)   cos(alfa)*cos(teta)  -sin(alfa)*cos(teta)
	         0              sin(alfa)             cos(alfa)];

