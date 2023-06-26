% RI0SI Vector ri0si.
% Y = RI0SI(A, D, ALPHA, FACTOR) calculates the vector 3x1 representing 
% the position of the center of mass of element i with respect to its coordinate system using the Denavit-Hartenberg parameters. 
% of coordinates using the Denavit-Hartenberg parameters A, D
% and ALPHA. FACTOR is a scalar used to locate the center of mass.

function y = ri0si(a,d,alfa,factor)

y = [		factor*a
	  factor*d*sin(alfa)
	  factor*d*cos(alfa)];
  
end
