% RI0PI Vector ri0pi.
% Y = RI0PI(A, D, ALPHA) calculates the vector 3x1 that represents the
% location of (xi,yi,zi) from the origin of (xi-1,yi-1,zi-1) 
% with respect to the i-th coordinate system using the
% Denavit-Hartenberg parameters A, D and ALPHA.

function y = ri0pi(a,d,alfa)

y = [      a	  
   	d*sin(alfa)	 
   	d*cos(alfa)];
