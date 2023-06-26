% RI0FI Vector ri0fi.
% Y = RI0FI(RI0AI, MI) calculates the vector 3x1 of forces acting on element i at the center of mass.
% on element i at the center of mass. RI0AI represents the 
% linear acceleration at the center of mass of element i. MI is the 
% mass of element i.
%

function y = ri0fi(ri0ai, mi)

y = ri0ai*mi;
