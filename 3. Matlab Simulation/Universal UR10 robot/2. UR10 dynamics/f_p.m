% F_P Vector of joint forces.
% Y = F_P(IRI_1, IR0FIA, QPI, BI) calculates the vector 3x1 representing 
% the input force to joint i. IRI_1 is the rotation matrix 
% from the i-th coordinate system to the (i-1)-th. IR0FIA is the vector 
% 3x1 of force exerted on element i by element i-1. QPI is 
% the magnitude of the velocity of the i-th joint with respect to the (i-1)-th element. 
% i-1. BI is the viscous damping coefficient for the i-th joint.
%

function y = f_p(iri_1, ir0fia, qpi, bi);

z = [0; 0; 1];
y = ir0fia'*(iri_1*z) + bi*qpi;
