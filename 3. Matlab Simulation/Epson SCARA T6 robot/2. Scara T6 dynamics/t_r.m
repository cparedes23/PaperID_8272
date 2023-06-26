% T_R Vector of joint pairs.
% Y = T_R(IRI_1, IR0NIA, QPI, BI) calculates the 3x1 vector that represents 
% the input torque to joint i. IRI_1 is the rotation matrix 
% from the i-th coordinate system to the (i-1)-th. IR0NIA is the vector 
% 3x1 of moment exerted on element i by element i-1. QPI is 
% the magnitude of the angular velocity of the i-th joint with respect to the (i-1)-th element. 
% of element i-1. BI is the viscous damping coefficient for
% of the i-th joint.
%


function y = t_r(iri_1, ir0nia, qpi, bi)

z = [0; 0; 1];
y = ir0nia'*(iri_1*z) + bi*qpi;
