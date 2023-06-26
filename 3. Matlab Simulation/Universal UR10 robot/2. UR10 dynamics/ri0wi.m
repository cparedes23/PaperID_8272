% RI0WI Vector ri0wi.
% Y = RI0WI(IRI_1, I_1R0WI_1, QPI) calculates the vector 3x1 representing 
% the angular velocity of the i-th joint with respect to the i-th coordinate system.
% i-th coordinate system. IRI_1 is the rotation matrix of the i-th coordinate system to the i-th coordinate system. 
% i-th coordinate system to the (i-1)-th coordinate system. I_1R0WI_1 is the 3x1 vector of angular velocity of the (i-1)-th joint. 
% angular velocity of the (i-1)-th joint. QPI is the magnitude of the velocity 
% angular % velocity of the i-th joint with respect to the i-1 element.

function y = ri0wi(iri_1, i_1r0wi_1, qpi)

z = [0; 0; 1];
y = iri_1*(i_1r0wi_1 + z*qpi);
