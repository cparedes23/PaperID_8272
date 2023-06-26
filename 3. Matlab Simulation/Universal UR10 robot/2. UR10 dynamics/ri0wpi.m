% RI0WPI Vector ri0wpi.
% Y = RI0WPI(IRI_1, I_1R0WPI_1, I_1R0WI_1, QPI, QPPI) calculates vector 
% 3x1 representing the angular acceleration of the i-th joint 
% with respect to the i-th coordinate system. IRI_1 is the rotation matrix 
% from the i-th coordinate system to the (i-1)-th. I_1R0WPI_1 is the vector 
% 3x1 angular acceleration vector of the (i-1)-th joint. I_1R0WI_1 is the 
% 3x1 vector of angular velocity of the (i-1)-th joint. QPI is the 
% magnitude of the angular velocity of the i-th joint with respect to the i-1 element. 
% element i-1. QPPI is the magnitude of the angular acceleration of the i-th joint with respect to the i-th element. 
% i-th joint with respect to the i-1th element.

function y = ri0wpi(iri_1, i_1r0wpi_1, i_1r0wi_1, qpi, qppi)
						       
z = [0; 0; 1];
a = cross(i_1r0wi_1,z*qpi);
y = iri_1*(i_1r0wpi_1 + z*qppi + a);



