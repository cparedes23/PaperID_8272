% RI0VPI_P Vector ri0vpi_p.
% Y = RI0VPI_P(IRI_1, I_1R0VPI_1, IR0WPI, IR0WI, IR0PI, IR0PI, QPI, QPPI) calculates the 
% vector 3x1 representing the linear acceleration of the joint
% of the i-th prismatic joint with respect to the i-th coordinate system. IRI_1 
% is the rotation matrix from the i-th coordinate system to the (i-1)-th coordinate system. 
% I_1R0VPI_1 % is the 3x1 linear acceleration vector of the joint. 
% (i-1)-th. IR0WPI is the 3x1 angular acceleration vector of the joint 
% i-th %. IR0WI is the 3x1 vector of angular velocity of the joint 
% i-th %. IR0PI is the 3x1 vector of localization of (xi,yi,zi) from the 
% origin of (xi-1,yi-1,zi-1) with respect to the i-th coordinate system.
% QPI is the magnitude of the angular velocity of the i-th joint with respect to the i-th coordinate system. 
% of element i-1. QPPI is the magnitude of the angular acceleration of the i-th joint with respect to the i-th element. 
% of the i-th joint with respect to the i-1th element.
%

function y = ri0vpi_p(iri_1, i_1r0vpi_1, ir0wpi, ir0wi, ir0pi, qpi, qppi)

z = [0; 0; 1];
a = ir0wi;
b = cross(a, ir0pi);
c = cross(a, b);
y = iri_1*(z*qppi + i_1r0vpi_1) + cross(ir0wpi, ir0pi) + 2*(cross(ir0wi, ((iri_1)*(z*qpi)))) + c;
