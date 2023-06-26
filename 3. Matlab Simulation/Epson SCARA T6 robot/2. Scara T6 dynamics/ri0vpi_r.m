% RI0VPI_R Vector ri0vpi_r.
% Y = RI0VPI_R(IRI_1, I_1R0VPI_1, IR0WPI, IR0WI, IR0PI) calculates the 
% vector 3x1 representing the linear acceleration of the joint
% rotational % with respect to the i-th coordinate system. IRI_1 
% is the rotation matrix from the i-th coordinate system to the (i-1)-th coordinate system. 
% I_1R0VPI_1 % is the 3x1 linear acceleration vector of the joint. 
% (i-1)-th. IR0WPI is the 3x1 angular acceleration vector of the joint 
% i-th %. IR0WI is the 3x1 vector of angular velocity of the joint 
% i-th %. IRIPI is the 3x1 vector of localization of (xi,yi,zi) from the 
% origin of (xi-1,yi-1,zi-1) with respect to the i-th coordinate system.
%

function y = ri0vpi_r(iri_1, i_1r0vpi_1, ir0wpi, ir0wi, ir0pi)

z = [0; 0; 1];
a = cross(ir0wpi, ir0pi);
b = cross(ir0wi,  ir0pi);
c = cross(ir0wi, b);
y = (a + c + iri_1*i_1r0vpi_1);

end

