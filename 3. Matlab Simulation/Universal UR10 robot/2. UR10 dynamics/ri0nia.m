% RI0NIA Vector ri0nia.
% Y = RI0NIA(IRI1, I1R0NI1A, I1R0NI1A, I1R0FI1A, IR0NI, IR0FI, I1R0PI, IR0PI, IR0SI) 
% calculates the vector 3x1 representing the moment exerted on element i by element i-1.
% i by element i-1. IRI1 is the rotation matrix of the i-th coordinate system at the i-th element. 
% i-th coordinate system to the (i+1)-th coordinate system. I1R0NI1A is the 3x1 vector of momentum 
% exerted on the (i+1)-element by the i-th element. I1R0FI1A is the vector 
% 3x1 vector of force exerted on element (i+1) by element i. IR0NI is 
% the vector 3x1 of external force acting on element i at the center of mass. 
% of mass. IR0FI is the 3x1 vector of external force acting on element i at the center of mass. 
% i at the center of mass. I1R0PI is the 3x1 vector of localization of the reference (i+1)-system.
% (i+1)-nth reference % system from the i-th. IR0PI is the 3x1 vector of 
% location of the i-th reference system from the (i-1)-th.
% IR0SI is the 3x1 vector of the position of the center of mass of the i-th element with respect to its coordinate system. 
% with respect to its coordinate system.
%

function y = ri0nia(iri1, i1r0ni1a, i1r0fi1a, ir0ni, ir0fi, i1r0pi, ir0pi, ir0si)

y = iri1*(i1r0ni1a + cross(i1r0pi, i1r0fi1a)) + cross((ir0pi+ir0si), ir0fi) + ir0ni;
