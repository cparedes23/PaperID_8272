% RI0FIA Vector ri0fia.
% Y = RI0FIA(IRI1, I1R0FI1A, IR0FI) calculates the vector 3x1 which 
% represents the force exerted on element i by element (i-1). 
% element (i-1). IRI1 is the rotation matrix of the i-th coordinate system to the i-th element. 
% i-th coordinate system to the (i+1)-th. I1R0FI1A is the vector 3x1
% force exerted on element (i+1) by element i.
% IR0FI is the vector 3x1 % of forces acting on element (i) at the center of mass. 
% i at the center of mass.
%

function y = ri0fia(iri1, i1r0fi1a, ir0fi)

y = iri1*(i1r0fi1a) + ir0fi;

end





